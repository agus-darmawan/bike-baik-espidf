#include "trip_manager.h"
#include "config.h"
#include "sensor_manager.h"
#include "vehicle_performance.h"
#include "kalman_filter.h"
#include "alert_manager.h"
#include "sim808.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "TRIP_MANAGER";

/* ===== Trip State ===== */
typedef enum {
    TRIP_STATE_IDLE,
    TRIP_STATE_STARTING,
    TRIP_STATE_ACTIVE,
    TRIP_STATE_STOPPING
} trip_state_t;

static struct {
    trip_state_t state;
    char trip_id[64];
    uint32_t trip_start_time;
    vehicle_params_t vehicle_params;

    // Sensor fusion
    vehicle_state_t fused_state;
    float last_altitude;

    // Timing
    TickType_t last_update_time;
} trip_ctx = {0};

/* ===== Alert Callback ===== */

static void alert_handler(const alert_data_t *alert, void *user_data)
{
    ESP_LOGW(TAG, "===== ALERT DETECTED =====");
    ESP_LOGW(TAG, "Type: %s", alert_type_to_string(alert->type));
    ESP_LOGW(TAG, "Serial Number: %s", alert->serial_number);
    ESP_LOGW(TAG, "Location: %.6f, %.6f", alert->latitude, alert->longitude);
    ESP_LOGW(TAG, "Speed: %.2f m/s", alert->speed);
    ESP_LOGW(TAG, "Accel: X=%.2f Y=%.2f Z=%.2f m/s²", 
             alert->accel_x, alert->accel_y, alert->accel_z);
    ESP_LOGW(TAG, "Orientation: Roll=%.1f Pitch=%.1f Yaw=%.1f°", 
             alert->roll, alert->pitch, alert->yaw);
    ESP_LOGW(TAG, "===========================");
    
    // Send alert to RabbitMQ
    char payload[512];
    int len = snprintf(
        payload, sizeof(payload),
        "{"
        "\"type\":\"%s\","
        "\"serial_number\":\"%s\","
        "\"timestamp\":%" PRIu64 ","
        "\"latitude\":%.6f,"
        "\"longitude\":%.6f,"
        "\"speed\":%.2f,"
        "\"accel_x\":%.2f,"
        "\"accel_y\":%.2f,"
        "\"accel_z\":%.2f,"
        "\"roll\":%.2f,"
        "\"pitch\":%.2f,"
        "\"yaw\":%.2f"
        "}",
        alert_type_to_string(alert->type),
        alert->serial_number,
        alert->timestamp,
        alert->latitude,
        alert->longitude,
        alert->speed,
        alert->accel_x,
        alert->accel_y,
        alert->accel_z,
        alert->roll,
        alert->pitch,
        alert->yaw
    );
    
    const char *topic = alert_type_to_topic(alert->type);
    
    esp_err_t ret = sim808_mqtt_publish(topic, payload, len, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Alert sent to %s successfully", topic);
    } else {
        ESP_LOGE(TAG, "Failed to send alert to %s", topic);
    }
}

/* ===== Helper Functions ===== */

static void generate_trip_id(char *buffer, size_t len)
{
    time_t now = time(NULL);
    snprintf(buffer, len, "TRIP_%lld", (long long) now);
}

static esp_err_t fetch_vehicle_params(void)
{
    ESP_LOGI(TAG, "Fetching vehicle parameters from API");

    char url[256];
    snprintf(url, sizeof(url), "%s/vehicle/parameters", API_SERVER_URL);

    sim808_http_response_t response;
    esp_err_t ret = sim808_http_get(url, &response, 10000);

    if (ret == ESP_OK && response.status_code == 200) {

        sscanf(response.data,
               "{\"wheelbase\":%f,\"wheel_circumference\":%f,\"mass\":%f}",
               &trip_ctx.vehicle_params.wheelbase,
               &trip_ctx.vehicle_params.wheel_circumference,
               &trip_ctx.vehicle_params.mass);

        ESP_LOGI(TAG,
                 "Vehicle params: wheelbase=%.2f, circumference=%.2f, mass=%.1f",
                 trip_ctx.vehicle_params.wheelbase,
                 trip_ctx.vehicle_params.wheel_circumference,
                 trip_ctx.vehicle_params.mass);

        sim808_http_free_response(&response);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Failed to fetch vehicle params, using defaults");
        trip_ctx.vehicle_params.wheelbase = VEHICLE_WHEELBASE_DEFAULT;
        trip_ctx.vehicle_params.wheel_circumference = VEHICLE_WHEEL_CIRCUMFERENCE_DEFAULT;
        trip_ctx.vehicle_params.mass = VEHICLE_MASS_DEFAULT;

        if (ret == ESP_OK) {
            sim808_http_free_response(&response);
        }
        return ESP_FAIL;
    }
}

static esp_err_t send_performance_data_to_rabbitmq(void)
{
    ESP_LOGI(TAG, "Sending performance data to RabbitMQ");

    vehicle_performance_t perf = performance_get_data();

    char payload[1024];

    int len = snprintf(
        payload, sizeof(payload),
        "{"
        "\"trip_id\":\"%s\","
        "\"serial_number\":\"%s\","
        "\"total_distance_km\":%.2f,"
        "\"average_speed\":%.2f,"
        "\"max_speed\":%.2f,"
        "\"rear_tire_wear\":%.2f,"
        "\"front_tire_wear\":%.2f,"
        "\"rear_brake_wear\":%.2f,"
        "\"front_brake_wear\":%.2f,"
        "\"chain_wear\":%.2f,"
        "\"engine_oil_wear\":%.2f,"
        "\"engine_wear\":%.2f,"
        "\"air_filter_wear\":%.2f,"
        "\"trip_count\":%" PRIu32
        "}",
        perf.order_id,
        VEHICLE_SERIAL_NUMBER,
        perf.total_distance_km,
        perf.average_speed,
        perf.max_speed,
        perf.s_rear_tire,
        perf.s_front_tire,
        perf.s_rear_brake_pad,
        perf.s_front_brake_pad,
        perf.s_chain_or_cvt,
        perf.s_engine_oil,
        perf.s_engine,
        perf.s_air_filter,
        perf.trip_count
    );

    esp_err_t ret = sim808_mqtt_publish(RABBITMQ_TOPIC, payload, len, 1);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Performance data sent successfully");
    } else {
        ESP_LOGE(TAG, "Failed to send performance data");
    }

    return ret;
}

static void perform_sensor_fusion(float dt)
{
    sensor_data_t sensors = sensor_manager_get_data();

    gps_measurement_t gps_meas = {0};
    imu_measurement_t imu_meas = {0};
    odometry_measurement_t odo_meas = {0};

    sim808_gps_data_t gps_data;
    if (sim808_gps_get_data(&gps_data) == ESP_OK && gps_data.fix_valid) {
        gps_meas.latitude = gps_data.latitude;
        gps_meas.longitude = gps_data.longitude;
        gps_meas.altitude = gps_data.altitude;
        gps_meas.speed = gps_data.speed / 3.6f;
        gps_meas.course = gps_data.course;
        gps_meas.valid = true;

        trip_ctx.last_altitude = gps_data.altitude;
    }

    imu_meas.ax = sensors.accel_x;
    imu_meas.ay = sensors.accel_y;
    imu_meas.az = sensors.accel_z;
    imu_meas.gx = sensors.gyro_x;
    imu_meas.gy = sensors.gyro_y;
    imu_meas.gz = sensors.gyro_z;
    imu_meas.roll = sensors.roll;
    imu_meas.pitch = sensors.pitch;
    imu_meas.yaw = sensors.yaw;

    if (sensors.rpm > 0) {
        float wheel_speed = (sensors.rpm / 60.0f) * trip_ctx.vehicle_params.wheel_circumference;
        odo_meas.speed = wheel_speed;
        odo_meas.distance = wheel_speed * dt;
        odo_meas.valid = true;
    }

    kalman_predict(dt);

    if (gps_meas.valid) {
        kalman_update_gps(&gps_meas);
    }

    kalman_update_imu(&imu_meas);

    if (odo_meas.valid) {
        kalman_update_odometry(&odo_meas);
    }

    kalman_get_state(&trip_ctx.fused_state);
    
    // Update alert detection with current sensor data
    sim808_gps_data_t gps_for_alert;
    float lat = 0.0f, lon = 0.0f;
    if (sim808_gps_get_data(&gps_for_alert) == ESP_OK && gps_for_alert.fix_valid) {
        lat = gps_for_alert.latitude;
        lon = gps_for_alert.longitude;
    }
    
    float speed = sqrtf(trip_ctx.fused_state.vx * trip_ctx.fused_state.vx +
                       trip_ctx.fused_state.vy * trip_ctx.fused_state.vy);
    
    alert_manager_update(
        sensors.accel_x, sensors.accel_y, sensors.accel_z,
        sensors.roll, sensors.pitch, sensors.yaw,
        speed, lat, lon
    );
}

static void update_vehicle_performance(float dt)
{
    float v_end = sqrtf(trip_ctx.fused_state.vx * trip_ctx.fused_state.vx +
                        trip_ctx.fused_state.vy * trip_ctx.fused_state.vy);

    float s_real = v_end * dt;

    sim808_gps_data_t gps_data;
    float h = 0.0f;
    if (sim808_gps_get_data(&gps_data) == ESP_OK && gps_data.fix_valid) {
        h = gps_data.altitude - trip_ctx.last_altitude;
        trip_ctx.last_altitude = gps_data.altitude;
    }

    sensor_data_t sensors = sensor_manager_get_data();
    float temp = sensors.temperature_c;

    bool is_braking = sensors.digital_input_state;

    if (is_braking) {
        performance_with_brake_update(s_real, h, v_end, temp, (int)dt,
                                      trip_ctx.vehicle_params.mass,
                                      trip_ctx.vehicle_params.wheelbase);
    } else {
        performance_without_brake_update(s_real, h, v_end, temp, (int)dt);
    }
}

/* ===== Trip Manager Task ===== */

static void trip_manager_task(void *arg)
{
    ESP_LOGI(TAG, "Trip manager task started");

    while (1) {
        sensor_data_t sensors = sensor_manager_get_data();
        float voltage = sensors.battery_voltage;

        switch (trip_ctx.state) {

            case TRIP_STATE_IDLE:
                if (voltage >= BATTERY_VOLTAGE_ENGINE_ON) {
                    ESP_LOGI(TAG, "Engine ON detected, starting trip");
                    trip_ctx.state = TRIP_STATE_STARTING;
                }
                break;

            case TRIP_STATE_STARTING:
                fetch_vehicle_params();

                if (sim808_gps_enable() != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to enable GPS");
                }

                kalman_config_t kf_config = {
                    .process_noise_pos = 0.1f,
                    .process_noise_vel = 0.1f,
                    .process_noise_heading = 0.01f,
                    .measurement_noise_gps = 5.0f,
                    .measurement_noise_imu = 0.1f,
                    .measurement_noise_odo = 0.5f
                };
                kalman_init(&kf_config);

                generate_trip_id(trip_ctx.trip_id, sizeof(trip_ctx.trip_id));
                performance_start_tracking(trip_ctx.trip_id);

                trip_ctx.trip_start_time = xTaskGetTickCount();
                trip_ctx.last_update_time = trip_ctx.trip_start_time;

                trip_ctx.state = TRIP_STATE_ACTIVE;

                ESP_LOGI(TAG, "Trip started: %s", trip_ctx.trip_id);
                break;

            case TRIP_STATE_ACTIVE:
                if (voltage < BATTERY_VOLTAGE_ENGINE_ON) {
                    ESP_LOGI(TAG, "Engine OFF detected, stopping trip");
                    trip_ctx.state = TRIP_STATE_STOPPING;
                } else {
                    TickType_t now = xTaskGetTickCount();
                    float dt = (now - trip_ctx.last_update_time) / configTICK_RATE_HZ;
                    trip_ctx.last_update_time = now;

                    perform_sensor_fusion(dt);

                    update_vehicle_performance(dt);
                }
                break;

            case TRIP_STATE_STOPPING:
                performance_stop_tracking();

                send_performance_data_to_rabbitmq();

                sim808_gps_disable();

                trip_ctx.state = TRIP_STATE_IDLE;
                memset(trip_ctx.trip_id, 0, sizeof(trip_ctx.trip_id));

                ESP_LOGI(TAG, "Trip stopped and data sent");
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(TRIP_UPDATE_INTERVAL_MS));
    }
}

/* ===== Public API ===== */

void trip_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing trip manager");

    memset(&trip_ctx, 0, sizeof(trip_ctx));
    trip_ctx.state = TRIP_STATE_IDLE;

    trip_ctx.vehicle_params.wheelbase = VEHICLE_WHEELBASE_DEFAULT;
    trip_ctx.vehicle_params.wheel_circumference = VEHICLE_WHEEL_CIRCUMFERENCE_DEFAULT;
    trip_ctx.vehicle_params.mass = VEHICLE_MASS_DEFAULT;

    performance_init();
    
    // Initialize alert manager
    alert_manager_init(VEHICLE_SERIAL_NUMBER);
    alert_manager_register_callback(alert_handler, NULL);

    ESP_LOGI(TAG, "Trip manager initialized (SN: %s)", VEHICLE_SERIAL_NUMBER);
}

void trip_manager_start(void)
{
    xTaskCreate(trip_manager_task, "trip_mgr", 8192, NULL, 5, NULL);
}

void trip_manager_get_vehicle_params(vehicle_params_t *params)
{
    if (params) {
        *params = trip_ctx.vehicle_params;
    }
}

bool trip_manager_is_trip_active(void)
{
    return (trip_ctx.state == TRIP_STATE_ACTIVE);
}

const char* trip_manager_get_current_trip_id(void)
{
    return (trip_ctx.state == TRIP_STATE_ACTIVE) ? trip_ctx.trip_id : NULL;
}