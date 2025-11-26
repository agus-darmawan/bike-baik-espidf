#include "trip_manager.h"
#include "config.h"
#include "sensor_manager.h"
#include "vehicle_performance.h"
#include "kalman_filter.h"
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

    ESP_LOGI(TAG, "Trip manager initialized");
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
