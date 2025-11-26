/**
 * @file alert_manager.c
 * @author @agus-darmawan
 * @brief Vehicle alert detection and management implementation
 * @version 0.1
 * @date 2025-11-26
 */

#include "alert_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "ALERT_MGR";

/* Detection thresholds */
#define CRASH_ACCEL_THRESHOLD    30.0f   /**< Crash detection: >3g sudden impact */
#define FALL_ANGLE_THRESHOLD     45.0f   /**< Fall detection: >45° tilt */
#define BUMP_ACCEL_Z_THRESHOLD   20.0f   /**< Bump detection: >2g vertical */

/* Debounce to prevent multiple alerts */
#define ALERT_DEBOUNCE_MS        3000    /**< Minimum time between same alert type */

/* Alert manager context */
static struct {
    char serial_number[32];
    alert_callback_t callback;
    void *callback_user_data;
    
    /* Last alert timestamps for debouncing */
    uint64_t last_crash_time;
    uint64_t last_fall_time;
    uint64_t last_bump_time;
    
    /* Previous values for change detection */
    float prev_accel_magnitude;
    
    bool initialized;
} alert_ctx = {0};

/* ===== Helper Functions ===== */

static uint64_t get_timestamp_ms(void)
{
    return esp_timer_get_time() / 1000ULL;
}

static bool is_debounce_elapsed(uint64_t last_time)
{
    uint64_t now = get_timestamp_ms();
    return (now - last_time) >= ALERT_DEBOUNCE_MS;
}

static float calculate_acceleration_magnitude(float ax, float ay, float az)
{
    return sqrtf(ax*ax + ay*ay + az*az);
}

static void trigger_alert(alert_type_t type, float accel_x, float accel_y, float accel_z,
                         float roll, float pitch, float yaw,
                         float speed, float latitude, float longitude)
{
    if (!alert_ctx.callback) {
        ESP_LOGW(TAG, "Alert detected but no callback registered");
        return;
    }
    
    alert_data_t alert = {
        .type = type,
        .timestamp = get_timestamp_ms(),
        .accel_x = accel_x,
        .accel_y = accel_y,
        .accel_z = accel_z,
        .roll = roll,
        .pitch = pitch,
        .yaw = yaw,
        .speed = speed,
        .latitude = latitude,
        .longitude = longitude
    };
    
    strncpy(alert.serial_number, alert_ctx.serial_number, sizeof(alert.serial_number) - 1);
    
    ESP_LOGW(TAG, "ALERT DETECTED: %s (SN: %s)", 
             alert_type_to_string(type), alert_ctx.serial_number);
    
    alert_ctx.callback(&alert, alert_ctx.callback_user_data);
}

/* ===== Public API Implementation ===== */

void alert_manager_init(const char *serial_number)
{
    ESP_LOGI(TAG, "Initializing alert manager");
    
    memset(&alert_ctx, 0, sizeof(alert_ctx));
    
    if (serial_number) {
        strncpy(alert_ctx.serial_number, serial_number, sizeof(alert_ctx.serial_number) - 1);
    } else {
        strcpy(alert_ctx.serial_number, "UNKNOWN");
    }
    
    alert_ctx.initialized = true;
    
    ESP_LOGI(TAG, "Alert manager initialized (SN: %s)", alert_ctx.serial_number);
}

void alert_manager_register_callback(alert_callback_t callback, void *user_data)
{
    alert_ctx.callback = callback;
    alert_ctx.callback_user_data = user_data;
    ESP_LOGI(TAG, "Alert callback registered");
}

void alert_manager_update(float accel_x, float accel_y, float accel_z,
                         float roll, float pitch, float yaw,
                         float speed, float latitude, float longitude)
{
    if (!alert_ctx.initialized) {
        return;
    }
    
    float accel_magnitude = calculate_acceleration_magnitude(accel_x, accel_y, accel_z);
    
    /* ===== CRASH DETECTION ===== */
    /* Detect sudden high acceleration (>3g) indicating collision */
    if (accel_magnitude > CRASH_ACCEL_THRESHOLD) {
        if (is_debounce_elapsed(alert_ctx.last_crash_time)) {
            trigger_alert(ALERT_TYPE_CRASH, accel_x, accel_y, accel_z,
                         roll, pitch, yaw, speed, latitude, longitude);
            alert_ctx.last_crash_time = get_timestamp_ms();
        }
    }
    
    /* ===== FALL DETECTION ===== */
    /* Detect if vehicle tipped over (>45° roll or pitch) */
    if (fabsf(roll) > FALL_ANGLE_THRESHOLD || fabsf(pitch) > FALL_ANGLE_THRESHOLD) {
        if (is_debounce_elapsed(alert_ctx.last_fall_time)) {
            trigger_alert(ALERT_TYPE_FALL, accel_x, accel_y, accel_z,
                         roll, pitch, yaw, speed, latitude, longitude);
            alert_ctx.last_fall_time = get_timestamp_ms();
        }
    }
    
    /* ===== BUMP DETECTION ===== */
    /* Detect large vertical acceleration (>2g) indicating bump/pothole */
    if (fabsf(accel_z) > BUMP_ACCEL_Z_THRESHOLD) {
        if (is_debounce_elapsed(alert_ctx.last_bump_time)) {
            trigger_alert(ALERT_TYPE_BUMP, accel_x, accel_y, accel_z,
                         roll, pitch, yaw, speed, latitude, longitude);
            alert_ctx.last_bump_time = get_timestamp_ms();
        }
    }
    
    /* Update previous values */
    alert_ctx.prev_accel_magnitude = accel_magnitude;
}

const char* alert_type_to_string(alert_type_t type)
{
    switch (type) {
        case ALERT_TYPE_CRASH: return "CRASH";
        case ALERT_TYPE_FALL:  return "FALL";
        case ALERT_TYPE_BUMP:  return "BUMP";
        default:               return "UNKNOWN";
    }
}

const char* alert_type_to_topic(alert_type_t type)
{
    switch (type) {
        case ALERT_TYPE_CRASH: return "vehicle.alert.crash";
        case ALERT_TYPE_FALL:  return "vehicle.alert.fall";
        case ALERT_TYPE_BUMP:  return "vehicle.alert.bump";
        default:               return "vehicle.alert.unknown";
    }
}