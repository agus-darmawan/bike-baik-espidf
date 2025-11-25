#include "sensor_manager.h"
#include "trip_manager.h"
#include "config.h"
#include "sim808.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  BIKE-BAIK IoT System with GPS & MQTT");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize NVS (required for WiFi/network)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize all sensors
    ESP_LOGI(TAG, "Initializing sensors...");
    sensor_manager_init();
    sensor_manager_start_tasks();
    
    // Initialize SIM808 module
    ESP_LOGI(TAG, "Initializing SIM808 GPS/GPRS module...");
    sim808_config_t sim_config = {
        .uart_port = SIM808_UART_PORT,
        .tx_pin = SIM808_TX_PIN,
        .rx_pin = SIM808_RX_PIN,
        .pwr_pin = SIM808_PWR_PIN,
        .rst_pin = SIM808_RST_PIN,
        .baud_rate = SIM808_BAUD_RATE,
        .apn = SIM808_APN,
        .apn_user = SIM808_APN_USER,
        .apn_pass = SIM808_APN_PASS
    };
    
    if (sim808_init(&sim_config) == ESP_OK) {
        ESP_LOGI(TAG, "SIM808 initialized");
        
        // Power on module
        if (sim808_power_on() == ESP_OK) {
            ESP_LOGI(TAG, "SIM808 powered on");
            
            // Connect to GPRS
            ESP_LOGI(TAG, "Connecting to GPRS...");
            if (sim808_gprs_connect() == ESP_OK) {
                ESP_LOGI(TAG, "GPRS connected");
                
                // Initialize HTTP
                sim808_http_init();
                
                // Initialize MQTT for RabbitMQ
                sim808_mqtt_init(RABBITMQ_BROKER, "bike-baik-client",
                               RABBITMQ_USERNAME, RABBITMQ_PASSWORD);
                sim808_mqtt_connect();
                
                ESP_LOGI(TAG, "Network services ready");
            } else {
                ESP_LOGW(TAG, "Failed to connect to GPRS");
            }
        } else {
            ESP_LOGW(TAG, "Failed to power on SIM808");
        }
    } else {
        ESP_LOGW(TAG, "Failed to initialize SIM808");
    }
    
    // Initialize and start trip manager
    ESP_LOGI(TAG, "Initializing trip manager...");
    trip_manager_init();
    trip_manager_start();
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "System started successfully!");
    ESP_LOGI(TAG, "Waiting for engine to start (battery voltage > %.2fV)...", BATTERY_VOLTAGE_ENGINE_ON);
    ESP_LOGI(TAG, "");

    // Main monitoring loop
    while (1) {
        sensor_data_t sensors = sensor_manager_get_data();
        
        // Display status every 5 seconds
        ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════╗");
        ESP_LOGI(TAG, "║              BIKE-BAIK SYSTEM STATUS                  ║");
        ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");
        
        // Battery & Trip Status
        ESP_LOGI(TAG, "║ Battery Voltage : %6.2f V  %s                ║", 
                 sensors.battery_voltage,
                 sensors.battery_voltage >= BATTERY_VOLTAGE_ENGINE_ON ? "[ENGINE ON ]" : "[ENGINE OFF]");
        
        if (trip_manager_is_trip_active()) {
            const char *trip_id = trip_manager_get_current_trip_id();
            ESP_LOGI(TAG, "║ Trip Status     : ACTIVE                            ║");
            ESP_LOGI(TAG, "║ Trip ID         : %-30s      ║", trip_id ? trip_id : "N/A");
        } else {
            ESP_LOGI(TAG, "║ Trip Status     : IDLE (waiting for engine start)  ║");
        }
        
        ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");
        
        // GPS Status
        sim808_gps_data_t gps;
        if (sim808_gps_get_data(&gps) == ESP_OK && gps.fix_valid) {
            ESP_LOGI(TAG, "║ GPS Fix         : VALID                              ║");
            ESP_LOGI(TAG, "║ Latitude        : %11.6f°                     ║", gps.latitude);
            ESP_LOGI(TAG, "║ Longitude       : %11.6f°                     ║", gps.longitude);
            ESP_LOGI(TAG, "║ Altitude        : %6.1f m                          ║", gps.altitude);
            ESP_LOGI(TAG, "║ Speed           : %6.1f km/h                       ║", gps.speed);
            ESP_LOGI(TAG, "║ Satellites      : %2d                                 ║", gps.satellites);
        } else {
            ESP_LOGI(TAG, "║ GPS Fix         : NO FIX                             ║");
        }
        
        ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");
        
        // Sensor Data
        ESP_LOGI(TAG, "║ Temperature     : %6.2f °C                         ║", sensors.temperature_c);
        ESP_LOGI(TAG, "║ Wheel RPM       : %6.1f rpm                        ║", sensors.rpm);
        ESP_LOGI(TAG, "║ Roll/Pitch/Yaw  : %6.1f° / %6.1f° / %6.1f°    ║",
                 sensors.roll, sensors.pitch, sensors.yaw);
        
        // Network Status
        ESP_LOGI(TAG, "╠════════════════════════════════════════════════════════╣");
        if (sim808_is_ready()) {
            int signal = sim808_get_signal_strength();
            ESP_LOGI(TAG, "║ GPRS Status     : %s                           ║",
                     sim808_gprs_is_connected() ? "CONNECTED" : "DISCONNECTED");
            ESP_LOGI(TAG, "║ Signal Strength : %2d/31                             ║", signal);
        } else {
            ESP_LOGI(TAG, "║ SIM808 Status   : NOT READY                          ║");
        }
        
        ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════╝");
        ESP_LOGI(TAG, "");
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
