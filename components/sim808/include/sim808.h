/**
 * @file sim808.h
 * @author @agus-darmawan
 * @brief SIM808 GPS/GPRS/GSM module driver with GPS, HTTP, and MQTT support
 * @version 0.1
 * @date 2025-11-25
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== GPS Data Structure ===== */
typedef struct {
    bool fix_valid;           /**< GPS fix status */
    float latitude;           /**< Latitude in decimal degrees */
    float longitude;          /**< Longitude in decimal degrees */
    float altitude;           /**< Altitude in meters */
    float speed;              /**< Speed in km/h */
    float course;             /**< Course over ground in degrees */
    uint8_t satellites;       /**< Number of satellites */
    float hdop;               /**< Horizontal dilution of precision */
    struct {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    } utc_time;
} sim808_gps_data_t;

/* ===== HTTP Response Structure ===== */
typedef struct {
    int status_code;          /**< HTTP status code */
    char *data;               /**< Response data (must be freed by caller) */
    size_t data_len;          /**< Response data length */
} sim808_http_response_t;

/* ===== MQTT Event Callback ===== */
typedef void (*sim808_mqtt_event_cb_t)(const char *topic, const char *data, size_t len, void *user_data);

/* ===== Configuration Structure ===== */
typedef struct {
    uart_port_t uart_port;    /**< UART port number */
    int tx_pin;               /**< TX GPIO pin */
    int rx_pin;               /**< RX GPIO pin */
    int pwr_pin;              /**< Power control pin (-1 if not used) */
    int rst_pin;              /**< Reset pin (-1 if not used) */
    uint32_t baud_rate;       /**< UART baud rate */
    
    /* APN settings */
    const char *apn;          /**< APN for GPRS */
    const char *apn_user;     /**< APN username (NULL if not required) */
    const char *apn_pass;     /**< APN password (NULL if not required) */
} sim808_config_t;

/* ===== Initialization ===== */

/**
 * @brief Initialize SIM808 module
 * @param config Pointer to configuration structure
 * @return ESP_OK on success
 */
esp_err_t sim808_init(const sim808_config_t *config);

/**
 * @brief Deinitialize SIM808 module
 */
void sim808_deinit(void);

/**
 * @brief Power on SIM808 module
 * @return ESP_OK on success
 */
esp_err_t sim808_power_on(void);

/**
 * @brief Power off SIM808 module
 * @return ESP_OK on success
 */
esp_err_t sim808_power_off(void);

/**
 * @brief Check if module is ready
 * @return true if ready, false otherwise
 */
bool sim808_is_ready(void);

/* ===== GPS Functions ===== */

/**
 * @brief Enable GPS
 * @return ESP_OK on success
 */
esp_err_t sim808_gps_enable(void);

/**
 * @brief Disable GPS
 * @return ESP_OK on success
 */
esp_err_t sim808_gps_disable(void);

/**
 * @brief Get GPS data
 * @param data Pointer to GPS data structure
 * @return ESP_OK on success
 */
esp_err_t sim808_gps_get_data(sim808_gps_data_t *data);

/* ===== Network Functions ===== */

/**
 * @brief Connect to GPRS network
 * @return ESP_OK on success
 */
esp_err_t sim808_gprs_connect(void);

/**
 * @brief Disconnect from GPRS network
 * @return ESP_OK on success
 */
esp_err_t sim808_gprs_disconnect(void);

/**
 * @brief Check GPRS connection status
 * @return true if connected, false otherwise
 */
bool sim808_gprs_is_connected(void);

/**
 * @brief Get signal strength
 * @return Signal strength (0-31, 99 if unknown)
 */
int sim808_get_signal_strength(void);

/* ===== HTTP Functions ===== */

/**
 * @brief Initialize HTTP service
 * @return ESP_OK on success
 */
esp_err_t sim808_http_init(void);

/**
 * @brief Terminate HTTP service
 * @return ESP_OK on success
 */
esp_err_t sim808_http_term(void);

/**
 * @brief Perform HTTP GET request
 * @param url URL to request
 * @param response Pointer to response structure (caller must free response->data)
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t sim808_http_get(const char *url, sim808_http_response_t *response, uint32_t timeout_ms);

/**
 * @brief Perform HTTP POST request
 * @param url URL to post to
 * @param data Data to post
 * @param data_len Data length
 * @param content_type Content-Type header (e.g., "application/json")
 * @param response Pointer to response structure (caller must free response->data)
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t sim808_http_post(const char *url, const char *data, size_t data_len,
                           const char *content_type, sim808_http_response_t *response,
                           uint32_t timeout_ms);

/**
 * @brief Free HTTP response data
 * @param response Pointer to response structure
 */
void sim808_http_free_response(sim808_http_response_t *response);

/* ===== MQTT Functions ===== */

/**
 * @brief Initialize MQTT client
 * @param broker MQTT broker URL (e.g., "tcp://broker.example.com:1883")
 * @param client_id Client ID
 * @param username Username (NULL if not required)
 * @param password Password (NULL if not required)
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_init(const char *broker, const char *client_id,
                           const char *username, const char *password);

/**
 * @brief Connect to MQTT broker
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_connect(void);

/**
 * @brief Disconnect from MQTT broker
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_disconnect(void);

/**
 * @brief Check MQTT connection status
 * @return true if connected, false otherwise
 */
bool sim808_mqtt_is_connected(void);

/**
 * @brief Publish message to MQTT topic
 * @param topic Topic to publish to
 * @param data Data to publish
 * @param data_len Data length
 * @param qos QoS level (0, 1, or 2)
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_publish(const char *topic, const char *data, size_t data_len, int qos);

/**
 * @brief Subscribe to MQTT topic
 * @param topic Topic to subscribe to
 * @param qos QoS level (0, 1, or 2)
 * @param callback Callback function for received messages
 * @param user_data User data passed to callback
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_subscribe(const char *topic, int qos,
                                sim808_mqtt_event_cb_t callback, void *user_data);

/**
 * @brief Unsubscribe from MQTT topic
 * @param topic Topic to unsubscribe from
 * @return ESP_OK on success
 */
esp_err_t sim808_mqtt_unsubscribe(const char *topic);

#ifdef __cplusplus
}
#endif
