#include "sim808.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SIM808_HTTP";

// External declarations from sim808.c
extern esp_err_t sim808_send_at_expect(const char *cmd, const char *expected, uint32_t timeout_ms);
extern void sim808_clear_rx_buffer(void);
extern char *sim808_get_rx_buffer(size_t *len); // Need to add this function

esp_err_t sim808_http_init(void)
{
    ESP_LOGI(TAG, "Initializing HTTP");
    
    if (sim808_send_at_expect("AT+HTTPINIT", "OK", 5000) != ESP_OK) {
        ESP_LOGW(TAG, "HTTP may already be initialized, terminating first");
        sim808_http_term();
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (sim808_send_at_expect("AT+HTTPINIT", "OK", 5000) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    
    // Set HTTP parameters
    sim808_send_at_expect("AT+HTTPPARA=\"CID\",1", "OK", 2000);
    
    return ESP_OK;
}

esp_err_t sim808_http_term(void)
{
    ESP_LOGI(TAG, "Terminating HTTP");
    return sim808_send_at_expect("AT+HTTPTERM", "OK", 5000);
}

esp_err_t sim808_http_get(const char *url, sim808_http_response_t *response, uint32_t timeout_ms)
{
    if (!url || !response) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "HTTP GET: %s", url);
    
    memset(response, 0, sizeof(sim808_http_response_t));
    
    // Set URL
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (sim808_send_at_expect(cmd, "OK", 2000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set URL");
        return ESP_FAIL;
    }
    
    // Perform GET request
    sim808_clear_rx_buffer();
    if (sim808_send_at_expect("AT+HTTPACTION=0", "OK", 2000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP action");
        return ESP_FAIL;
    }
    
    // Wait for action to complete: +HTTPACTION: 0,<status>,<datalen>
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    
    // Read response
    sim808_clear_rx_buffer();
    if (sim808_send_at_expect("AT+HTTPREAD", "+HTTPREAD:", 5000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HTTP response");
        return ESP_FAIL;
    }
    
    // Parse response (simplified - needs proper implementation)
    size_t rx_len;
    char *rx_buf = sim808_get_rx_buffer(&rx_len);
    if (rx_buf && rx_len > 0) {
        response->data = malloc(rx_len + 1);
        if (response->data) {
            memcpy(response->data, rx_buf, rx_len);
            response->data[rx_len] = '\0';
            response->data_len = rx_len;
            response->status_code = 200; // Simplified
        }
    }
    
    return ESP_OK;
}

esp_err_t sim808_http_post(const char *url, const char *data, size_t data_len,
                           const char *content_type, sim808_http_response_t *response,
                           uint32_t timeout_ms)
{
    if (!url || !data || !response) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "HTTP POST: %s (len=%d)", url, data_len);
    
    memset(response, 0, sizeof(sim808_http_response_t));
    
    // Set URL
    char cmd[512];
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (sim808_send_at_expect(cmd, "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Set content type
    if (content_type) {
        snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"CONTENT\",\"%s\"", content_type);
        sim808_send_at_expect(cmd, "OK", 2000);
    }
    
    // Set data
    snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,10000", data_len);
    if (sim808_send_at_expect(cmd, "DOWNLOAD", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Send data
    // TODO: Implement actual data sending
    
    // Perform POST request
    sim808_clear_rx_buffer();
    if (sim808_send_at_expect("AT+HTTPACTION=1", "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Wait and read response (similar to GET)
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    
    return ESP_OK;
}

void sim808_http_free_response(sim808_http_response_t *response)
{
    if (response && response->data) {
        free(response->data);
        response->data = NULL;
        response->data_len = 0;
    }
}
