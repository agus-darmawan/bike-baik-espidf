#include "sim808.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SIM808";

#define AT_RESPONSE_BUFFER_SIZE 2048
#define AT_TIMEOUT_MS 5000
#define UART_RX_BUFFER_SIZE 2048
#define UART_TX_BUFFER_SIZE 512

typedef struct {
    uart_port_t uart_port;
    int pwr_pin;
    int rst_pin;
    
    char apn[64];
    char apn_user[32];
    char apn_pass[32];
    
    bool initialized;
    bool gprs_connected;
    bool mqtt_connected;
    bool gps_enabled;
    
    SemaphoreHandle_t uart_mutex;
    QueueHandle_t event_queue;
    TaskHandle_t rx_task_handle;
    
    char rx_buffer[AT_RESPONSE_BUFFER_SIZE];
    size_t rx_len;
    
} sim808_context_t;

static sim808_context_t g_ctx = {0};

/* ===== Forward Declarations ===== */
static esp_err_t sim808_send_at(const char *cmd);
static esp_err_t sim808_wait_response(const char *expected, uint32_t timeout_ms);

/* ===== Internal Functions ===== */

static void sim808_rx_task(void *arg)
{
    uint8_t data[128];
    
    while (1) {
        int len = uart_read_bytes(g_ctx.uart_port, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(100))) {
                for (int i = 0; i < len && g_ctx.rx_len < AT_RESPONSE_BUFFER_SIZE - 1; i++) {
                    g_ctx.rx_buffer[g_ctx.rx_len++] = data[i];
                    g_ctx.rx_buffer[g_ctx.rx_len] = '\0';
                }
                xSemaphoreGive(g_ctx.uart_mutex);
            }
        }
    }
}

static esp_err_t sim808_send_at(const char *cmd)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    int len = uart_write_bytes(g_ctx.uart_port, cmd, strlen(cmd));
    uart_write_bytes(g_ctx.uart_port, "\r\n", 2);
    
    ESP_LOGD(TAG, "AT> %s", cmd);
    return (len > 0) ? ESP_OK : ESP_FAIL;
}

static esp_err_t sim808_wait_response(const char *expected, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    
    while ((xTaskGetTickCount() - start) < timeout) {
        if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(10))) {
            if (strstr(g_ctx.rx_buffer, expected) != NULL) {
                ESP_LOGD(TAG, "AT< %s (found in buffer)", expected);
                xSemaphoreGive(g_ctx.uart_mutex);
                return ESP_OK;
            }
            xSemaphoreGive(g_ctx.uart_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGW(TAG, "Timeout waiting for: %s", expected);
    return ESP_ERR_TIMEOUT;
}

/* ===== Public/Exported Internal Functions (for http and mqtt modules) ===== */

esp_err_t sim808_send_at_expect(const char *cmd, const char *expected, uint32_t timeout_ms)
{
    if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(1000))) {
        g_ctx.rx_len = 0;
        g_ctx.rx_buffer[0] = '\0';
        xSemaphoreGive(g_ctx.uart_mutex);
    }
    
    esp_err_t ret = sim808_send_at(cmd);
    if (ret != ESP_OK) return ret;
    
    return sim808_wait_response(expected, timeout_ms);
}

void sim808_clear_rx_buffer(void)
{
    if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(100))) {
        g_ctx.rx_len = 0;
        g_ctx.rx_buffer[0] = '\0';
        xSemaphoreGive(g_ctx.uart_mutex);
    }
}

/* ===== Public API Implementation ===== */

esp_err_t sim808_init(const sim808_config_t *config)
{
    if (!config) return ESP_ERR_INVALID_ARG;
    if (g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Initializing SIM808 on UART%d", config->uart_port);
    
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.uart_port = config->uart_port;
    g_ctx.pwr_pin = config->pwr_pin;
    g_ctx.rst_pin = config->rst_pin;
    
    if (config->apn) strncpy(g_ctx.apn, config->apn, sizeof(g_ctx.apn) - 1);
    if (config->apn_user) strncpy(g_ctx.apn_user, config->apn_user, sizeof(g_ctx.apn_user) - 1);
    if (config->apn_pass) strncpy(g_ctx.apn_pass, config->apn_pass, sizeof(g_ctx.apn_pass) - 1);
    
    // Configure GPIO pins
    if (g_ctx.pwr_pin >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << g_ctx.pwr_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(g_ctx.pwr_pin, 0);
    }
    
    if (g_ctx.rst_pin >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << g_ctx.rst_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&io_conf);
        gpio_set_level(g_ctx.rst_pin, 1);
    }
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(g_ctx.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(g_ctx.uart_port, config->tx_pin, config->rx_pin,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(g_ctx.uart_port, UART_RX_BUFFER_SIZE,
                                        UART_TX_BUFFER_SIZE, 0, NULL, 0));
    
    // Create mutex and task
    g_ctx.uart_mutex = xSemaphoreCreateMutex();
    if (!g_ctx.uart_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    xTaskCreate(sim808_rx_task, "sim808_rx", 4096, NULL, 10, &g_ctx.rx_task_handle);
    
    g_ctx.initialized = true;
    ESP_LOGI(TAG, "SIM808 initialized");
    
    return ESP_OK;
}

void sim808_deinit(void)
{
    if (!g_ctx.initialized) return;
    
    if (g_ctx.rx_task_handle) {
        vTaskDelete(g_ctx.rx_task_handle);
        g_ctx.rx_task_handle = NULL;
    }
    
    if (g_ctx.uart_mutex) {
        vSemaphoreDelete(g_ctx.uart_mutex);
        g_ctx.uart_mutex = NULL;
    }
    
    uart_driver_delete(g_ctx.uart_port);
    
    g_ctx.initialized = false;
    ESP_LOGI(TAG, "SIM808 deinitialized");
}

esp_err_t sim808_power_on(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Powering on SIM808");
    
    if (g_ctx.pwr_pin >= 0) {
        gpio_set_level(g_ctx.pwr_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(g_ctx.pwr_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    // Wait for module to be ready
    for (int i = 0; i < 20; i++) {
        if (sim808_send_at_expect("AT", "OK", 1000) == ESP_OK) {
            ESP_LOGI(TAG, "Module ready");
            
            // Basic configuration
            sim808_send_at_expect("ATE0", "OK", 1000); // Echo off
            sim808_send_at_expect("AT+CMEE=2", "OK", 1000); // Enable error reporting
            sim808_send_at_expect("AT+CREG=1", "OK", 1000); // Enable network registration
            
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    ESP_LOGE(TAG, "Module not responding");
    return ESP_ERR_TIMEOUT;
}

esp_err_t sim808_power_off(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Powering off SIM808");
    sim808_send_at_expect("AT+CPOWD=1", "OK", 5000);
    
    return ESP_OK;
}

bool sim808_is_ready(void)
{
    if (!g_ctx.initialized) return false;
    return (sim808_send_at_expect("AT", "OK", 1000) == ESP_OK);
}

int sim808_get_signal_strength(void)
{
    if (!g_ctx.initialized) return -1;
    
    sim808_clear_rx_buffer();
    if (sim808_send_at_expect("AT+CSQ", "OK", 2000) != ESP_OK) {
        return -1;
    }
    
    // Parse response: +CSQ: <rssi>,<ber>
    if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(100))) {
        char *p = strstr(g_ctx.rx_buffer, "+CSQ:");
        if (p) {
            int rssi, ber;
            if (sscanf(p, "+CSQ: %d,%d", &rssi, &ber) == 2) {
                xSemaphoreGive(g_ctx.uart_mutex);
                return rssi;
            }
        }
        xSemaphoreGive(g_ctx.uart_mutex);
    }
    
    return -1;
}

/* ===== GPS Functions ===== */

esp_err_t sim808_gps_enable(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Enabling GPS");
    
    if (sim808_send_at_expect("AT+CGPSPWR=1", "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Set GPS to output GGA and RMC sentences
    sim808_send_at_expect("AT+CGPSOUT=32", "OK", 2000);
    
    g_ctx.gps_enabled = true;
    return ESP_OK;
}

esp_err_t sim808_gps_disable(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Disabling GPS");
    sim808_send_at_expect("AT+CGPSPWR=0", "OK", 2000);
    
    g_ctx.gps_enabled = false;
    return ESP_OK;
}

esp_err_t sim808_gps_get_data(sim808_gps_data_t *data)
{
    if (!g_ctx.initialized || !data) return ESP_ERR_INVALID_ARG;
    if (!g_ctx.gps_enabled) return ESP_ERR_INVALID_STATE;
    
    sim808_clear_rx_buffer();
    if (sim808_send_at_expect("AT+CGPSINF=0", "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Parse GPS data from response
    // Format: +CGPSINF: <mode>,<lon>,<lat>,<alt>,<utc>,<ttff>,<num>,<speed>,<course>
    if (xSemaphoreTake(g_ctx.uart_mutex, pdMS_TO_TICKS(100))) {
        char *p = strstr(g_ctx.rx_buffer, "+CGPSINF:");
        if (p) {
            int mode;
            float lon, lat, alt, speed, course;
            int num;
            char utc[32];
            
            int parsed = sscanf(p, "+CGPSINF: %d,%f,%f,%f,%[^,],%*d,%d,%f,%f",
                               &mode, &lon, &lat, &alt, utc, &num, &speed, &course);
            
            if (parsed >= 8 && mode != 0) {
                data->fix_valid = true;
                data->longitude = lon;
                data->latitude = lat;
                data->altitude = alt;
                data->speed = speed;
                data->course = course;
                data->satellites = num;
                
                // Parse UTC time (format: yyyyMMddhhmmss.sss)
                if (strlen(utc) >= 14) {
                    sscanf(utc, "%4hu%2hhu%2hhu%2hhu%2hhu%2hhu",
                          &data->utc_time.year,
                          &data->utc_time.month,
                          &data->utc_time.day,
                          &data->utc_time.hour,
                          &data->utc_time.minute,
                          &data->utc_time.second);
                }
                
                xSemaphoreGive(g_ctx.uart_mutex);
                return ESP_OK;
            }
        }
        xSemaphoreGive(g_ctx.uart_mutex);
    }
    
    data->fix_valid = false;
    return ESP_FAIL;
}

/* ===== Network Functions ===== */

esp_err_t sim808_gprs_connect(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Connecting to GPRS with APN: %s", g_ctx.apn);
    
    // Set connection type to GPRS
    if (sim808_send_at_expect("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"", "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Set APN
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", g_ctx.apn);
    if (sim808_send_at_expect(cmd, "OK", 2000) != ESP_OK) {
        return ESP_FAIL;
    }
    
    // Set username if provided
    if (strlen(g_ctx.apn_user) > 0) {
        snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"USER\",\"%s\"", g_ctx.apn_user);
        sim808_send_at_expect(cmd, "OK", 2000);
    }
    
    // Set password if provided
    if (strlen(g_ctx.apn_pass) > 0) {
        snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"PWD\",\"%s\"", g_ctx.apn_pass);
        sim808_send_at_expect(cmd, "OK", 2000);
    }
    
    // Open bearer
    if (sim808_send_at_expect("AT+SAPBR=1,1", "OK", 30000) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open bearer");
        return ESP_FAIL;
    }
    
    // Query bearer status
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (sim808_send_at_expect("AT+SAPBR=2,1", "OK", 2000) == ESP_OK) {
        g_ctx.gprs_connected = true;
        ESP_LOGI(TAG, "GPRS connected");
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

esp_err_t sim808_gprs_disconnect(void)
{
    if (!g_ctx.initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Disconnecting GPRS");
    sim808_send_at_expect("AT+SAPBR=0,1", "OK", 5000);
    
    g_ctx.gprs_connected = false;
    return ESP_OK;
}

bool sim808_gprs_is_connected(void)
{
    return g_ctx.gprs_connected;
}