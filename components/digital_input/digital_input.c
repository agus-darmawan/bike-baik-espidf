#include "digital_input.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "digital_input";

static gpio_num_t g_pin = GPIO_NUM_NC;
static uint32_t g_debounce_ms = 0;
static volatile int64_t last_change_us = 0;
static volatile uint32_t change_count = 0;
static volatile int last_level = -1;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR digital_input_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (last_change_us != 0 && (now - last_change_us) < (int64_t)(g_debounce_ms * 1000)) {
        return;
    }
    last_change_us = now;

    int level = gpio_get_level(g_pin);
    if (level != last_level) {
        portENTER_CRITICAL_ISR(&mux);
        change_count++;
        last_level = level;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

esp_err_t digital_input_init(const digital_input_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    g_pin = cfg->pin;
    g_debounce_ms = cfg->debounce_ms;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->pin),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = (cfg->pull_mode == GPIO_PULLUP_ONLY || cfg->pull_mode == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (cfg->pull_mode == GPIO_PULLDOWN_ONLY || cfg->pull_mode == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
    };
    
    esp_err_t r = gpio_config(&io);
    if (r != ESP_OK) return r;

    last_level = gpio_get_level(g_pin);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(cfg->pin, digital_input_isr, NULL);
    
    ESP_LOGI(TAG, "Digital input init on GPIO %d", cfg->pin);
    return ESP_OK;
}

bool digital_input_read(void)
{
    if (g_pin == GPIO_NUM_NC) return false;
    return gpio_get_level(g_pin);
}

uint32_t digital_input_get_change_count(void)
{
    uint32_t val;
    portENTER_CRITICAL(&mux);
    val = change_count;
    portEXIT_CRITICAL(&mux);
    return val;
}