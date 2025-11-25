#include "prox_sensor.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static const char *TAG = "proximity";

static volatile uint32_t pulse_count = 0;
static volatile int64_t last_us = 0;
static uint32_t g_debounce_us = 0;

// penting!! IDF v6 masih support ini asal include portmacro.h
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR prox_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (last_us != 0 && (now - last_us) < (int64_t)g_debounce_us) return;
    last_us = now;

    portENTER_CRITICAL_ISR(&mux);
    pulse_count++;
    portEXIT_CRITICAL_ISR(&mux);
}

esp_err_t proximity_init(const proximity_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    g_debounce_us = cfg->debounce_us;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << cfg->pin),
        .mode = GPIO_MODE_INPUT,
        .intr_type = cfg->trigger,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t r = gpio_config(&io);
    if (r != ESP_OK) return r;

    gpio_install_isr_service(0);
    gpio_isr_handler_add(cfg->pin, prox_isr, NULL);
    ESP_LOGI(TAG, "proximity init pin %d", cfg->pin);
    return ESP_OK;
}

uint32_t proximity_get_count(void)
{
    uint32_t val;
    portENTER_CRITICAL(&mux);
    val = pulse_count;
    portEXIT_CRITICAL(&mux);
    return val;
}
