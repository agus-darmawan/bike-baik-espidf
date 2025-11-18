#include "prox_sensor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "PROX";

static volatile uint32_t s_pulse_count = 0;
static void IRAM_ATTR prox_isr(void *arg)
{
    __atomic_add_fetch(&s_pulse_count, 1, __ATOMIC_RELAXED);
}

esp_err_t prox_init(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    gpio_num_t g = (gpio_num_t)((gpio_num == -1) ? PROX_SENSOR_GPIO : gpio_num);

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << g,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = intr_type
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ESP_ERROR_CHECK(gpio_isr_handler_add(g, prox_isr, NULL));
    ESP_LOGI(TAG, "Prox init on GPIO %d", g);
    return ESP_OK;
}

static uint32_t prox_get_and_reset(void)
{
    return __atomic_exchange_n(&s_pulse_count, 0u, __ATOMIC_ACQ_REL);
}

static float prox_speed_kmh(uint32_t pulses, float dt_sec)
{
    if (dt_sec <= 0.0f) return 0.0f;
    float rev_per_sec = ((float)pulses / (float)PROX_PULSES_PER_REV) / dt_sec;
    float speed_ms = rev_per_sec * PROX_WHEEL_CIRC_M;
    return speed_ms * 3.6f;
}

/* Blocking sample: tunggu sample_ms, baca pulsa, hitung speed */
void prox_sample(prox_data_t *out, uint32_t sample_ms)
{
    int64_t t0 = esp_timer_get_time();
    esp_rom_delay_us(sample_ms * 1000ULL);
    int64_t t1 = esp_timer_get_time();

    out->pulses  = prox_get_and_reset();
    out->dt_sec  = (t1 - t0) / 1000000.0f;
    out->speed_kmh = prox_speed_kmh(out->pulses, out->dt_sec);
}
