#include "max6675.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "max6675";

static spi_device_handle_t dev = NULL;
static SemaphoreHandle_t g_spi_sem = NULL;

esp_err_t max6675_init(spi_host_device_t host,
                      gpio_num_t clk_pin,
                      gpio_num_t miso_pin,
                      gpio_num_t cs_pin,
                      uint32_t hz,
                      SemaphoreHandle_t spi_sem)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,
        .miso_io_num = miso_pin,
        .sclk_io_num = clk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16/8
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = hz,
        .mode = 0,
        .spics_io_num = cs_pin,
        .queue_size = 1,
    };

    ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed (%d)", ret);
        return ret;
    }

    ret = spi_bus_add_device(host, &devcfg, &dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed (%d)", ret);
        return ret;
    }

    g_spi_sem = spi_sem; // may be NULL
    ESP_LOGI(TAG, "MAX6675 initialized");
    return ESP_OK;
}

float max6675_read_temperature(void)
{
    if (!dev) return -1000.0f;

    if (g_spi_sem) {
        if (xSemaphoreTake(g_spi_sem, pdMS_TO_TICKS(200)) != pdTRUE) {
            ESP_LOGW(TAG, "SPI busy");
            return -1000.0f;
        }
    }

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.flags = SPI_TRANS_USE_RXDATA;

    esp_err_t err = spi_device_transmit(dev, &t);

    if (g_spi_sem) xSemaphoreGive(g_spi_sem);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi transmit err %d", err);
        return -1000.0f;
    }

    uint16_t v = (t.rx_data[0] << 8) | t.rx_data[1];
    if (v & 0x04) {
        ESP_LOGW(TAG, "thermocouple open");
        return -1000.0f;
    }
    v >>= 3;
    float t_c = v * 0.25f;
    return t_c;
}