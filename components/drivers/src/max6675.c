#include "max6675.h"
#include "esp_log.h"
#include "esp_err.h"

// SPI handle
static spi_device_handle_t spi;

/**
 * Initialize MAX6675 sensor over SPI.
 * Configures the SPI bus and device to communicate with the MAX6675.
 */
esp_err_t max6675_init(void) {
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,  // No MOSI pin (only read)
        .miso_io_num = MAX6675_MISO_PIN,
        .sclk_io_num = MAX6675_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_SPEED_HZ,  // SPI clock speed
        .mode = 0,                      // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = MAX6675_CS_PIN, // Chip select pin
        .queue_size = 1,                // We only need one transaction at a time
        .pre_cb = NULL,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE("MAX6675", "Failed to initialize SPI bus");
        return ret;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE("MAX6675", "Failed to add SPI device");
        return ret;
    }

    ESP_LOGI("MAX6675", "MAX6675 initialized successfully");
    return ESP_OK;
}

/**
 * Read the temperature value from MAX6675.
 * Retrieves 12-bit temperature data and converts it to Celsius.
 * @return Temperature in Celsius.
 */
float max6675_read_temperature(void) {
    uint8_t data[2] = {0};
    esp_err_t ret;

    // SPI transaction to read 2 bytes of data
    spi_transaction_t trans = {
        .length = 16,          // 16 bits = 2 bytes
        .tx_buffer = NULL,     // No data to transmit
        .rx_buffer = data,     // Data to receive
        .flags = SPI_TRANS_USE_RXDATA
    };

    ret = spi_device_transmit(spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE("MAX6675", "Failed to read from MAX6675");
        return -1.0f; // Error reading data
    }

    // Combine the two bytes and convert to temperature
    int raw_value = ((data[0] << 8) | data[1]) >> 3;
    float temperature = raw_value * 0.25; // MAX6675 provides 0.25°C resolution

    ESP_LOGI("MAX6675", "Temperature: %.2f°C", temperature);
    return temperature;
}