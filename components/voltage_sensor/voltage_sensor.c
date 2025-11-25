#include "voltage_sensor.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "voltage_sensor";

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;
static adc_channel_t g_channel = ADC_CHANNEL_0;
static float g_r1 = 0.0f;
static float g_r2 = 0.0f;
static uint8_t g_samples = 1;

esp_err_t voltage_sensor_init(const voltage_sensor_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (cfg->r1 <= 0 || cfg->r2 <= 0) return ESP_ERR_INVALID_ARG;

    g_channel = cfg->channel;
    g_r1 = cfg->r1;
    g_r2 = cfg->r2;
    g_samples = (cfg->samples > 0) ? cfg->samples : 1;

    // Init ADC
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = cfg->unit,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_cfg, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed (%d)", ret);
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = cfg->attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(adc_handle, cfg->channel, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed (%d)", ret);
        return ret;
    }

    // Init calibration
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = cfg->unit,
        .atten = cfg->attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = cfg->unit,
        .atten = cfg->attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle);
#endif

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration init failed, using raw values");
        cali_handle = NULL;
    }

    ESP_LOGI(TAG, "Voltage sensor init on ADC%d CH%d, R1=%.1f R2=%.1f", 
             cfg->unit + 1, cfg->channel, cfg->r1, cfg->r2);
    return ESP_OK;
}

int voltage_sensor_read_raw(void)
{
    if (!adc_handle) return -1;

    int sum = 0;
    for (int i = 0; i < g_samples; i++) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, g_channel, &raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_read failed (%d)", ret);
            return -1;
        }
        sum += raw;
    }
    return sum / g_samples;
}

float voltage_sensor_read(void)
{
    int raw = voltage_sensor_read_raw();
    if (raw < 0) return -1.0f;

    float adc_voltage = 0.0f;

    if (cali_handle) {
        int voltage_mv = 0;
        esp_err_t ret = adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "adc_cali_raw_to_voltage failed (%d)", ret);
            return -1.0f;
        }
        adc_voltage = voltage_mv / 1000.0f;
    } else {
        // Rough conversion without calibration (for 11dB atten)
        adc_voltage = (raw / 4095.0f) * 3.3f;
    }

    // Apply voltage divider formula: Vout = Vin * R2 / (R1 + R2)
    // So: Vin = Vout * (R1 + R2) / R2
    float actual_voltage = adc_voltage * (g_r1 + g_r2) / g_r2;
    
    return actual_voltage;
}