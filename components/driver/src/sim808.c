#include "sim808.h"
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "stdlib.h"

static const char *TAG = "SIM808";
static sim808_uart_cfg_t cfg;

static char rx_buf[SIM808_RX_BUFFER];

void sim808_init(sim808_uart_cfg_t uart_cfg)
{
    cfg = uart_cfg;

    uart_config_t uart_config = {
        .baud_rate = cfg.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(cfg.uart_num, &uart_config);
    uart_set_pin(cfg.uart_num, cfg.tx_pin, cfg.rx_pin, -1, -1);
    uart_driver_install(cfg.uart_num, SIM808_RX_BUFFER, SIM808_TX_BUFFER, 0, NULL, 0);

    ESP_LOGI(TAG, "SIM808 initialized on UART %d", cfg.uart_num);

    sim808_send_cmd("AT\r\n", "OK", 1000);
    sim808_send_cmd("ATE0\r\n", "OK", 1000);  // echo off
}

bool sim808_send_cmd(const char *cmd, const char *expect, int timeout_ms)
{
    uart_flush(cfg.uart_num);
    uart_write_bytes(cfg.uart_num, cmd, strlen(cmd));

    char line[256];

    int t = 0;
    while (t < timeout_ms)
    {
        if (sim808_readline(line, sizeof(line), 100))
        {
            if (strstr(line, expect))
                return true;
        }
        t += 100;
    }
    return false;
}

bool sim808_readline(char *out, int max_len, int timeout_ms)
{
    int len = uart_read_bytes(cfg.uart_num, (uint8_t *)out, max_len - 1, timeout_ms / portTICK_PERIOD_MS);
    if (len > 0)
    {
        out[len] = '\0';
        return true;
    }
    return false;
}

/* ================================
 *            GSM
 * ================================*/
bool sim808_get_signal(int *rssi)
{
    sim808_send_cmd("AT+CSQ\r\n", "OK", 1000);

    char line[64];
    while (sim808_readline(line, sizeof(line), 200))
    {
        if (strstr(line, "+CSQ"))
        {
            int val = 0;
            sscanf(line, "+CSQ: %d", &val);
            *rssi = val;
            return true;
        }
    }
    return false;
}

bool sim808_get_imei(char *out)
{
    sim808_send_cmd("AT+GSN\r\n", "", 1000);

    char line[64];
    while (sim808_readline(line, sizeof(line), 200))
    {
        if (strlen(line) > 5 && line[0] != 'A') {
            strcpy(out, line);
            out[strcspn(out, "\r\n")] = 0;
            return true;
        }
    }
    return false;
}

/* ================================
 *            GPS
 * ================================*/
bool sim808_gps_power(bool on)
{
    return sim808_send_cmd(on ? "AT+CGNSPWR=1\r\n" : "AT+CGNSPWR=0\r\n", "OK", 1000);
}

bool sim808_gps_get(sim808_gps_t *gps)
{
    sim808_send_cmd("AT+CGNSINF\r\n", "OK", 1000);

    char line[256];
    while (sim808_readline(line, sizeof(line), 200))
    {
        if (strstr(line, "+CGNSINF"))
        {
            // Example:
            // +CGNSINF: 1,1,20240621142356.000, -7.12345,112.12345,12.3, 0.5, ...
            int fix;
            sscanf(line, "+CGNSINF: %d,%d", &fix, &gps->valid);

            if (!gps->valid) return false;

            sscanf(line,
                "+CGNSINF: %d,%d,%*[^,],%f,%f,%f,%f",
                &fix,
                &gps->valid,
                &gps->latitude,
                &gps->longitude,
                &gps->altitude,
                &gps->speed_kmh
            );
            return true;
        }
    }
    return false;
}

/* ================================
 *            GPRS / TCP
 * ================================*/
bool sim808_gprs_attach(const char *apn)
{
    sim808_send_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n", "OK", 2000);

    char buf[64];
    sprintf(buf, "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", apn);
    sim808_send_cmd(buf, "OK", 2000);

    sim808_send_cmd("AT+SAPBR=1,1\r\n", "OK", 5000);

    return true;
}

bool sim808_tcp_open(const char *host, int port)
{
    char cmd[128];
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n", host, port);
    return sim808_send_cmd(cmd, "CONNECT OK", 6000);
}

bool sim808_tcp_send(const char *data)
{
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%d\r\n", strlen(data));
    if (!sim808_send_cmd(cmd, ">", 2000)) return false;

    uart_write_bytes(cfg.uart_num, data, strlen(data));
    return sim808_send_cmd("", "SEND OK", 2000);
}

bool sim808_tcp_close()
{
    return sim808_send_cmd("AT+CIPCLOSE\r\n", "OK", 3000);
}
