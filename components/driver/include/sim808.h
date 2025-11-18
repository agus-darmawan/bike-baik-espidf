#ifndef SIM808_H
#define SIM808_H

#include "stdint.h"
#include "stdbool.h"
#include "driver/uart.h"

// UART buffer
#define SIM808_RX_BUFFER 1024
#define SIM808_TX_BUFFER 1024

// UART config
typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int baudrate;
} sim808_uart_cfg_t;

// Parsed GPS data
typedef struct {
    bool valid;
    float latitude;
    float longitude;
    float altitude;
    float speed_kmh;
} sim808_gps_t;

// Public API
void sim808_init(sim808_uart_cfg_t cfg);
bool sim808_send_cmd(const char *cmd, const char *expect, int timeout_ms);
bool sim808_readline(char *out, int max_len, int timeout_ms);

// GSM
bool sim808_get_signal(int *rssi);
bool sim808_get_imei(char *out_imei);

// GPS
bool sim808_gps_power(bool on);
bool sim808_gps_get(sim808_gps_t *gps);

// GPRS (TCP)
bool sim808_gprs_attach(const char *apn);
bool sim808_tcp_open(const char *host, int port);
bool sim808_tcp_send(const char *data);
bool sim808_tcp_close();

#endif
