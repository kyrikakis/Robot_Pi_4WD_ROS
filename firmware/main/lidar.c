// Lidar: Oradar MS200 on UART1 (230400 baud). The sensor streams by itself.
//
// Data packet (47 bytes, little endian):
//   0x54, 0x2C (low 5 bits: 12 points), speed u16, start angle u16 (0.01 deg),
//   12 x {distance u16 (mm), intensity u8}, end angle u16, timestamp u16, CRC-8 (poly 0x4D).
// Points are binned to whole degrees as the stock firmware did. A revolution is
// complete when the start angle wraps; its bins are handed over and cleared, so a
// point the sensor did not return reads 0 (invalid) instead of an old value.

#include <string.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rainbow.h"

#define LIDAR_UART UART_NUM_1
#define LIDAR_TX 17
#define LIDAR_RX 18
#define LIDAR_BAUD 230400
#define PKT_HEADER 0x54
#define PKT_POINTS 12
#define PKT_LEN (11 + 3 * PKT_POINTS)

static uint8_t s_crc_table[256];
static uint16_t s_work_dist[LIDAR_POINTS];
static uint8_t s_work_int[LIDAR_POINTS];
static uint16_t s_ready_dist[LIDAR_POINTS];
static uint8_t s_ready_int[LIDAR_POINTS];
static bool s_ready;
static volatile bool s_enabled = true;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static void crc_table_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
        s_crc_table[i] = crc;
    }
}

static uint8_t crc8(const uint8_t *buf, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) crc = s_crc_table[crc ^ buf[i]];
    return crc;
}

static uint16_t u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void handle_packet(const uint8_t *pkt)
{
    static int last_start = -1;
    if (!s_enabled) {
        last_start = -1;
        return;
    }
    int count = pkt[1] & 0x1F;
    if (count != PKT_POINTS) return;
    int start = u16(pkt + 4);
    int end = u16(pkt + 6 + 3 * PKT_POINTS);

    if (last_start >= 0 && start < last_start) {
        taskENTER_CRITICAL(&s_lock);
        memcpy(s_ready_dist, s_work_dist, sizeof(s_ready_dist));
        memcpy(s_ready_int, s_work_int, sizeof(s_ready_int));
        s_ready = true;
        taskEXIT_CRITICAL(&s_lock);
        memset(s_work_dist, 0, sizeof(s_work_dist));
        memset(s_work_int, 0, sizeof(s_work_int));
    }
    last_start = start;

    int span = end >= start ? end - start : 36000 + end - start;
    int step = span / (count - 1);
    for (int i = 0; i < count; i++) {
        int bin = ((start + i * step) / 100) % LIDAR_POINTS;
        s_work_dist[bin] = u16(pkt + 6 + 3 * i);
        s_work_int[bin] = pkt[8 + 3 * i];
    }
}

static void lidar_task(void *arg)
{
    uint8_t rx[128];
    uint8_t pkt[PKT_LEN];
    int have = 0;
    while (1) {
        int n = uart_read_bytes(LIDAR_UART, rx, sizeof(rx), pdMS_TO_TICKS(20));
        for (int k = 0; k < n; k++) {
            uint8_t b = rx[k];
            if (have == 0 && b != PKT_HEADER) continue;
            if (have == 1 && (b & 0x1F) != PKT_POINTS) {
                have = (b == PKT_HEADER) ? 1 : 0;
                continue;
            }
            pkt[have++] = b;
            if (have == PKT_LEN) {
                if (crc8(pkt, PKT_LEN - 1) == pkt[PKT_LEN - 1]) handle_packet(pkt);
                have = 0;
            }
        }
    }
}

bool lidar_take_revolution(uint16_t distance_mm[LIDAR_POINTS], uint8_t intensity[LIDAR_POINTS])
{
    bool got = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_ready) {
        memcpy(distance_mm, s_ready_dist, sizeof(s_ready_dist));
        memcpy(intensity, s_ready_int, sizeof(s_ready_int));
        s_ready = false;
        got = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return got;
}

void lidar_set_enabled(bool on)
{
    taskENTER_CRITICAL(&s_lock);
    s_enabled = on;
    s_ready = false;
    taskEXIT_CRITICAL(&s_lock);
    if (!on) {
        // Next revolution starts clean (the lidar task only writes bins while enabled).
        memset(s_work_dist, 0, sizeof(s_work_dist));
        memset(s_work_int, 0, sizeof(s_work_int));
    }
}

void lidar_init(void)
{
    crc_table_init();
    uart_config_t cfg = {
        .baud_rate = LIDAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LIDAR_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LIDAR_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LIDAR_UART, LIDAR_TX, LIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreatePinnedToCore(lidar_task, "lidar", 4096, NULL, 6, NULL, 1);
}
