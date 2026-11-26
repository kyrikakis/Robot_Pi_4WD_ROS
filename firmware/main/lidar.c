// Lidar: Oradar MS200 on UART1 (230400 baud, TX 17 -> lidar RX, RX 18 <- lidar TX).
//
// Data packet (47 bytes, little endian):
//   0x54, 0x2C (low 5 bits: 12 points), speed u16 (deg/s: Hz = speed / 360),
//   start angle u16 (0.01 deg), 12 x {distance u16 (mm), intensity u8},
//   end angle u16, timestamp u16, CRC-8 (poly 0x4D).
// Points are binned to whole degrees as the stock firmware did. A revolution is
// complete when the start angle wraps; its bins are handed over and cleared, so a
// point the sensor did not return reads 0 (invalid) instead of an old value.
//
// Commands (Oradar's MS200 SDK, MIT; same frames in the MS200k manual):
//   A5 F5 <cmd> C1 <len> <payload> <xor of all previous bytes> 31 F2
//   run mode   cmd A2, payload 81 (ranging) / 80 (standby: no ranging, no data)
//   speed      cmd A1, payload <Hz> 00, 5..15 Hz (sent in standby)
// The sensor does not acknowledge reliably, so as in the SDK a command is
// confirmed by the data stream: it stops (standby), starts (run), or turns at
// the new speed (+-10 %). Commands are at least 400 ms apart (SDK: faster ones
// are ignored). All of it runs in the lidar task, which keeps reading meanwhile.

#include <string.h>

#include "driver/uart.h"
#include "esp_timer.h"
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

#define STREAM_TIMEOUT_US 500000   // no packet this long: not streaming
#define COMMAND_GAP_US 400000
#define CONFIRM_TIMEOUT_MS 5000
#define SPEED_CONFIRM_TIMEOUT_MS 10000

typedef enum { CMD_NONE, CMD_ENABLE, CMD_SPEED } cmd_kind_t;

static uint8_t s_crc_table[256];
static uint16_t s_work_dist[LIDAR_POINTS];
static uint8_t s_work_int[LIDAR_POINTS];
static uint16_t s_ready_dist[LIDAR_POINTS];
static uint8_t s_ready_int[LIDAR_POINTS];
static bool s_ready;
static float s_ready_period;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Lidar task state.
static bool s_publish = true;       // requested state: scans handed over or dropped
static int64_t s_last_packet_us;
static int64_t s_last_command_us;
static int64_t s_last_revolution_us;
static volatile float s_hz;          // rotation reported by the packets

// Command hand-over (under s_lock).
static cmd_kind_t s_cmd_kind;
static int s_cmd_value;
static uint32_t s_cmd_ticket, s_next_ticket = 1;
static uint32_t s_done_ticket;
static lidar_result_t s_done_result;

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
    int64_t now = esp_timer_get_time();
    s_last_packet_us = now;
    s_hz = u16(pkt + 2) / 360.0f;
    if (!s_publish) {
        last_start = -1;
        return;
    }
    int count = pkt[1] & 0x1F;
    int start = u16(pkt + 4);
    int end = u16(pkt + 6 + 3 * PKT_POINTS);

    if (last_start >= 0 && start < last_start) {
        float period = s_last_revolution_us ? (now - s_last_revolution_us) / 1e6f : 0.0f;
        s_last_revolution_us = now;
        taskENTER_CRITICAL(&s_lock);
        memcpy(s_ready_dist, s_work_dist, sizeof(s_ready_dist));
        memcpy(s_ready_int, s_work_int, sizeof(s_ready_int));
        s_ready_period = period;
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

// Read and parse whatever arrives for up to wait_ms.
static void pump(int wait_ms)
{
    static uint8_t pkt[PKT_LEN];
    static int have = 0;
    uint8_t rx[128];
    int n = uart_read_bytes(LIDAR_UART, rx, sizeof(rx), pdMS_TO_TICKS(wait_ms));
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

static void pump_for(int ms)
{
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) pump(20);
}

static bool streaming(void) { return esp_timer_get_time() - s_last_packet_us < STREAM_TIMEOUT_US; }

static void send_command(uint8_t cmd, const uint8_t *payload, int len)
{
    int64_t wait = s_last_command_us + COMMAND_GAP_US - esp_timer_get_time();
    if (wait > 0) pump_for((int)(wait / 1000) + 1);
    uint8_t frame[16] = {0xA5, 0xF5, cmd, 0xC1, (uint8_t)len};
    memcpy(frame + 5, payload, len);
    uint8_t x = 0;
    for (int i = 0; i < 5 + len; i++) x ^= frame[i];
    frame[5 + len] = x;
    frame[6 + len] = 0x31;
    frame[7 + len] = 0xF2;
    uart_write_bytes(LIDAR_UART, (const char *)frame, 8 + len);
    s_last_command_us = esp_timer_get_time();
}

static void send_run(bool run)
{
    uint8_t mode = run ? 0x81 : 0x80;
    send_command(0xA2, &mode, 1);
}

// Wait until the stream is in the wanted state (and, if hz > 0, turns within 10 % of it).
static bool wait_for(bool want_stream, int hz, int timeout_ms)
{
    int64_t end = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < end) {
        pump(20);
        if (streaming() != want_stream) continue;
        if (hz <= 0) return true;
        if (s_hz > hz * 0.9f && s_hz < hz * 1.1f) return true;
    }
    return false;
}

static bool do_enable(bool on)
{
    s_publish = on;
    if (!on) {
        memset(s_work_dist, 0, sizeof(s_work_dist));
        memset(s_work_int, 0, sizeof(s_work_int));
        s_last_revolution_us = 0;
    }
    send_run(on);
    return wait_for(on, 0, CONFIRM_TIMEOUT_MS);
}

static bool do_speed(int hz)
{
    bool was_running = streaming();
    if (was_running) {
        send_run(false);
        if (!wait_for(false, 0, CONFIRM_TIMEOUT_MS)) return false;
    }
    uint8_t payload[2] = {(uint8_t)hz, 0};
    send_command(0xA1, payload, 2);
    if (!was_running && !s_publish) return true;  // stays in standby; checked on the next enable
    send_run(true);
    return wait_for(true, hz, SPEED_CONFIRM_TIMEOUT_MS);
}

static void lidar_task(void *arg)
{
    // A lidar left in standby by an earlier session would never send again: start it.
    pump_for(500);
    send_run(true);

    while (1) {
        pump(20);
        taskENTER_CRITICAL(&s_lock);
        cmd_kind_t kind = s_cmd_kind;
        int value = s_cmd_value;
        uint32_t ticket = s_cmd_ticket;
        taskEXIT_CRITICAL(&s_lock);
        if (kind == CMD_NONE) continue;

        bool ok = (kind == CMD_ENABLE) ? do_enable(value != 0) : do_speed(value);
        taskENTER_CRITICAL(&s_lock);
        s_cmd_kind = CMD_NONE;
        s_done_ticket = ticket;
        s_done_result = ok ? LIDAR_OK : LIDAR_FAILED;
        if (!s_publish) s_ready = false;
        taskEXIT_CRITICAL(&s_lock);
    }
}

static uint32_t submit(cmd_kind_t kind, int value)
{
    uint32_t ticket = 0;
    taskENTER_CRITICAL(&s_lock);
    if (s_cmd_kind == CMD_NONE) {
        ticket = s_next_ticket++;
        s_cmd_kind = kind;
        s_cmd_value = value;
        s_cmd_ticket = ticket;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ticket;
}

uint32_t lidar_command_enable(bool on) { return submit(CMD_ENABLE, on ? 1 : 0); }

uint32_t lidar_command_speed(int hz)
{
    if (hz < LIDAR_MIN_HZ || hz > LIDAR_MAX_HZ) return 0;
    return submit(CMD_SPEED, hz);
}

lidar_result_t lidar_command_result(uint32_t ticket)
{
    lidar_result_t r = LIDAR_BUSY;
    taskENTER_CRITICAL(&s_lock);
    if (ticket != 0 && s_done_ticket == ticket) r = s_done_result;
    taskEXIT_CRITICAL(&s_lock);
    return r;
}

float lidar_rotation_hz(void) { return streaming() ? s_hz : 0.0f; }

bool lidar_take_revolution(uint16_t distance_mm[LIDAR_POINTS], uint8_t intensity[LIDAR_POINTS],
                           float *period_s)
{
    bool got = false;
    taskENTER_CRITICAL(&s_lock);
    if (s_ready) {
        memcpy(distance_mm, s_ready_dist, sizeof(s_ready_dist));
        memcpy(intensity, s_ready_int, sizeof(s_ready_int));
        *period_s = s_ready_period;
        s_ready = false;
        got = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return got;
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
