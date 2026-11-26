// micro-ROS custom transport: UART0 on GPIO43/44 (the CP2102 to the Pi).

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

#include "rainbow.h"

#define UROS_UART UART_NUM_0
#define UROS_TX 43
#define UROS_RX 44
#define UROS_BUF 2048

static size_t s_port = UROS_UART;

static bool uart_open(struct uxrCustomTransport *transport)
{
    if (uart_is_driver_installed(UROS_UART)) return true;
    uart_config_t cfg = {
        .baud_rate = CONFIG_RAINBOW_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(UROS_UART, UROS_BUF, UROS_BUF, 0, NULL, 0) != ESP_OK) return false;
    if (uart_param_config(UROS_UART, &cfg) != ESP_OK) return false;
    if (uart_set_pin(UROS_UART, UROS_TX, UROS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    return true;
}

// The driver stays installed across sessions; reconnecting reuses it.
static bool uart_close(struct uxrCustomTransport *transport) { return true; }

static size_t uart_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    int n = uart_write_bytes(UROS_UART, (const char *)buf, len);
    return n < 0 ? 0 : (size_t)n;
}

static size_t uart_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    int n = uart_read_bytes(UROS_UART, buf, len, pdMS_TO_TICKS(timeout));
    return n < 0 ? 0 : (size_t)n;
}

void transport_init(void)
{
    rmw_uros_set_custom_transport(true, (void *)&s_port, uart_open, uart_close, uart_write, uart_read);
}
