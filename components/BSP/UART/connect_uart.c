#include "connect_uart.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

static QueueHandle_t s_uart_event_queue[UART_NUM_MAX] = {0};

static esp_err_t uart_apply_config_ex(uart_port_t port, int baud, gpio_num_t tx, gpio_num_t rx, int rts, int cts, int rx_buf, int tx_buf, int queue_len, int rx_tout);
static esp_err_t uart_comm_init_brain(void);
static esp_err_t uart_comm_init_ci03t(void);
static esp_err_t uart_apply_config_ex(uart_port_t port, int baud, gpio_num_t tx, gpio_num_t rx, int rts, int cts, int rx_buf, int tx_buf, int queue_len, int rx_tout)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_param_config(port, &cfg);
    if (ret != ESP_OK)
        return ret;
    ret = uart_set_pin(port, tx, rx, rts, cts);
    if (ret != ESP_OK)
        return ret;
    QueueHandle_t *queue_param = NULL;
    if (queue_len > 0)
    {
        queue_param = &s_uart_event_queue[port];
    }
    ret = uart_driver_install(port, rx_buf, tx_buf, queue_len, queue_param, 0);
    if (ret != ESP_OK)
        return ret;
    uart_set_rx_timeout(port, rx_tout);
    return ESP_OK;
}

static esp_err_t uart_comm_init_brain(void)
{
    return uart_apply_config_ex(BRAIN_UART_NUM, BRAIN_UART_BAUD_RATE, BRAIN_UART_TXD_PIN, BRAIN_UART_RXD_PIN, BRAIN_UART_RTS_PIN, BRAIN_UART_CTS_PIN, BRAIN_UART_RX_BUF_SIZE, BRAIN_UART_TX_BUF_SIZE, 0, 2);
}

static esp_err_t uart_comm_init_ci03t(void)
{
    return uart_apply_config_ex(CI03T_UART_NUM, CI03T_UART_BAUD_RATE, CI03T_UART_TXD_PIN, CI03T_UART_RXD_PIN, CI03T_UART_RTS_PIN, CI03T_UART_CTS_PIN, CI03T_UART_RX_BUF_SIZE, CI03T_UART_TX_BUF_SIZE, 4, 10);
}

esp_err_t uart_comm_init_all(void)
{
    esp_err_t r1 = uart_comm_init_brain();
    esp_err_t r2 = uart_comm_init_ci03t();
    return (r1 == ESP_OK && r2 == ESP_OK) ? ESP_OK : (r1 != ESP_OK ? r1 : r2);
}

QueueHandle_t uart_get_event_queue(uart_port_t port)
{
    return s_uart_event_queue[port];
}
