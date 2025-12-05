#ifndef __CONNECT_UART_H__
#define __CONNECT_UART_H__

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /// @module UART通信模块
    /// @brief ESP32-S3 UART通信驱动，支持DMA收发和数据包协议
    /// @warning 使用前必须调用uart_comm_init()进行初始化
    /// @note 支持线程安全操作，内置错误处理和重传机制
    /// @author ZHC
    /// @date 2025
    /// @version 1.0

    // =============================================================================
    // 配置参数定义
    // =============================================================================

#define BRAIN_UART_NUM UART_NUM_1             ///< 使用UART1端口
#define BRAIN_UART_BAUD_RATE 1000000          ///< 波特率1000000
#define BRAIN_UART_TXD_PIN GPIO_NUM_17        ///< TXD引脚IO17
#define BRAIN_UART_RXD_PIN GPIO_NUM_18        ///< RXD引脚IO18
#define BRAIN_UART_RTS_PIN UART_PIN_NO_CHANGE ///< 不使用RTS
#define BRAIN_UART_CTS_PIN UART_PIN_NO_CHANGE ///< 不使用CTS
#define BRAIN_UART_RX_BUF_SIZE (8192)         ///< 接收缓冲区大小8192字节
#define BRAIN_UART_TX_BUF_SIZE (8192)         ///< 发送缓冲区大小8192字节

#define CI03T_UART_NUM UART_NUM_2             ///< 使用UART2端口
#define CI03T_UART_BAUD_RATE 115200           ///< 波特率115200
#define CI03T_UART_TXD_PIN GPIO_NUM_42        ///< TXD引脚IO42
#define CI03T_UART_RXD_PIN GPIO_NUM_41        ///< RXD引脚IO41
#define CI03T_UART_RTS_PIN UART_PIN_NO_CHANGE ///< 不使用RTS
#define CI03T_UART_CTS_PIN UART_PIN_NO_CHANGE ///< 不使用CTS
#define CI03T_UART_RX_BUF_SIZE (256)          ///< 接收缓冲区大小128字节
#define CI03T_UART_TX_BUF_SIZE (0)            ///< 发送缓冲区大小0字节（禁用TX缓冲）

    esp_err_t uart_comm_init_all(void);
    QueueHandle_t uart_get_event_queue(uart_port_t port);

#ifdef __cplusplus
}
#endif

#endif /* __CONNECT_UART_H__ */
