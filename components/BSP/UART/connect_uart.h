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

#define UART_COMM_PORT_NUM UART_NUM_1        ///< 使用UART1端口
#define UART_COMM_BAUD_RATE 115200           ///< 波特率115200
#define UART_COMM_TXD_PIN GPIO_NUM_17        ///< TXD引脚IO17
#define UART_COMM_RXD_PIN GPIO_NUM_18        ///< RXD引脚IO18
#define UART_COMM_RTS_PIN UART_PIN_NO_CHANGE ///< 不使用RTS
#define UART_COMM_CTS_PIN UART_PIN_NO_CHANGE ///< 不使用CTS

#define UART_COMM_BUF_SIZE 1024        ///< UART缓冲区大小
#define UART_COMM_QUEUE_SIZE 20        ///< 事件队列大小
#define UART_COMM_TASK_STACK_SIZE 4096 ///< 任务栈大小
#define UART_COMM_TASK_PRIORITY 10     ///< 任务优先级

    // =============================================================================
    // 数据包协议定义
    // =============================================================================

#define UART_PACKET_START_FLAG 0xAA ///< 起始标志
#define UART_PACKET_END_FLAG 0x55   ///< 结束标志

// 上行数据包（设备上报给上位机）
#define UART_UPLINK_PACKET_SIZE 17 ///< 上行数据包总长度
#define UART_UPLINK_DATA_SIZE 14   ///< 上行数据载荷长度（不包括start_flag、length、end_flag）

// 下行数据包（上位机下发给设备）
#define UART_DOWNLINK_PACKET_SIZE 16 ///< 下行数据包总长度
#define UART_DOWNLINK_DATA_SIZE 13   ///< 下行数据载荷长度

#define UART_TIMEOUT_MS 100    ///< 超时时间100ms
#define UART_MAX_RETRY_COUNT 3 ///< 最大重传次数

// 调试配置
#define UART_DEBUG_BYTE_BY_BYTE 0  ///< 1=逐字节发送调试, 0=批量发送
#define UART_DEBUG_SEND_DELAY_MS 1 ///< 逐字节发送间隔(ms)

    // =============================================================================
    // 枚举类型定义
    // =============================================================================

    /**
     * @brief UART通信状态枚举
     */
    typedef enum
    {
        UART_STATE_IDLE = 0,  ///< 空闲状态
        UART_STATE_RECEIVING, ///< 接收中
        UART_STATE_SENDING,   ///< 发送中
        UART_STATE_ERROR,     ///< 错误状态
        UART_STATE_TIMEOUT    ///< 超时状态
    } uart_comm_state_t;

    /**
     * @brief 数据包解析状态枚举
     */
    typedef enum
    {
        PACKET_STATE_WAIT_START = 0, ///< 等待起始标志
        PACKET_STATE_WAIT_LENGTH,    ///< 等待长度字节
        PACKET_STATE_WAIT_DATA,      ///< 等待数据载荷
        PACKET_STATE_WAIT_END,       ///< 等待结束标志
        PACKET_STATE_COMPLETE        ///< 数据包完整
    } packet_parse_state_t;

    /**
     * @brief 错误类型枚举
     */
    typedef enum
    {
        UART_ERROR_NONE = 0,    ///< 无错误
        UART_ERROR_TIMEOUT,     ///< 超时错误
        UART_ERROR_LENGTH,      ///< 长度错误
        UART_ERROR_FORMAT,      ///< 格式错误
        UART_ERROR_BUFFER_FULL, ///< 缓冲区满
        UART_ERROR_HARDWARE     ///< 硬件错误
    } uart_error_type_t;

    // =============================================================================
    // 结构体定义
    // =============================================================================

    /**
     * @brief 上行数据包结构（设备上报给上位机）
     * 用于上报里程计、电量、音量等状态数据
     */
    typedef struct
    {
        uint8_t start_flag; ///< 起始标志 0xAA
        uint8_t length;     ///< 数据包长度

        // 里程计数据
        uint32_t left_encoder;  ///< 左轮编码器计数
        uint32_t right_encoder; ///< 右轮编码器计数

        // 系统状态数据
        uint16_t battery_voltage;   ///< 电池电压 (mV)
        uint8_t battery_percentage; ///< 电池电量百分比
        uint8_t volume_level;       ///< 音量等级 (0-100)

        // 传感器数据
        uint16_t temperature; ///< 温度 (0.1°C)

        uint8_t end_flag; ///< 结束标志 0x55
    } __attribute__((packed)) uart_uplink_packet_t;

    /**
     * @brief 下行数据包结构（上位机下发给设备）
     * 用于下发运动控制指令
     */
    typedef struct
    {
        uint8_t start_flag; ///< 起始标志 0xAA
        uint8_t length;     ///< 数据包长度

        // 运动控制指令
        float linear_velocity;  ///< 线速度 (m/s)
        float angular_velocity; ///< 角速度 (rad/s)

        // 控制参数
        uint16_t control_mode; ///< 控制模式
        uint8_t enable_flag;   ///< 使能标志

        uint8_t end_flag; ///< 结束标志 0x55
    } __attribute__((packed)) uart_downlink_packet_t;

    /**
     * @brief 通用数据包联合体
     * 用于统一处理上行和下行数据包
     */
    typedef union
    {
        uart_uplink_packet_t uplink;     ///< 上行数据包
        uart_downlink_packet_t downlink; ///< 下行数据包
        uint8_t raw_data[17];            ///< 原始数据（最大包长度）
    } uart_packet_union_t;

    /**
     * @brief 数据包解析器结构
     */
    typedef struct
    {
        packet_parse_state_t state; ///< 解析状态
        uart_packet_union_t packet; ///< 当前数据包
        uint8_t data_index;         ///< 数据索引
        uint32_t timeout_start;     ///< 超时开始时间
    } packet_parser_t;

    /**
     * @brief 错误统计结构
     */
    typedef struct
    {
        uint32_t timeout_count;       ///< 超时次数
        uint32_t format_error_count;  ///< 格式错误次数
        uint32_t total_rx_count;      ///< 总接收次数
        uint32_t total_tx_count;      ///< 总发送次数
        uart_error_type_t last_error; ///< 最后错误类型
    } uart_error_stats_t;

    /**
     * @brief UART通信控制结构
     */
    typedef struct
    {
        uart_comm_state_t state;     ///< 通信状态
        QueueHandle_t uart_queue;    ///< UART事件队列
        QueueHandle_t tx_queue;      ///< 发送队列
        SemaphoreHandle_t tx_mutex;  ///< 发送互斥锁
        SemaphoreHandle_t rx_mutex;  ///< 接收互斥锁
        TaskHandle_t rx_task_handle; ///< 接收任务句柄
        TaskHandle_t tx_task_handle; ///< 发送任务句柄

        packet_parser_t parser;         ///< 数据包解析器
        uart_error_stats_t error_stats; ///< 错误统计
        bool is_initialized;            ///< 初始化标志
    } uart_comm_control_t;

    /**
     * @brief 接收回调函数类型
     * @param packet 接收到的数据包指针
     * @param user_data 用户数据指针
     */
    typedef void (*uart_rx_callback_t)(const uart_downlink_packet_t *packet, void *user_data);

    // =============================================================================
    // API函数声明
    // =============================================================================

    /**
     * @brief 初始化UART通信模块
     * @param rx_callback 接收回调函数
     * @param user_data 用户数据指针
     * @retval ESP_OK 初始化成功
     * @retval ESP_FAIL 初始化失败
     */
    esp_err_t uart_comm_init(uart_rx_callback_t rx_callback, void *user_data);

    /**
     * @brief 反初始化UART通信模块
     * @retval ESP_OK 反初始化成功
     * @retval ESP_FAIL 反初始化失败
     */
    esp_err_t uart_comm_deinit(void);

    /**
     * @brief 发送上行数据包（非阻塞）
     * @param packet 上行数据包指针
     * @retval ESP_OK 发送成功
     * @retval ESP_ERR_INVALID_ARG 参数错误
     * @retval ESP_ERR_INVALID_STATE 状态错误
     * @retval ESP_ERR_NO_MEM 内存不足
     */
    esp_err_t uart_comm_send_uplink_packet(const uart_uplink_packet_t *packet);

    /**
     * @brief 获取通信状态
     * @retval uart_comm_state_t 当前通信状态
     */
    uart_comm_state_t uart_comm_get_state(void);

    /**
     * @brief 获取错误统计信息
     * @param stats 错误统计结构指针
     * @retval ESP_OK 获取成功
     * @retval ESP_ERR_INVALID_ARG 参数错误
     */
    esp_err_t uart_comm_get_error_stats(uart_error_stats_t *stats);

    /**
     * @brief 清除错误统计
     * @retval ESP_OK 清除成功
     */
    esp_err_t uart_comm_clear_error_stats(void);

    /**
     * @brief 设置接收回调函数
     * @param rx_callback 接收回调函数
     * @param user_data 用户数据指针
     * @retval ESP_OK 设置成功
     * @retval ESP_ERR_INVALID_ARG 参数错误
     */
    esp_err_t uart_comm_set_rx_callback(uart_rx_callback_t rx_callback, void *user_data);

    /**
     * @brief 验证上行数据包格式
     * @param packet 上行数据包指针
     * @retval true 格式正确
     * @retval false 格式错误
     */
    bool uart_comm_validate_uplink_packet(const uart_uplink_packet_t *packet);

    /**
     * @brief 验证下行数据包格式
     * @param packet 下行数据包指针
     * @retval true 格式正确
     * @retval false 格式错误
     */
    bool uart_comm_validate_downlink_packet(const uart_downlink_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif /* __CONNECT_UART_H__ */