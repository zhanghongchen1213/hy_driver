#ifndef __APP_BLUE_CONTROL_H__
#define __APP_BLUE_CONTROL_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "gatts_server.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// 蓝牙数据发送频率配置
#define BLE_SEND_INTERVAL_DEBUG_MS 30 // 调试模式：30ms，用于实时监控

/// @module 蓝牙控制
/// @brief 蓝牙双向控制接口
/// @warning 需要确保控制命令格式与客户端一致
/// @brief 控制命令结构体
typedef struct
{
    float linear_vel;   // 线速度
    float angular_vel;  // 角速度
    uint32_t timestamp; // 时间戳
} ControlCommand;

/// @brief 蓝牙接收数据结构体
typedef struct
{
    uint8_t data[36]; // 接收的字节数据
    uint16_t len;     // 数据长度
} wxx_received_data_t;

void rtos_blue_control_init(void);

#endif
