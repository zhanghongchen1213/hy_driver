#ifndef __APP_UART_CONTROL_H__
#define __APP_UART_CONTROL_H__

#include "all_include.h"

// =============================================================================
// 数据包协议定义
// =============================================================================

#define SEND_PACKET_START_FLAG 0xAA ///< 发送包起始标志
#define SEND_PACKET_END_FLAG 0x55   ///< 发送包结束标志

#define RECEIVE_PACKET_START_FLAG 0xAA ///< 接收包起始标志
#define RECEIVE_PACKET_END_FLAG 0x66   ///< 接收包结束标志

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

    uint16_t chat_gpt_count; ///< 用户触发CHAT_GPT的次数统计

    // --- 电机与PID状态 (调试用) ---
    float left_target_speed;  ///< 左电机目标速度
    float right_target_speed; ///< 右电机目标速度
    float left_actual_speed;  ///< 左电机实际速度
    float right_actual_speed; ///< 右电机实际速度
    float left_kp;
    float left_ki;
    float left_kd;
    float right_kp;
    float right_ki;
    float right_kd;

#if UPDATE_ODOMETRY_DEBUG
    // --- 里程计与姿态 (用于SLAM/Nav/RVIZ) ---
    float position_x; ///< 位置 X (mm)
    float position_y; ///< 位置 Y (mm)

    // 姿态四元数 (用于数字孪生)
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    // --- 关节状态 (用于Joint State) ---
    int16_t servo_a_angle;
    int16_t servo_b_angle;
    int16_t servo_c_angle;
#endif

    uint32_t timestamp; ///< 时间戳 (ms)
    uint8_t end_flag;   ///< 结束标志 0x55
} __attribute__((packed)) uart_uplink_packet_t;

/**
 * @brief 下行数据包结构（上位机下发给设备）
 * 用于下发运动控制指令
 */
typedef struct
{
    uint8_t start_flag; ///< 起始标志 0xAA

    uint8_t audio_stream_flag; ///< 音频流标志位，0表示未触发，1表示语音识别结束，2表示TTS合成结束，3表示端侧HTTPS下发完成，4表示端侧HTTPS接收完成

    float left_target_speed;  ///< 左电机目标速度
    float right_target_speed; ///< 右电机目标速度
    float left_kp;            ///< 左电机PID比例系数
    float left_ki;            ///< 左电机PID积分系数
    float left_kd;            ///< 左电机PID微分系数
    float right_kp;           ///< 右电机PID比例系数
    float right_ki;           ///< 右电机PID积分系数
    float right_kd;           ///< 右电机PID微分系数

#if UPDATE_ODOMETRY_DEBUG
    float linear_vel;    ///< 线速度 (mm/s)
    float angular_vel;   ///< 角速度 (rad/s)
    float servo_a_angle; ///< 舵机A角度 (°)
    float servo_b_angle; ///< 舵机B角度 (°)
    float servo_c_angle; ///< 舵机C角度 (°)
#endif

    uint32_t timestamp; ///< 时间戳 (ms)
    uint8_t end_flag;   ///< 结束标志 0x66
} __attribute__((packed)) uart_downlink_packet_t;

typedef enum
{
    WAKE_UP = 0x00,
    CHAT_GPT,
    WAKE_EXIT
} CI_03T_CMD;

void rtos_uart_init(void);

#endif /* __APP_UART_CONTROL_H__ */