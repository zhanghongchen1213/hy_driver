#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "esp_err.h"
#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "string.h"

// TB6612FNG GPIO引脚定义
#define PWMA 9
#define AIN1 11
#define AIN2 10

#define PWMB 14
#define BIN1 12
#define BIN2 13

// 编码器GPIO引脚定义
#define ENCODER_A_PHASE_A_PIN 3  ///< 电机A编码器A相引脚
#define ENCODER_A_PHASE_B_PIN 46 ///< 电机A编码器B相引脚
#define ENCODER_B_PHASE_A_PIN 21 ///< 电机B编码器A相引脚
#define ENCODER_B_PHASE_B_PIN 47 ///< 电机B编码器B相引脚

// 编码器参数定义 - N20电机官方参数
#define ENCODER_PPR 7              ///< 编码器每转脉冲数（基础脉冲，7PPR）
#define ENCODER_OUTPUT_PPR 2654.1f ///< 减速后输出轴编码线数（官方标准：2654.1线/转）
#define ENCODER_SAMPLE_TIME_MS 1   ///< 编码器采样时间间隔（毫秒）

// PCNT计数器限制值定义
#define PCNT_UNIT_MAX_COUNT 32767  ///< PCNT单元最大计数值
#define PCNT_UNIT_MIN_COUNT -32768 ///< PCNT单元最小计数值

// 机械参数定义
#define WHEEL_DIAMETER_MM 65.0f                           ///< 轮子直径（毫米），空载转速58RPM,等效200mm/s
#define WHEEL_BASE_MM 150.0f                              ///< 轮距（毫米）- 两轮中心距离
#define WHEEL_CIRCUMFERENCE_MM (M_PI * WHEEL_DIAMETER_MM) ///< 轮子周长（毫米）

// 电机标识枚举
typedef enum
{
    MOTOR_A = 0,   ///< 电机A，左轮(左侧接口)
    MOTOR_B = 1,   ///< 电机B，右轮(下方接口)
    MOTOR_BOTH = 2 ///< 双电机
} motor_id_t;

// 电机方向枚举
typedef enum
{
    MOTOR_DIRECTION_FORWARD = 0, ///< 正转
    MOTOR_DIRECTION_REVERSE = 1  ///< 反转
} motor_direction_t;

// 电机停止方式枚举
typedef enum
{
    MOTOR_STOP_COAST = 0, ///< 滑行停止（快速衰减）
    MOTOR_STOP_BRAKE = 1  ///< 刹车停止（慢速衰减）
} motor_stop_mode_t;

// 编码器模式枚举
typedef enum
{
    ENCODER_MODE_X1 = 0, ///< 1倍频模式
    ENCODER_MODE_X2 = 1, ///< 2倍频模式
    ENCODER_MODE_X4 = 2  ///< 4倍频模式
} encoder_mode_t;

// 编码器结构体
typedef struct
{
    pcnt_unit_handle_t pcnt_unit; ///< 脉冲计数器单元句柄
    int32_t pulse_count;          ///< 脉冲计数值
    float speed_rpm;              ///< 转速（转/分钟）
    encoder_mode_t mode;          ///< 编码器模式
    bool is_initialized;          ///< 初始化状态
} encoder_t;

/**
 * @brief TB6612FNG电机控制结构体
 */
typedef struct
{
    gpio_num_t ain1_pin;                 ///< 电机A方向控制引脚1
    gpio_num_t ain2_pin;                 ///< 电机A方向控制引脚2
    gpio_num_t pwma_pin;                 ///< 电机A速度控制PWM引脚
    gpio_num_t bin1_pin;                 ///< 电机B方向控制引脚1
    gpio_num_t bin2_pin;                 ///< 电机B方向控制引脚2
    gpio_num_t pwmb_pin;                 ///< 电机B速度控制PWM引脚
    mcpwm_timer_handle_t timer_handle;   ///< MCPWM定时器句柄
    mcpwm_oper_handle_t operator_handle; ///< MCPWM操作器句柄
    mcpwm_cmpr_handle_t cmpr_a_handle;   ///< 电机A比较器句柄
    mcpwm_cmpr_handle_t cmpr_b_handle;   ///< 电机B比较器句柄
    mcpwm_gen_handle_t gen_a_handle;     ///< 电机A生成器句柄
    mcpwm_gen_handle_t gen_b_handle;     ///< 电机B生成器句柄
    bool is_initialized;                 ///< 系统初始化状态
    bool motor_a_enabled;                ///< 电机A使能状态
    bool motor_b_enabled;                ///< 电机B使能状态
} tb6612fng_motor_t;

// 双电机控制结构体
typedef struct
{
    tb6612fng_motor_t tb6612fng; ///< TB6612FNG电机驱动器
    encoder_t encoder_a;         ///< 电机A编码器
    encoder_t encoder_b;         ///< 电机B编码器
    bool is_initialized;         ///< 初始化状态
} dual_motor_t;

// 车体运动状态结构体
typedef struct
{
    float motor_a_rpm;     ///< 电机A输出轴实际转速（转/分钟）
    float motor_b_rpm;     ///< 电机B输出轴实际转速（转/分钟）
    uint32_t timestamp_ms; ///< 数据时间戳（毫秒）
    bool data_valid;       ///< 数据有效性标志
} vehicle_motion_state_t;

/**
 * @brief 双电机系统初始化
 */
esp_err_t motor_init(void);

/**
 * @brief 控制双电机停止
 * @param motor_a_stop_mode 电机A停止方式
 * @param motor_b_stop_mode 电机B停止方式
 */
esp_err_t motor_stop(motor_stop_mode_t motor_a_stop_mode, motor_stop_mode_t motor_b_stop_mode);

/**
 * @brief 激活双电机使能状态
 */
esp_err_t motor_enable(void);

/**
 * @brief 解除双电机使能状态，急停功能
 */
esp_err_t motor_disable(void);

/**
 * @brief 设置指定电机转速和方向
 * @param motor_id 目标电机标识
 * @param speed 设定的转速值（0-100，对应0%-100%占空比）
 * @param direction 运动方向（前进/后退）
 */
esp_err_t motor_set_speed_and_dir(motor_id_t motor_id, uint32_t speed, motor_direction_t direction);

/**
 * @brief 获取电机的实际转速
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param rpm 输出转速值（转/分钟）
 */
esp_err_t motor_get_rpm(motor_id_t motor_id, float *rpm);

#endif