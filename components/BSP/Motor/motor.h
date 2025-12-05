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
#define ENCODER_POLES 14           ///< 磁环触发极数（14极，7对极）
#define ENCODER_GEAR_RATIO 380.0f  ///< 减速比（1:380）
#define ENCODER_OUTPUT_PPR 2654.1f ///< 减速后输出轴编码线数（官方标准：2654.1线/转）
#define ENCODER_SAMPLE_TIME_MS 10  ///< 转速采样时间间隔（毫秒）

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
    float motor_a_rpm;          ///< 电机A转速（转/分钟）
    float motor_b_rpm;          ///< 电机B转速（转/分钟）
    float motor_a_position_deg; ///< 电机A位置角度（度）
    float motor_b_position_deg; ///< 电机B位置角度（度）
    float travel_distance_mm;   ///< 累积行驶距离（毫米）
    float vehicle_angle_deg;    ///< 车体转角（度，正值为逆时针）
    uint32_t timestamp_ms;      ///< 数据时间戳（毫秒）
    bool data_valid;            ///< 数据有效性标志
} vehicle_motion_state_t;

/**
 * @brief 双电机系统初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_init(void);

/**
 * @brief 激活双电机使能状态
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_enable(void);

/**
 * @brief 控制双电机停止
 * @param motor_a_stop_mode 电机A停止方式
 * @param motor_b_stop_mode 电机B停止方式
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_stop(motor_stop_mode_t motor_a_stop_mode, motor_stop_mode_t motor_b_stop_mode);

/**
 * @brief 解除双电机使能状态
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_disable(void);

/**
 * @brief 设置指定电机转速
 * @param motor_id 目标电机标识
 * @param speed 设定的转速值（0-10000，对应0%-100%占空比）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_set_speed(motor_id_t motor_id, uint32_t speed);

/**
 * @brief 控制指定电机运动方向
 * @param motor_id 目标电机标识
 * @param direction 运动方向（前进/后退）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_set_direction(motor_id_t motor_id, motor_direction_t direction);

/**
 * @brief 编码器初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t encode_init(void);

/**
 * @brief 获取指定电机的转速
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param rpm 输出转速值（转/分钟）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_get_rpm(motor_id_t motor_id, float *rpm);

/**
 * @brief 获取指定电机的位置角度
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param position 输出位置角度值（度）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_get_position(motor_id_t motor_id, float *position);

/**
 * @brief 获取车体累积行驶距离
 * @param distance 输出累积行驶距离（毫米）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 基于双轮编码器数据计算的累积行驶距离
 */
esp_err_t motor_get_travel_distance(float *distance);

/**
 * @brief 获取车体转角
 * @param angle 输出车体转角（度）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 基于差分驱动模型计算的车体转角，正值为逆时针转向
 */
esp_err_t motor_get_vehicle_angle(float *angle);

/**
 * @brief 重置里程计数据
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 清零累积行驶距离和车体转角数据
 */
esp_err_t motor_reset_odometry(void);

/**
 * @brief 获取完整的车体运动状态
 * @param[out] motion_state 车体运动状态结构体指针
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 编码器未初始化
 */
esp_err_t motor_get_motion_state(vehicle_motion_state_t *motion_state);

#endif