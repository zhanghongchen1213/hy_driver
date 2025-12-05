#ifndef __APP_PID_CONTROL_H__
#define __APP_PID_CONTROL_H__

#include "bsp_drivers.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"

// 控制系统参数定义
#define CONTROL_SAMPLE_TIME_MS 10       ///< 控制周期（毫秒）
#define PWM_MAX_AMPLITUDE 7100          ///< PWM最大幅值
#define POSITION_INTEGRAL_LIMIT 1500000 ///< 位置环积分限幅
#define SPEED_INTEGRAL_LIMIT 10000      ///< 速度环积分限幅
#define POSITION_ERROR 5.0f             ///< 位置误差
#define TURN_ERROR_ANGLE 2.0f           ///< 转向误差角度

// 履带小车控制模式枚举
typedef enum
{
    TRACK_CONTROL_STOP = 0,  ///< 停止模式
    TRACK_CONTROL_SPEED,     ///< 速度控制模式
    TRACK_CONTROL_POSITION,  ///< 位置控制模式
    TRACK_CONTROL_DUAL_LOOP, ///< 双环控制模式（位置外环+速度内环）
    TRACK_CONTROL_TURN,      ///< 转向控制模式
    TRACK_CONTROL_BRAKE      ///< 刹车模式
} track_control_mode_t;

// 履带小车运动方向枚举
typedef enum
{
    TRACK_DIRECTION_FORWARD = 0, ///< 前进
    TRACK_DIRECTION_BACKWARD,    ///< 后退
    TRACK_DIRECTION_TURN_LEFT,   ///< 左转
    TRACK_DIRECTION_TURN_RIGHT,  ///< 右转
    TRACK_DIRECTION_STOP         ///< 停止
} track_direction_t;

// PI控制器参数结构体
typedef struct
{
    float kp;             ///< 比例系数
    float ki;             ///< 积分系数
    float integral_limit; ///< 积分限幅
    float output_limit;   ///< 输出限幅
} pi_controller_params_t;

// PI控制器状态结构体
typedef struct
{
    float bias;                    ///< 当前偏差
    float last_bias;               ///< 上次偏差
    float integral_bias;           ///< 积分偏差
    float output;                  ///< 控制器输出
    pi_controller_params_t params; ///< 控制器参数
    bool is_initialized;           ///< 初始化状态
} pi_controller_t;

// 履带小车控制系统结构体
typedef struct
{
    // 控制模式和状态
    track_control_mode_t control_mode; ///< 当前控制模式
    track_direction_t direction;       ///< 运动方向
    bool is_running;                   ///< 运行状态
    bool emergency_stop;               ///< 紧急停止标志

    // 目标值
    float target_speed_left;     ///< 左轮目标速度（RPM）
    float target_speed_right;    ///< 右轮目标速度（RPM）
    float target_position_left;  ///< 左轮目标位置（度）
    float target_position_right; ///< 右轮目标位置（度）
    float target_distance;       ///< 目标行驶距离（毫米）
    float target_angle;          ///< 目标转向角度（度）

    // 当前状态值
    float current_speed_left;     ///< 左轮当前速度（RPM）
    float current_speed_right;    ///< 右轮当前速度（RPM）
    float current_position_left;  ///< 左轮当前位置（度）
    float current_position_right; ///< 右轮当前位置（度）
    float current_distance;       ///< 当前行驶距离（毫米）
    float current_angle;          ///< 当前转向角度（度）

    // PI控制器
    pi_controller_t speed_pi_left;     ///< 左轮速度PI控制器
    pi_controller_t speed_pi_right;    ///< 右轮速度PI控制器
    pi_controller_t position_pi_left;  ///< 左轮位置PI控制器
    pi_controller_t position_pi_right; ///< 右轮位置PI控制器

    // 控制输出
    int pwm_left;  ///< 左轮PWM输出
    int pwm_right; ///< 右轮PWM输出

    // 时间戳
    uint32_t last_update_time; ///< 上次更新时间戳

    bool is_initialized; ///< 系统初始化状态
} track_vehicle_control_t;

// 函数声明

/**
 * @brief 履带小车控制系统初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t track_control_init(void);

/**
 * @brief 位置式PI控制器
 * @param[in,out] controller PI控制器结构体指针
 * @param[in] current_value 当前值
 * @param[in] target_value 目标值
 * @retval 控制器输出值
 */
float position_pi_controller(pi_controller_t *controller, float current_value, float target_value);

/**
 * @brief 增量式PI控制器
 * @param[in,out] controller PI控制器结构体指针
 * @param[in] current_value 当前值
 * @param[in] target_value 目标值
 * @retval 控制器输出值
 */
float incremental_pi_controller(pi_controller_t *controller, float current_value, float target_value);

/**
 * @brief 设置履带小车直线运动（速度控制）
 * @param[in] direction 运动方向
 * @param[in] speed 目标速度（RPM）
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_set_linear_speed(track_direction_t direction, float speed);

/**
 * @brief 设置履带小车直线运动（位置控制）
 * @param[in] direction 运动方向
 * @param[in] distance 目标距离（毫米）
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_set_linear_position(track_direction_t direction, float distance);

/**
 * @brief 设置履带小车复合控制（双环控制）
 * @param[in] direction 运动方向
 * @param[in] distance 目标距离（毫米）
 * @param[in] max_speed 最大速度限制（RPM）
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_set_dual_loop_control(track_direction_t direction, float distance, float max_speed);

/**
 * @brief 设置履带小车转向控制
 * @param[in] direction 转向方向（左转/右转）
 * @param[in] angle 目标转向角度（度）
 * @param[in] turn_speed 转向速度（RPM）
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_set_turn_control(track_direction_t direction, float target_angle, float turn_speed);

/**
 * @brief 履带小车刹车
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_brake(void);

/**
 * @brief 履带小车紧急停止
 * @retval ESP_OK 成功
 */
esp_err_t track_emergency_stop(void);

/**
 * @brief 履带小车控制任务（需要定期调用）
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_control_task(void);

/**
 * @brief 获取履带小车当前状态
 * @param[out] control_state 控制状态结构体指针
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_get_control_state(track_vehicle_control_t *control_state);

/**
 * @brief 重置履带小车控制系统
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统未初始化
 */
esp_err_t track_control_reset(void);

#endif