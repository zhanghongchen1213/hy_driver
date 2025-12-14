#ifndef __APP_PID_CONTROL_H__
#define __APP_PID_CONTROL_H__

#include "all_include.h"

// PID 控制参数结构体
typedef struct
{
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float output_max;   // 最大输出限幅 (例如 100.0)
    float integral_max; // 积分限幅，防止积分饱和
    float dead_zone;    // 死区,代表误差允许的最小范围
} pid_params_t;

// PID 运行时状态结构体
typedef struct
{
    float target_speed; // 目标速度
    float actual_speed; // 实际测量速度
    float error_last;   // 上次误差
    float integral_sum; // 积分累计
    float output;       // PID计算输出
} pid_state_t;

// PID 控制块
typedef struct
{
    motor_id_t motor_id; // 电机ID
    pid_params_t params; // PID参数(KP/KI/KD,输出限幅,积分限幅,死区)
    pid_state_t state;   // PID状态(目标速度,当前设定值,实际速度,上次误差,积分累计,滤波后的速度,输出)
} pid_ctrl_block_t;

/**
 * @brief 初始化PID控制系统
 * @note 创建定时器和控制任务
 */
void app_pid_init(void);

/**
 * @brief 设置目标速度
 * @param motor_id 电机ID (MOTOR_A, MOTOR_B, MOTOR_BOTH)
 * @param speed 目标速度 (RPM)
 */
void app_pid_set_speed(motor_id_t motor_id, float speed);

/**
 * @brief 更新PID参数
 * @param motor_id 电机ID
 * @param kp 比例
 * @param ki 积分
 * @param kd 微分
 */
void app_pid_set_params(motor_id_t motor_id, float kp, float ki, float kd);

/**
 * @brief 获取PID运行状态
 * @param motor_id 电机ID
 * @return 指向控制块的指针
 */
const pid_ctrl_block_t *app_pid_get_status(motor_id_t motor_id);

#endif
