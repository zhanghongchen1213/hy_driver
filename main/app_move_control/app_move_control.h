#ifndef __APP_MOVE_CONTROL_H__
#define __APP_MOVE_CONTROL_H__

#include "all_include.h"

// 机械参数定义
#define WHEEL_DIAMETER_M 0.060f                         ///< 轮子直径（米），空载转速58RPM,等效0.2m/s
#define WHEEL_BASE_M 0.125f                             ///< 轮距（米）- 左右两轮中心距离
#define WHEEL_CIRCUMFERENCE_M (M_PI * WHEEL_DIAMETER_M) ///< 轮子周长（米）

// 简单的四元数结构体
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct
{
    float x;           // x轴上的位置（m）
    float y;           // y轴上的位置（m）
    float angle;       // 角度（rad）
    float linear_vel;  // 线速度（m/s）
    float angular_vel; // 角速度（rad/s）

    float gyro_x; // 陀螺仪x轴角速度（rad/s）
    float gyro_y; // 陀螺仪y轴角速度（rad/s）
    float gyro_z; // 陀螺仪z轴角速度（rad/s）

    quaternion_t q; // 姿态四元数
} odom_t;

/**
 * @brief 初始化运动控制
 */
void app_move_control_init(void);

/**
 * @brief 逆运动学解析：线速度/角速度 -> 左右轮RPM
 * @param linear_vel 线速度 (m/s)
 * @param angular_vel 角速度 (rad/s)
 * @param rpm_left 输出左轮RPM
 * @param rpm_right 输出右轮RPM
 */
void inverse_kinematics(float linear_vel, float angular_vel, float *rpm_left, float *rpm_right);

/**
 * @brief 正运动学解析：左右轮RPM -> 线速度/角速度
 * @param rpm_left 左轮RPM
 * @param rpm_right 右轮RPM
 * @param linear_vel 输出线速度 (m/s)
 * @param angular_vel 输出角速度 (rad/s)
 */
void forward_kinematics(float rpm_left, float rpm_right, float *linear_vel, float *angular_vel);

/**
 * @brief 获取里程计信息
 */
odom_t *get_odometry(void);

#endif // __APP_MOVE_CONTROL_H__