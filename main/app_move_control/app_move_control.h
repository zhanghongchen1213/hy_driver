#ifndef __APP_MOVE_CONTROL_H__
#define __APP_MOVE_CONTROL_H__

#include "all_include.h"
#include "app_pid_control.h"

// 机械参数定义
#define WHEEL_DIAMETER_MM 65.0f                           ///< 轮子直径（毫米），空载转速58RPM,等效200mm/s
#define WHEEL_BASE_MM 150.0f                              ///< 轮距（毫米）- 两轮中心距离
#define WHEEL_CIRCUMFERENCE_MM (M_PI * WHEEL_DIAMETER_MM) ///< 轮子周长（毫米）

typedef struct
{
    float x;             // x轴上的位置
    float y;             // y轴上的位置
    float angle;         // 角度
    float linear_speed;  // 线速度
    float angular_speed; // 角速度
} odom_t;

#endif // __APP_MOVE_CONTROL_H__