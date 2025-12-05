#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "iot_servo.h"

#define SERVO_GPIO_A 8  // 右手舵机
#define SERVO_GPIO_B 19 // 左手舵机
#define SERVO_GPIO_C 20 // 腰部舵机

#define SERVO_LEDC_A LEDC_CHANNEL_0
#define SERVO_LEDC_B LEDC_CHANNEL_1
#define SERVO_LEDC_C LEDC_CHANNEL_2

// 统一角度系统定义
#define SERVO_UNIFIED_ZERO_ANGLE 0 // 统一角度系统的零位角度
#define SERVO_UNIFIED_MAX_ANGLE 60 // 统一角度系统的最大角度

// 各舵机的零位实际角度修正量（系统上电时的零位状态）
#define SERVO_A_ZERO_POSITION 0   // SERVO_A零位时的实际角度
#define SERVO_B_ZERO_POSITION 180 // SERVO_B零位时的实际角度
#define SERVO_C_ZERO_POSITION 30  // SERVO_C零位时的实际角度（包含DEBUG_ANGLE）

typedef enum
{
    SERVO_A = 0,
    SERVO_B,
    SERVO_C,
} servo_id_t;

typedef struct
{
    int16_t angle_a;
    int16_t angle_b;
    int16_t angle_c;
} servo_angles_t;

void servo_init(void);
void servo_reset(void);
void servo_set_angle(servo_id_t id, int16_t angle);
servo_angles_t servo_get_actual_angles(void);

#endif
