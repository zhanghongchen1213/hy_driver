#include "servo_control.h"

static const char *TAG = "SERVO";
static int16_t s_servo_actual_angles[3] = {0, 0, 0};
void servo_init(void)
{
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_GPIO_A,
                SERVO_GPIO_B,
                SERVO_GPIO_C,
            },
            .ch = {
                SERVO_LEDC_A,
                SERVO_LEDC_B,
                SERVO_LEDC_C,
            },
        },
        .channel_number = 3,
    };

    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    servo_reset();
    servo_angles_t angles = servo_get_actual_angles();
    ESP_LOGI(TAG, "舵机初始化完成 - 实际角度A:%d°, B:%d°, C:%d°", angles.angle_a, angles.angle_b, angles.angle_c);
}

void servo_reset(void)
{
    // 使用统一角度系统设置所有舵机到零位（统一角度0°）
    servo_set_angle(SERVO_A, SERVO_UNIFIED_ZERO_ANGLE);
    servo_set_angle(SERVO_B, SERVO_UNIFIED_ZERO_ANGLE);
    servo_set_angle(SERVO_C, SERVO_UNIFIED_ZERO_ANGLE);
    ESP_LOGI(TAG, "舵机复位完成 - 所有舵机设置为统一角度%d°", SERVO_UNIFIED_ZERO_ANGLE);
}

void servo_set_angle(servo_id_t id, int16_t unified_angle)
{
    int16_t actual_angle = 0;

    // 参数范围检查
    if (unified_angle < SERVO_UNIFIED_ZERO_ANGLE || unified_angle > SERVO_UNIFIED_MAX_ANGLE)
    {
        ESP_LOGW(TAG, "统一角度%d超出范围[%d-%d]", unified_angle, SERVO_UNIFIED_ZERO_ANGLE, SERVO_UNIFIED_MAX_ANGLE);
        return;
    }

    switch (id)
    {
    case SERVO_A:
        // 右手舵机：统一角度直接映射到实际角度
        // 统一角度0° -> 实际角度0°，统一角度180° -> 实际角度180°
        actual_angle = SERVO_A_ZERO_POSITION + unified_angle;
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_LEDC_A, actual_angle);
        s_servo_actual_angles[0] = unified_angle;
        ESP_LOGD(TAG, "舵机A：统一角度%d° -> 实际角度%d°", unified_angle, actual_angle);
        break;

    case SERVO_B:
        // 左手舵机：统一角度反向映射到实际角度
        // 统一角度0° -> 实际角度180°，统一角度180° -> 实际角度0°
        actual_angle = SERVO_B_ZERO_POSITION - unified_angle;
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_LEDC_B, actual_angle);
        s_servo_actual_angles[1] = unified_angle;
        ESP_LOGD(TAG, "舵机B：统一角度%d° -> 实际角度%d°", unified_angle, actual_angle);
        break;

    case SERVO_C:
        // 腰部舵机：统一角度加上零位偏移
        // 统一角度0° -> 实际角度30°，统一角度180° -> 实际角度210°
        actual_angle = SERVO_C_ZERO_POSITION + unified_angle;
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, SERVO_LEDC_C, actual_angle);
        s_servo_actual_angles[2] = unified_angle;
        ESP_LOGD(TAG, "舵机C：统一角度%d° -> 实际角度%d°", unified_angle, actual_angle);
        break;

    default:
        ESP_LOGW(TAG, "无效的舵机ID: %d", id);
        break;
    }
}

servo_angles_t servo_get_actual_angles(void)
{
    servo_angles_t angles;
    angles.angle_a = s_servo_actual_angles[0];
    angles.angle_b = s_servo_actual_angles[1];
    angles.angle_c = s_servo_actual_angles[2];
    return angles;
}
