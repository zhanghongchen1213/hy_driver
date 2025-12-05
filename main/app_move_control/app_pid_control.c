/**
 * @file app_move_control.c
 * @brief 履带小车运动控制系统实现
 * @author ZHC
 * @date 2025
 * @version 1.0
 */

#include "app_pid_control.h"

static const char *TAG = "pid_control";

// 全局控制系统实例
static track_vehicle_control_t g_track_control = {
    .control_mode = TRACK_CONTROL_STOP,
    .direction = TRACK_DIRECTION_STOP,
    .is_running = false,
    .emergency_stop = false,
    .target_speed_left = 0.0f,
    .target_speed_right = 0.0f,
    .target_position_left = 0.0f,
    .target_position_right = 0.0f,
    .target_distance = 0.0f,
    .target_angle = 0.0f,
    .current_speed_left = 0.0f,
    .current_speed_right = 0.0f,
    .current_position_left = 0.0f,
    .current_position_right = 0.0f,
    .current_distance = 0.0f,
    .current_angle = 0.0f,
    .pwm_left = 0,
    .pwm_right = 0,
    .last_update_time = 0,
    .is_initialized = false};

// 静态函数声明
static esp_err_t init_pi_controller(pi_controller_t *controller, float kp, float ki, float integral_limit, float output_limit);
static void limit_pwm_output(int *pwm_left, int *pwm_right);
static int float_abs(float value);
static esp_err_t update_current_state(void);
static esp_err_t set_motor_pwm(int pwm_left, int pwm_right);

/**
 * @brief 初始化PI控制器
 * @param[out] controller PI控制器结构体指针
 * @param[in] kp 比例系数
 * @param[in] ki 积分系数
 * @param[in] integral_limit 积分限幅
 * @param[in] output_limit 输出限幅
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 */
static esp_err_t init_pi_controller(pi_controller_t *controller, float kp, float ki, float integral_limit, float output_limit)
{
    if (controller == NULL)
    {
        ESP_LOGE(TAG, "PI控制器指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    controller->bias = 0.0f;
    controller->last_bias = 0.0f;
    controller->integral_bias = 0.0f;
    controller->output = 0.0f;
    controller->params.kp = kp;
    controller->params.ki = ki;
    controller->params.integral_limit = integral_limit;
    controller->params.output_limit = output_limit;
    controller->is_initialized = true;

    ESP_LOGI(TAG, "PI控制器初始化完成 - Kp:%.4f, Ki:%.6f", kp, ki);
    return ESP_OK;
}

/**
 * @brief PWM输出限幅
 * @param[in,out] pwm_left 左轮PWM指针
 * @param[in,out] pwm_right 右轮PWM指针
 */
static void limit_pwm_output(int *pwm_left, int *pwm_right)
{
    if (*pwm_left > PWM_MAX_AMPLITUDE)
        *pwm_left = PWM_MAX_AMPLITUDE;
    if (*pwm_left < -PWM_MAX_AMPLITUDE)
        *pwm_left = -PWM_MAX_AMPLITUDE;
    if (*pwm_right > PWM_MAX_AMPLITUDE)
        *pwm_right = PWM_MAX_AMPLITUDE;
    if (*pwm_right < -PWM_MAX_AMPLITUDE)
        *pwm_right = -PWM_MAX_AMPLITUDE;
}

/**
 * @brief 浮点数绝对值
 * @param[in] value 输入值
 * @retval 绝对值
 */
static int float_abs(float value)
{
    return (int)((value < 0) ? -value : value);
}

/**
 * @brief 更新当前状态值
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t update_current_state(void)
{
    esp_err_t ret = ESP_OK;
    vehicle_motion_state_t motion_state;

    // 获取电机运动状态
    ret = motor_get_motion_state(&motion_state);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取电机状态失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 更新当前状态值
    g_track_control.current_speed_left = motion_state.motor_a_rpm; // 电机A作为左轮，符号修正
    g_track_control.current_speed_right = motion_state.motor_b_rpm; // 电机B作为右轮
    g_track_control.current_position_left = motion_state.motor_a_position_deg; // 左轮位置符号修正
    g_track_control.current_position_right = motion_state.motor_b_position_deg;
    g_track_control.current_distance = motion_state.travel_distance_mm;
    g_track_control.current_angle = motion_state.vehicle_angle_deg;

    return ESP_OK;
}

/**
 * @brief 设置电机PWM输出
 * @param[in] pwm_left 左轮PWM值
 * @param[in] pwm_right 右轮PWM值
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t set_motor_pwm(int pwm_left, int pwm_right)
{
    esp_err_t ret = ESP_OK;

    // 设置左轮（电机A）
    if (pwm_left == 0)
    {
        ret = motor_stop(MOTOR_STOP_BRAKE, MOTOR_STOP_COAST);
    }
    else
    {
        motor_direction_t dir_left = (pwm_left > 0) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;
        ret = motor_set_direction(MOTOR_A, dir_left);
        if (ret == ESP_OK)
        {
            ret = motor_set_speed(MOTOR_A, float_abs(pwm_left) * 100 / PWM_MAX_AMPLITUDE);
        }
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置左轮失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置右轮（电机B）
    if (pwm_right == 0)
    {
        ret = motor_stop(MOTOR_STOP_COAST, MOTOR_STOP_BRAKE);
    }
    else
    {
        motor_direction_t dir_right = (pwm_right > 0) ? MOTOR_DIRECTION_FORWARD : MOTOR_DIRECTION_REVERSE;
        ret = motor_set_direction(MOTOR_B, dir_right);
        if (ret == ESP_OK)
        {
            ret = motor_set_speed(MOTOR_B, float_abs(pwm_right) * 100 / PWM_MAX_AMPLITUDE);
        }
    }

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置右轮失败: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief 履带小车控制系统初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t track_control_init(void)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "履带小车控制系统初始化开始");

    // 使能电机
    ret = motor_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机使能失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 重置里程计
    ret = motor_reset_odometry();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "里程计重置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化速度PI控制器
    ret = init_pi_controller(&g_track_control.speed_pi_left, 200.0f, 200.0f, SPEED_INTEGRAL_LIMIT, PWM_MAX_AMPLITUDE);
    if (ret != ESP_OK)
        return ret;

    ret = init_pi_controller(&g_track_control.speed_pi_right, 200.0f, 200.0f, SPEED_INTEGRAL_LIMIT, PWM_MAX_AMPLITUDE);
    if (ret != ESP_OK)
        return ret;

    // 初始化位置PI控制器
    ret = init_pi_controller(&g_track_control.position_pi_left, 0.02f, 0.0002f, POSITION_INTEGRAL_LIMIT, 1000.0f);
    if (ret != ESP_OK)
        return ret;

    ret = init_pi_controller(&g_track_control.position_pi_right, 0.02f, 0.0002f, POSITION_INTEGRAL_LIMIT, 1000.0f);
    if (ret != ESP_OK)
        return ret;

    // 初始化控制系统状态
    g_track_control.control_mode = TRACK_CONTROL_STOP;
    g_track_control.direction = TRACK_DIRECTION_STOP;
    g_track_control.is_running = false;
    g_track_control.emergency_stop = false;
    g_track_control.last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_track_control.is_initialized = true;

    ESP_LOGI(TAG, "履带小车控制系统初始化完成");
    return ESP_OK;
}

/**
 * @brief 位置式PI控制器
 * @param[in,out] controller PI控制器结构体指针
 * @param[in] target 目标值
 * @param[in] current 当前值
 * @retval 控制器输出值
 */
float position_pi_controller(pi_controller_t *controller, float target, float current)
{
    if (controller == NULL || !controller->is_initialized)
    {
        ESP_LOGE(TAG, "PI控制器未初始化");
        return 0.0f;
    }

    // 计算误差
    controller->bias = target - current;

    // 积分项累加
    controller->integral_bias += controller->bias;

    // 积分限幅
    if (controller->integral_bias > controller->params.integral_limit)
        controller->integral_bias = controller->params.integral_limit;
    else if (controller->integral_bias < -controller->params.integral_limit)
        controller->integral_bias = -controller->params.integral_limit;

    // PI控制器输出
    controller->output = controller->params.kp * controller->bias +
                         controller->params.ki * controller->integral_bias;

    // 输出限幅
    if (controller->output > controller->params.output_limit)
        controller->output = controller->params.output_limit;
    else if (controller->output < -controller->params.output_limit)
        controller->output = -controller->params.output_limit;

    return controller->output;
}

/**
 * @brief 增量式PI控制器（基于参考代码Incremental_PI）
 * @param[in,out] controller PI控制器结构体指针
 * @param[in] target 目标值
 * @param[in] current 当前值
 * @retval 控制器输出值
 */
float incremental_pi_controller(pi_controller_t *controller, float target, float current)
{
    if (controller == NULL || !controller->is_initialized)
    {
        ESP_LOGE(TAG, "PI控制器未初始化");
        return 0.0f;
    }

    float increment;

    // 计算当前误差
    controller->bias = target - current;

    // 增量式PI控制算法（基于参考代码）
    increment = controller->params.kp * (controller->bias - controller->last_bias) +
                controller->params.ki * controller->bias;

    // 更新输出
    controller->output += increment;

    // 输出限幅
    if (controller->output > controller->params.output_limit)
        controller->output = controller->params.output_limit;
    else if (controller->output < -controller->params.output_limit)
        controller->output = -controller->params.output_limit;

    // 保存上次误差
    controller->last_bias = controller->bias;

    return controller->output;
}

/**
 * @brief 设置履带小车直线速度控制
 * @param[in] target_speed 目标速度 (rpm)
 * @param[in] direction 运动方向
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_set_linear_speed(track_direction_t direction, float target_speed)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_track_control.emergency_stop)
    {
        ESP_LOGW(TAG, "紧急停止状态，无法设置速度");
        return ESP_ERR_INVALID_STATE;
    }

    if (target_speed < 0 || target_speed > 1000.0f)
    {
        ESP_LOGE(TAG, "目标速度超出范围: %.2f", target_speed);
        return ESP_ERR_INVALID_ARG;
    }

    // 设置控制模式和参数
    g_track_control.control_mode = TRACK_CONTROL_SPEED;
    g_track_control.direction = direction;

    // 根据方向设置目标速度
    switch (direction)
    {
    case TRACK_DIRECTION_FORWARD:
        g_track_control.target_speed_left = target_speed;
        g_track_control.target_speed_right = target_speed;
        break;

    case TRACK_DIRECTION_BACKWARD:
        g_track_control.target_speed_left = -target_speed;
        g_track_control.target_speed_right = -target_speed;
        break;

    case TRACK_DIRECTION_STOP:
        g_track_control.target_speed_left = 0.0f;
        g_track_control.target_speed_right = 0.0f;
        break;

    default:
        ESP_LOGE(TAG, "无效的运动方向: %d", direction);
        return ESP_ERR_INVALID_ARG;
    }

    g_track_control.is_running = (direction != TRACK_DIRECTION_STOP);

    ESP_LOGI(TAG, "设置直线速度控制 - 目标速度:%.2f rpm, 方向:%d", target_speed, direction);
    return ESP_OK;
}

/**
 * @brief 设置履带小车直线位置控制
 * @param[in] target_distance 目标距离 (mm)
 * @param[in] direction 运动方向
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_set_linear_position(track_direction_t direction, float target_distance)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_track_control.emergency_stop)
    {
        ESP_LOGW(TAG, "紧急停止状态，无法设置位置");
        return ESP_ERR_INVALID_STATE;
    }

    if (target_distance < 0 || target_distance > 10000.0f)
    {
        ESP_LOGE(TAG, "目标距离超出范围: %.2f mm", target_distance);
        return ESP_ERR_INVALID_ARG;
    }

    // 更新当前状态
    esp_err_t ret = update_current_state();
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 设置控制模式和参数
    g_track_control.control_mode = TRACK_CONTROL_POSITION;
    g_track_control.direction = direction;
    g_track_control.target_distance = target_distance;

    // 计算目标位置（基于当前位置的相对移动）
    float distance_per_degree = WHEEL_CIRCUMFERENCE_MM / 360.0f;
    float target_position_change = target_distance / distance_per_degree;

    switch (direction)
    {
    case TRACK_DIRECTION_FORWARD:
        g_track_control.target_position_left = g_track_control.current_position_left + target_position_change;
        g_track_control.target_position_right = g_track_control.current_position_right + target_position_change;
        break;

    case TRACK_DIRECTION_BACKWARD:
        g_track_control.target_position_left = g_track_control.current_position_left - target_position_change;
        g_track_control.target_position_right = g_track_control.current_position_right - target_position_change;
        break;

    case TRACK_DIRECTION_STOP:
        g_track_control.target_position_left = g_track_control.current_position_left;
        g_track_control.target_position_right = g_track_control.current_position_right;
        break;

    default:
        ESP_LOGE(TAG, "无效的运动方向: %d", direction);
        return ESP_ERR_INVALID_ARG;
    }

    g_track_control.is_running = (direction != TRACK_DIRECTION_STOP);

    ESP_LOGI(TAG, "设置直线位置控制 - 目标距离:%.2f mm, 方向:%d", target_distance, direction);
    return ESP_OK;
}

/**
 * @brief 设置履带小车双环控制（位置环+速度环）
 * @param[in] target_distance 目标距离 (mm)
 * @param[in] max_speed 最大速度限制 (rpm)
 * @param[in] direction 运动方向
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_set_dual_loop_control(track_direction_t direction, float target_distance, float max_speed)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_track_control.emergency_stop)
    {
        ESP_LOGW(TAG, "紧急停止状态，无法设置双环控制");
        return ESP_ERR_INVALID_STATE;
    }

    if (target_distance < 0 || target_distance > 10000.0f)
    {
        ESP_LOGE(TAG, "目标距离超出范围: %.2f mm", target_distance);
        return ESP_ERR_INVALID_ARG;
    }

    if (max_speed <= 0 || max_speed > 1000.0f)
    {
        ESP_LOGE(TAG, "最大速度超出范围: %.2f rpm", max_speed);
        return ESP_ERR_INVALID_ARG;
    }

    // 先设置位置控制
    esp_err_t ret = track_set_linear_position(target_distance, direction);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 修改控制模式为双环控制
    g_track_control.control_mode = TRACK_CONTROL_DUAL_LOOP;

    // 设置速度限制
    switch (direction)
    {
    case TRACK_DIRECTION_FORWARD:
        g_track_control.target_speed_left = max_speed;
        g_track_control.target_speed_right = max_speed;
        break;

    case TRACK_DIRECTION_BACKWARD:
        g_track_control.target_speed_left = -max_speed;
        g_track_control.target_speed_right = -max_speed;
        break;

    case TRACK_DIRECTION_STOP:
        g_track_control.target_speed_left = 0.0f;
        g_track_control.target_speed_right = 0.0f;
        break;

    default:
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "设置双环控制 - 目标距离:%.2f mm, 最大速度:%.2f rpm, 方向:%d",
             target_distance, max_speed, direction);
    return ESP_OK;
}

/**
 * @brief 设置履带小车转向控制
 * @param[in] target_angle 目标转向角度 (度)
 * @param[in] turn_speed 转向速度 (rpm)
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_set_turn_control(track_direction_t direction, float target_angle, float turn_speed)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (g_track_control.emergency_stop)
    {
        ESP_LOGW(TAG, "紧急停止状态，无法设置转向");
        return ESP_ERR_INVALID_STATE;
    }

    if (target_angle < -360.0f || target_angle > 360.0f)
    {
        ESP_LOGE(TAG, "目标角度超出范围: %.2f 度", target_angle);
        return ESP_ERR_INVALID_ARG;
    }

    // 更新当前状态
    esp_err_t ret = update_current_state();
    if (ret != ESP_OK)
    {
        return ret;
    }

    // 设置控制模式和参数
    g_track_control.control_mode = TRACK_CONTROL_TURN;
    g_track_control.target_angle = g_track_control.current_angle + target_angle;

    // 根据转向方向设置左右轮速度（差分转向）
    g_track_control.direction = direction;

    if (direction == TRACK_DIRECTION_TURN_RIGHT) // 右转
    {
        g_track_control.target_speed_left = turn_speed;   // 左轮正转
        g_track_control.target_speed_right = -turn_speed; // 右轮反转
    }
    else if (direction == TRACK_DIRECTION_TURN_LEFT) // 左转
    {
        g_track_control.target_speed_left = -turn_speed; // 左轮反转
        g_track_control.target_speed_right = turn_speed; // 右轮正转
    }
    else // 停止或其他方向
    {
        g_track_control.target_speed_left = 0.0f;
        g_track_control.target_speed_right = 0.0f;
    }

    g_track_control.is_running = (target_angle != 0);

    ESP_LOGI(TAG, "设置转向控制 - 方向:%d, 目标角度:%.2f 度, 转向速度:%.2f rpm", direction, target_angle, turn_speed);
    return ESP_OK;
}

/**
 * @brief 履带小车刹车
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_brake(void)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 设置刹车状态
    g_track_control.control_mode = TRACK_CONTROL_STOP;
    g_track_control.direction = TRACK_DIRECTION_STOP;
    g_track_control.is_running = false;

    // 清零目标值
    g_track_control.target_speed_left = 0.0f;
    g_track_control.target_speed_right = 0.0f;
    g_track_control.pwm_left = 0;
    g_track_control.pwm_right = 0;

    // 停止电机（刹车模式）
    esp_err_t ret = motor_stop(MOTOR_STOP_BRAKE, MOTOR_STOP_BRAKE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机刹车失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "履带小车刹车完成");
    return ESP_OK;
}

/**
 * @brief 履带小车紧急停止
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_emergency_stop(void)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 设置紧急停止标志
    g_track_control.emergency_stop = true;

    // 立即刹车
    esp_err_t ret = track_brake();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "紧急停止刹车失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 禁用电机
    ret = motor_disable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "紧急停止禁用电机失败: %s", esp_err_to_name(ret));
    }

    ESP_LOGW(TAG, "履带小车紧急停止激活");
    return ESP_OK;
}

/**
 * @brief 履带小车控制任务（需要周期性调用）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t track_control_task(void)
{
    if (!g_track_control.is_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (g_track_control.emergency_stop)
    {
        return ESP_OK; // 紧急停止状态，不执行控制
    }

    // 更新当前状态
    esp_err_t ret = update_current_state();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "更新状态失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 根据控制模式执行相应的控制算法
    switch (g_track_control.control_mode)
    {
    case TRACK_CONTROL_SPEED:
    {
        // 速度环控制
        float pwm_left_f = incremental_pi_controller(&g_track_control.speed_pi_left,
                                                     g_track_control.target_speed_left,
                                                     g_track_control.current_speed_left);
        float pwm_right_f = incremental_pi_controller(&g_track_control.speed_pi_right,
                                                      g_track_control.target_speed_right,
                                                      g_track_control.current_speed_right);

        g_track_control.pwm_left = (int)pwm_left_f;
        g_track_control.pwm_right = (int)pwm_right_f;
        break;
    }

    case TRACK_CONTROL_POSITION:
    {
        // 位置环控制
        float speed_left = position_pi_controller(&g_track_control.position_pi_left,
                                                  g_track_control.target_position_left,
                                                  g_track_control.current_position_left);
        float speed_right = position_pi_controller(&g_track_control.position_pi_right,
                                                   g_track_control.target_position_right,
                                                   g_track_control.current_position_right);

        // 位置环输出作为速度环输入
        float pwm_left_f = incremental_pi_controller(&g_track_control.speed_pi_left,
                                                     speed_left,
                                                     g_track_control.current_speed_left);
        float pwm_right_f = incremental_pi_controller(&g_track_control.speed_pi_right,
                                                      speed_right,
                                                      g_track_control.current_speed_right);

        g_track_control.pwm_left = (int)pwm_left_f;
        g_track_control.pwm_right = (int)pwm_right_f;

        // 检查是否到达目标位置
        float position_error_left = g_track_control.target_position_left - g_track_control.current_position_left;
        float position_error_right = g_track_control.target_position_right - g_track_control.current_position_right;

        if (fabsf(position_error_left) < POSITION_ERROR && fabsf(position_error_right) < POSITION_ERROR) // 位置误差范围
        {
            ESP_LOGI(TAG, "到达目标位置，停止运动");
            track_brake();
        }
        break;
    }

    case TRACK_CONTROL_DUAL_LOOP:
    {
        // 双环控制：外环位置，内环速度
        float speed_left = position_pi_controller(&g_track_control.position_pi_left,
                                                  g_track_control.target_position_left,
                                                  g_track_control.current_position_left);
        float speed_right = position_pi_controller(&g_track_control.position_pi_right,
                                                   g_track_control.target_position_right,
                                                   g_track_control.current_position_right);

        // 速度限幅
        if (speed_left > g_track_control.target_speed_left)
            speed_left = g_track_control.target_speed_left;
        if (speed_left < -g_track_control.target_speed_left)
            speed_left = -g_track_control.target_speed_left;
        if (speed_right > g_track_control.target_speed_right)
            speed_right = g_track_control.target_speed_right;
        if (speed_right < -g_track_control.target_speed_right)
            speed_right = -g_track_control.target_speed_right;

        // 速度环控制
        float pwm_left_f = incremental_pi_controller(&g_track_control.speed_pi_left,
                                                     speed_left,
                                                     g_track_control.current_speed_left);
        float pwm_right_f = incremental_pi_controller(&g_track_control.speed_pi_right,
                                                      speed_right,
                                                      g_track_control.current_speed_right);

        g_track_control.pwm_left = (int)pwm_left_f;
        g_track_control.pwm_right = (int)pwm_right_f;

        // 检查是否到达目标位置
        float position_error_left = g_track_control.target_position_left - g_track_control.current_position_left;
        float position_error_right = g_track_control.target_position_right - g_track_control.current_position_right;

        if (fabsf(position_error_left) < POSITION_ERROR && fabsf(position_error_right) < POSITION_ERROR)
        {
            ESP_LOGI(TAG, "双环控制到达目标位置，停止运动");
            track_brake();
        }
        break;
    }

    case TRACK_CONTROL_TURN:
    {
        // 转向控制
        float angle_error = g_track_control.target_angle - g_track_control.current_angle;

        if (fabsf(angle_error) < TURN_ERROR_ANGLE) // 转向误差范围
        {
            ESP_LOGI(TAG, "到达目标角度，停止转向");
            track_brake();
        }
        else
        {
            // 使用速度环控制转向
            float pwm_left_f = incremental_pi_controller(&g_track_control.speed_pi_left,
                                                         g_track_control.target_speed_left,
                                                         g_track_control.current_speed_left);
            float pwm_right_f = incremental_pi_controller(&g_track_control.speed_pi_right,
                                                          g_track_control.target_speed_right,
                                                          g_track_control.current_speed_right);

            g_track_control.pwm_left = (int)pwm_left_f;
            g_track_control.pwm_right = (int)pwm_right_f;
        }
        break;
    }

    case TRACK_CONTROL_STOP:
    default:
    {
        // 停止状态
        g_track_control.pwm_left = 0;
        g_track_control.pwm_right = 0;
        break;
    }
    }

    // PWM输出限幅
    limit_pwm_output(&g_track_control.pwm_left, &g_track_control.pwm_right);

    // 设置电机PWM输出
    if (g_track_control.is_running)
    {
        ret = set_motor_pwm(g_track_control.pwm_left, g_track_control.pwm_right);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机PWM失败: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    // 更新时间戳
    g_track_control.last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    return ESP_OK;
}

/**
 * @brief 获取履带小车控制状态
 * @param[out] state 控制状态结构体指针
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_get_control_state(track_vehicle_control_t *state)
{
    if (state == NULL)
    {
        ESP_LOGE(TAG, "状态指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 更新当前状态
    esp_err_t ret = update_current_state();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "更新状态失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 复制控制状态
    *state = g_track_control;

    return ESP_OK;
}

/**
 * @brief 重置履带小车控制系统
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_STATE 系统状态无效
 */
esp_err_t track_control_reset(void)
{
    if (!g_track_control.is_initialized)
    {
        ESP_LOGE(TAG, "控制系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 先停止运动
    esp_err_t ret = track_brake();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "重置时刹车失败: %s", esp_err_to_name(ret));
    }

    // 重置里程计
    ret = motor_reset_odometry();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "重置里程计失败: %s", esp_err_to_name(ret));
    }

    // 重置PI控制器
    g_track_control.speed_pi_left.bias = 0.0f;
    g_track_control.speed_pi_left.last_bias = 0.0f;
    g_track_control.speed_pi_left.integral_bias = 0.0f;
    g_track_control.speed_pi_left.output = 0.0f;

    g_track_control.speed_pi_right.bias = 0.0f;
    g_track_control.speed_pi_right.last_bias = 0.0f;
    g_track_control.speed_pi_right.integral_bias = 0.0f;
    g_track_control.speed_pi_right.output = 0.0f;

    g_track_control.position_pi_left.bias = 0.0f;
    g_track_control.position_pi_left.last_bias = 0.0f;
    g_track_control.position_pi_left.integral_bias = 0.0f;
    g_track_control.position_pi_left.output = 0.0f;

    g_track_control.position_pi_right.bias = 0.0f;
    g_track_control.position_pi_right.last_bias = 0.0f;
    g_track_control.position_pi_right.integral_bias = 0.0f;
    g_track_control.position_pi_right.output = 0.0f;

    // 重置控制状态
    g_track_control.control_mode = TRACK_CONTROL_STOP;
    g_track_control.direction = TRACK_DIRECTION_STOP;
    g_track_control.is_running = false;
    g_track_control.emergency_stop = false;

    // 清零目标值和当前值
    g_track_control.target_speed_left = 0.0f;
    g_track_control.target_speed_right = 0.0f;
    g_track_control.target_position_left = 0.0f;
    g_track_control.target_position_right = 0.0f;
    g_track_control.target_distance = 0.0f;
    g_track_control.target_angle = 0.0f;

    g_track_control.current_speed_left = 0.0f;
    g_track_control.current_speed_right = 0.0f;
    g_track_control.current_position_left = 0.0f;
    g_track_control.current_position_right = 0.0f;
    g_track_control.current_distance = 0.0f;
    g_track_control.current_angle = 0.0f;

    g_track_control.pwm_left = 0;
    g_track_control.pwm_right = 0;

    ESP_LOGI(TAG, "履带小车控制系统重置完成");
    return ESP_OK;
}