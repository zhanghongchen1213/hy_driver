#include "app_move_control.h"

static const char *TAG = "move_control";

// 控制模式中文描述数组
static const char *control_mode_str[] = {
    "停止模式",
    "速度控制模式",
    "位置控制模式",
    "双环控制模式",
    "转向控制模式",
    "刹车模式"};

// 运动方向中文描述数组
static const char *direction_str[] = {
    "前进",
    "后退",
    "左转",
    "右转",
    "停止"};

TaskHandle_t move_control_task_handle;
TaskHandle_t monitor_task_handle;

void move_control_task(void *pvParameters);
void monitor_task(void *pvParameters);

/**
 * @brief 履带小车基本控制示例
 */
void track_basic_control_example(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== 履带小车基本控制示例 ===");

    // 1. 初始化控制系统
    ESP_LOGI(TAG, "1. 初始化控制系统");
    ret = track_control_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "控制系统初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. 速度控制示例 - 前进
    ESP_LOGI(TAG, "2. 速度控制 - 前进 200 rpm");
    ret = track_set_linear_speed(TRACK_DIRECTION_FORWARD, 3.0f);
    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1500)); // 运行1.5秒
    }

    // 3. 刹车
    ESP_LOGI(TAG, "3. 刹车停止");
    track_brake();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 4. 速度控制示例 - 后退
    ESP_LOGI(TAG, "4. 速度控制 - 后退 150 rpm");
    ret = track_set_linear_speed(TRACK_DIRECTION_BACKWARD, 1.0f);
    if (ret == ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1500)); // 运行1.5秒
    }

    // 5. 刹车
    ESP_LOGI(TAG, "5. 刹车停止");
    track_brake();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "基本控制示例完成");
}

/**
 * @brief 履带小车位置控制示例
 */
void track_position_control_example(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== 履带小车位置控制示例 ===");

    // 1. 初始化控制系统
    ESP_LOGI(TAG, "1. 初始化控制系统");
    ret = track_control_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "控制系统初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 1. 位置控制示例 - 前进100mm
    ESP_LOGI(TAG, "1. 位置控制 - 前进 100 mm");
    ret = track_set_linear_position(TRACK_DIRECTION_FORWARD, 100.0f);
    if (ret == ESP_OK)
    {
        // 等待到达目标位置（最多10秒）
        for (int i = 0; i < 100; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "到达目标位置，实际行驶距离: %.2f mm", state.current_distance);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. 位置控制示例 - 后退100mm
    ESP_LOGI(TAG, "2. 位置控制 - 后退 100 mm");
    ret = track_set_linear_position(TRACK_DIRECTION_BACKWARD, 100.0f);
    if (ret == ESP_OK)
    {
        // 等待到达目标位置
        for (int i = 0; i < 100; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "到达目标位置，实际行驶距离: %.2f mm", state.current_distance);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    ESP_LOGI(TAG, "位置控制示例完成");
}

/**
 * @brief 履带小车双环控制示例
 */
void track_dual_loop_control_example(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== 履带小车双环控制示例 ===");

    // 重置系统
    track_control_reset();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1. 双环控制示例 - 前进1000mm，最大速度300rpm
    ESP_LOGI(TAG, "1. 双环控制 - 前进 1000 mm, 最大速度 300 rpm");
    ret = track_set_dual_loop_control(TRACK_DIRECTION_FORWARD, 1000.0f, 300.0f);
    if (ret == ESP_OK)
    {
        // 等待到达目标位置
        for (int i = 0; i < 200; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "双环控制到达目标位置，实际行驶距离: %.2f mm", state.current_distance);
                    break;
                }

                // 每秒打印一次状态
                if (i % 10 == 0)
                {
                    ESP_LOGI(TAG, "当前距离: %.2f mm, 左轮速度: %.2f rpm, 右轮速度: %.2f rpm",
                             state.current_distance, state.current_speed_left, state.current_speed_right);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    ESP_LOGI(TAG, "双环控制示例完成");
}

/**
 * @brief 履带小车转向控制示例
 */
void track_turning_control_example(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== 履带小车转向控制示例 ===");

    // 重置系统
    track_control_reset();
    vTaskDelay(pdMS_TO_TICKS(500));

    // 1. 右转90度
    ESP_LOGI(TAG, "1. 右转 90 度，转向速度 100 rpm");
    ret = track_set_turn_control(TRACK_DIRECTION_TURN_RIGHT, 90.0f, 100.0f);
    if (ret == ESP_OK)
    {
        // 等待转向完成
        for (int i = 0; i < 100; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "右转完成，当前角度: %.2f 度", state.current_angle);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. 左转180度
    ESP_LOGI(TAG, "2. 左转 180 度，转向速度 80 rpm");
    ret = track_set_turn_control(TRACK_DIRECTION_TURN_LEFT, 180.0f, 80.0f);
    if (ret == ESP_OK)
    {
        // 等待转向完成
        for (int i = 0; i < 150; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "左转完成，当前角度: %.2f 度", state.current_angle);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3. 右转90度回到初始方向
    ESP_LOGI(TAG, "3. 右转 90 度回到初始方向");
    ret = track_set_turn_control(TRACK_DIRECTION_TURN_RIGHT, 90.0f, 100.0f);
    if (ret == ESP_OK)
    {
        // 等待转向完成
        for (int i = 0; i < 100; i++)
        {
            track_vehicle_control_t state;
            if (track_get_control_state(&state) == ESP_OK)
            {
                if (!state.is_running)
                {
                    ESP_LOGI(TAG, "转向完成，当前角度: %.2f 度", state.current_angle);
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    ESP_LOGI(TAG, "转向控制示例完成");
}

/**
 * @brief 履带小车安全功能示例
 */
void track_safety_example(void)
{
    ESP_LOGI(TAG, "=== 履带小车安全功能示例 ===");

    // 1. 正常运行
    ESP_LOGI(TAG, "1. 启动正常运行");
    track_set_linear_speed(TRACK_DIRECTION_FORWARD, 200.0f);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 2. 紧急停止
    ESP_LOGI(TAG, "2. 触发紧急停止");
    track_emergency_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 3. 尝试在紧急停止状态下运行（应该失败）
    ESP_LOGI(TAG, "3. 尝试在紧急停止状态下运行");
    esp_err_t ret = track_set_linear_speed(TRACK_DIRECTION_FORWARD, 100.0f);
    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "紧急停止状态下无法启动运行（正常行为）");
    }

    // 4. 重置系统恢复正常
    ESP_LOGI(TAG, "4. 重置系统恢复正常");
    track_control_reset();

    // 重新初始化
    track_control_init();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 5. 验证系统恢复正常
    ESP_LOGI(TAG, "5. 验证系统恢复正常");
    ret = track_set_linear_speed(TRACK_DIRECTION_FORWARD, 100.0f);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "系统恢复正常，可以正常运行");
        vTaskDelay(pdMS_TO_TICKS(1000));
        track_brake();
    }

    ESP_LOGI(TAG, "安全功能示例完成");
}

void rtos_move_control_init(void)
{
    //*=====创建运动控制任务=====*//
    BaseType_t result = pdPASS;
    result = xTaskCreatePinnedToCore(move_control_task, "move_control_task", 4 * 1024, NULL, 3, &move_control_task_handle, 1);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "运动控制任务创建失败");
    }
    result = xTaskCreatePinnedToCore(monitor_task, "monitor_task", 4 * 1024, NULL, 3, &monitor_task_handle, 1);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "监控任务创建失败");
    }

    // 等待任务启动
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 依次执行各种控制示例
    // 速度环控制示例
    track_basic_control_example();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 位置环控制示例
    // track_position_control_example();
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // track_dual_loop_control_example();
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // track_turning_control_example();
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // track_safety_example();

    ESP_LOGI(TAG, "=== 履带小车综合测试完成 ===");
}

void move_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "履带小车控制任务启动");

    while (1)
    {
        // 周期性调用控制任务
        esp_err_t ret = track_control_task();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "控制任务执行失败: %s", esp_err_to_name(ret));
        }

        // 控制周期：20ms (50Hz)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void monitor_task(void *pvParameters)
{
    track_vehicle_control_t control_state;

    ESP_LOGI(TAG, "履带小车状态监控任务启动");

    while (1)
    {
        // 获取控制状态
        esp_err_t ret = track_get_control_state(&control_state);
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "=== 履带小车状态 ===");
            ESP_LOGI(TAG, "控制模式: %s, 运动方向: %s",
                     control_mode_str[control_state.control_mode],
                     direction_str[control_state.direction]);
            ESP_LOGI(TAG, "运行状态: %s, 紧急停止: %s",
                     control_state.is_running ? "运行中" : "停止",
                     control_state.emergency_stop ? "是" : "否");
            ESP_LOGI(TAG, "左轮 - 目标速度:%.2f rpm, 当前速度:%.2f rpm, PWM:%d",
                     control_state.target_speed_left, control_state.current_speed_left, control_state.pwm_left);
            ESP_LOGI(TAG, "右轮 - 目标速度:%.2f rpm, 当前速度:%.2f rpm, PWM:%d",
                     control_state.target_speed_right, control_state.current_speed_right, control_state.pwm_right);
            ESP_LOGI(TAG, "位置 - 左轮:%.2f°, 右轮:%.2f°",
                     control_state.current_position_left, control_state.current_position_right);
            ESP_LOGI(TAG, "里程 - 距离:%.2f mm, 角度:%.2f°",
                     control_state.current_distance, control_state.current_angle);
            ESP_LOGI(TAG, "========================");
        }
        else
        {
            ESP_LOGE(TAG, "获取控制状态失败: %s", esp_err_to_name(ret));
        }

        // 监控周期：0.5秒
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}