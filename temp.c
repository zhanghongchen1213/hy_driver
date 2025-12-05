static void test_task(void *pvParameters);
static void motor_task(void *pvParameters);
static void uplink_task(void *pvParameters);
// 任务句柄
static TaskHandle_t g_uplink_task_handle = NULL;

// 上行数据包计数器
static uint32_t g_packet_counter = 0;
static uint32_t g_left_encoder_count = 0;
static uint32_t g_right_encoder_count = 0;

// 运动控制相关全局变量
static uart_downlink_packet_t g_latest_motion_cmd; // 最新运动控制指令
static uint32_t g_cmd_counter = 0;                 // 接收指令计数
static void uart_rx_callback(const uart_downlink_packet_t *packet, void *user_data)
{
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "接收到空数据包指针！");
        return;
    }

    // 更新接收指令计数
    g_cmd_counter++;

    // ========== 保存最新指令到全局变量 ==========
    // 直接复制结构体内容，无需手动解析
    memcpy(&g_latest_motion_cmd, packet, sizeof(uart_downlink_packet_t));
    // ========== 打印接收到的运动控制指令 ==========
    ESP_LOGI(TAG, "========== 收到上位机运动控制指令 [第%lu条] ==========", g_cmd_counter);
    ESP_LOGI(TAG, "线速度: %.3f m/s", g_latest_motion_cmd.linear_velocity);
    ESP_LOGI(TAG, "角速度: %.3f rad/s", g_latest_motion_cmd.angular_velocity);
    ESP_LOGI(TAG, "控制模式: %d", g_latest_motion_cmd.control_mode);
    ESP_LOGI(TAG, "使能标志: %s", g_latest_motion_cmd.enable_flag ? "启用" : "禁用");

    ESP_LOGI(TAG, "================================================");
}

static esp_err_t send_uplink_packet(void)
{
    // 模拟传感器数据更新
    g_left_encoder_count += 10;  // 模拟左轮编码器增量
    g_right_encoder_count += 12; // 模拟右轮编码器增量

    // 构造上行数据包
    uart_uplink_packet_t uplink_packet = {
        .start_flag = UART_PACKET_START_FLAG,
        .length = UART_UPLINK_PACKET_SIZE - 3, // 总长度减去起始、长度、结束标志
        .left_encoder = g_left_encoder_count,
        .right_encoder = g_right_encoder_count,
        .battery_voltage = 12000 - (g_packet_counter % 100), // 模拟电池电压变化
        .battery_percentage = 85 - (g_packet_counter % 20),  // 模拟电池电量变化
        .volume_level = 50 + (g_packet_counter % 10),        // 模拟音量变化
        .temperature = 250 + (g_packet_counter % 30),        // 模拟温度变化
        .end_flag = UART_PACKET_END_FLAG};

    // 发送数据包
    esp_err_t ret = uart_comm_send_uplink_packet(&uplink_packet);
    if (ret == ESP_OK)
    {
        g_packet_counter++;
    }
    else
    {
        ESP_LOGE(TAG, "上行数据包发送失败: %s", esp_err_to_name(ret));
    }

    return ret;
}

static void uplink_task(void *pvParameters)
{
    ESP_LOGI(TAG, "上行数据包定时发送任务启动 - 周期: 20ms");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(10000); // 1s周期

    while (1)
    {
        // 发送上行数据包
        send_uplink_packet();

        // 等待下一个周期
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void motor_task(void *pvParameters)
{
    while (1)
    {
        float rpm_a = 0;
        motor_get_rpm(MOTOR_A, &rpm_a);
        float rpm_b = 0;
        motor_get_rpm(MOTOR_B, &rpm_b);
        ESP_LOGI(TAG, "电机A RPM: %f, 电机B RPM: %f", rpm_a, rpm_b);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void test_task(void *pvParameters)
{
#if 1
    ESP_LOGI(TAG, "开始双电机控制演示...");
    esp_err_t ret;
    ESP_LOGI(TAG, "步骤1: 使能电机");
    ret = motor_enable();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机使能失败!");
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ESP_LOGI(TAG, "步骤2: 单独控制电机A");
    // ret = motor_set_speed(MOTOR_A, 80); // 70%速度
    // ret = motor_set_direction(MOTOR_A, MOTOR_DIRECTION_FORWARD);
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // ESP_LOGI(TAG, "步骤3: 单独控制电机B");
    // ret = motor_set_speed(MOTOR_B, 80); // 80%速度
    // ret = motor_set_direction(MOTOR_B, MOTOR_DIRECTION_FORWARD);
    // vTaskDelay(pdMS_TO_TICKS(2000));

    // ESP_LOGI(TAG, "步骤4: 双电机速度控制");
    // ret = motor_set_speed(MOTOR_BOTH, 20); // 20%速度
    // vTaskDelay(pdMS_TO_TICKS(3000));
    // ret = motor_set_speed(MOTOR_BOTH, 50); // 60%速度
    // vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "步骤4: 双电机方向控制");
    ret = motor_set_direction(MOTOR_BOTH, MOTOR_DIRECTION_FORWARD);
    ret = motor_set_speed(MOTOR_BOTH, 80); // 80%速度
    vTaskDelay(pdMS_TO_TICKS(5000));
    ret = motor_set_direction(MOTOR_BOTH, MOTOR_DIRECTION_REVERSE);
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "步骤5: 电机停止控制");
    ret = motor_stop(MOTOR_STOP_BRAKE, MOTOR_STOP_BRAKE);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "步骤6: 禁用电机");
    ret = motor_disable();
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 循环演示
    vTaskDelay(pdMS_TO_TICKS(10000));

    // 重新开始演示
    test_task(pvParameters);
#endif
}