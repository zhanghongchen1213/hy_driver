#include "app_uart_control.h"
#include "app_pid_control.h"

static const char *TAG = "APP_UART";

static uint16_t s_chat_gpt_count = 0;
uint8_t s_audio_stream_flag = 0;

static QueueHandle_t s_ci03t_uart_queue = NULL;
static QueueHandle_t s_brain_uart_queue = NULL;
TaskHandle_t CI03T_UART_MONITOR_task_handle = NULL;
TaskHandle_t BRAIN_UART_RX_task_handle = NULL;
TaskHandle_t BRAIN_UART_TX_task_handle = NULL;
static void CI03T_UART_MONITOR_task(void *pvParameters);
static void BRAIN_UART_RX_task(void *pvParameters);
static void BRAIN_UART_TX_task(void *pvParameters);
static void brain_parse_downlink(const uint8_t *buf, int len);
static void brain_execute_command(const uart_downlink_packet_t *pkt);
static void brain_send_uplink(void);

void rtos_uart_init(void)
{
    BaseType_t result = pdPASS;
    s_ci03t_uart_queue = uart_get_event_queue(CI03T_UART_NUM);
    if (s_ci03t_uart_queue == NULL)
    {
        ESP_LOGE(TAG, "CI03T事件队列获取失败");
        return;
    }
    result = xTaskCreatePinnedToCore(CI03T_UART_MONITOR_task, "CI03T_UART_MONITOR_task", 3 * 1024, NULL, 3, &CI03T_UART_MONITOR_task_handle, 1);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "CI03T_UART_MONITOR_task任务创建失败");
    }
    s_brain_uart_queue = uart_get_event_queue(BRAIN_UART_NUM);
    if (s_brain_uart_queue == NULL)
    {
        ESP_LOGE(TAG, "BRAIN事件队列获取失败");
    }
    else
    {
        result = xTaskCreatePinnedToCore(BRAIN_UART_RX_task, "BRAIN_UART_RX_task", 4 * 1024, NULL, 15, &BRAIN_UART_RX_task_handle, 1);
        if (result != pdPASS)
        {
            ESP_LOGE(TAG, "BRAIN_UART_RX_task任务创建失败");
        }
    }
    result = xTaskCreatePinnedToCore(BRAIN_UART_TX_task, "BRAIN_UART_TX_task", 3 * 1024, NULL, 12, &BRAIN_UART_TX_task_handle, 1);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "BRAIN_UART_TX_task任务创建失败");
    }
}

static void CI03T_UART_MONITOR_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t byte = 0;
    for (;;)
    {
        if (xQueueReceive(s_ci03t_uart_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                int rx = event.size;
                while (rx > 0)
                {
                    int n = uart_read_bytes(CI03T_UART_NUM, &byte, 1, 0);
                    if (n == 1)
                    {
                        CI_03T_CMD cmd = (CI_03T_CMD)byte;
                        switch (cmd)
                        {
                        case WAKE_UP:
                            ESP_LOGI(TAG, "收到指令: 唤醒 (0x%02X)", byte);
                            break;
                        case CHAT_GPT:
                            // 保证在播报过程中，音频流不会被打断
                            if (s_audio_stream_flag == 0 || s_audio_stream_flag == 4)
                            {
                                s_chat_gpt_count++;
                                ESP_LOGI(TAG, "收到指令: 交互 (0x%02X), 累计次数: %u", byte, s_chat_gpt_count);
                            }
                            else
                            {
                                ESP_LOGI(TAG, "收到指令: 交互 (0x%02X), 但音频流标志为 %d, 忽略计数", byte, s_audio_stream_flag);
                            }
                            break;
                        case WAKE_EXIT:
                            ESP_LOGI(TAG, "收到指令: 退出唤醒 (0x%02X)", byte);
                            break;
                        default:
                            ESP_LOGW(TAG, "未知指令: 0x%02X", byte);
                            break;
                        }
                    }
                    rx--;
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(CI03T_UART_NUM);
                xQueueReset(s_ci03t_uart_queue);
                break;
            default:
                break;
            }
        }
    }
}

static void BRAIN_UART_RX_task(void *pvParameters)
{
    uart_event_t event;
    for (;;)
    {
        if (xQueueReceive(s_brain_uart_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                int remain = event.size;
                uint8_t buf[256];
                while (remain > 0)
                {
                    int chunk = remain < (int)sizeof(buf) ? remain : (int)sizeof(buf);
                    int r = uart_read_bytes(BRAIN_UART_NUM, buf, chunk, 0);
                    if (r > 0)
                    {
                        brain_parse_downlink(buf, r);
                        remain -= r;
                    }
                    else
                    {
                        break;
                    }
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                uart_flush_input(BRAIN_UART_NUM);
                xQueueReset(s_brain_uart_queue);
                break;
            default:
                break;
            }
        }
    }
}

static void BRAIN_UART_TX_task(void *pvParameters)
{
    for (;;)
    {
        brain_send_uplink();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief 解析 BRAIN 模块下行的串口数据包
 *
 * 本函数从接收缓冲区中逐字节扫描，查找合法的数据包：
 * 1. 以 RECEIVE_PACKET_START_FLAG 开头；
 * 2. 剩余长度足够（sizeof(uart_downlink_packet_t)）；
 * 3. 包尾字节为 RECEIVE_PACKET_END_FLAG；
 * 找到合法包后，提取字段并调用 brain_execute_command() 执行对应指令。
 *
 * @param buf 串口接收到的原始数据缓冲区
 * @param len 缓冲区有效长度
 */
static void brain_parse_downlink(const uint8_t *buf, int len)
{
    /* 逐字节扫描缓冲区 */
    for (int i = 0; i < len; i++)
    {
        /* 检测到包头标志 */
        if (buf[i] == RECEIVE_PACKET_START_FLAG)
        {
            /* 检查剩余长度是否足够一个完整包 */
            if (len - i >= sizeof(uart_downlink_packet_t))
            {
                /* 检查包尾标志是否匹配 */
                if (buf[i + sizeof(uart_downlink_packet_t) - 1] == RECEIVE_PACKET_END_FLAG)
                {
                    uart_downlink_packet_t *pkt = (uart_downlink_packet_t *)(buf + i);

                    /* 提取 audio_stream_flag (位于包头后第 1 字节) */
                    // 由于结构体是 packed 的，可以直接访问

                    /* 执行对应指令 */
                    brain_execute_command(pkt);

                    /* 跳过本包长度，继续扫描后续数据 */
                    i += sizeof(uart_downlink_packet_t) - 1;
                }
            }
        }
    }
}

static void brain_execute_command(const uart_downlink_packet_t *pkt)
{
    s_audio_stream_flag = pkt->audio_stream_flag;
    ESP_LOGI(TAG, "收到控制指令 时间戳=%u, 音频流标志=%u", (unsigned)pkt->timestamp, (unsigned)pkt->audio_stream_flag);

    // 更新PID目标速度
    float left_target_speed = pkt->left_target_speed;
    float right_target_speed = pkt->right_target_speed;
    app_pid_set_speed(MOTOR_A, pkt->left_target_speed);
    app_pid_set_speed(MOTOR_B, pkt->right_target_speed);

    // 更新PID参数
    app_pid_set_params(MOTOR_A, pkt->left_kp, pkt->left_ki, pkt->left_kd);
    app_pid_set_params(MOTOR_B, pkt->right_kp, pkt->right_ki, pkt->right_kd);

    ESP_LOGI(TAG, "更新电机控制: 左目标=%.2f, 右目标=%.2f, 左PID[%.2f,%.2f,%.2f], 右PID[%.2f,%.2f,%.2f]",
             pkt->left_target_speed, pkt->right_target_speed,
             pkt->left_kp, pkt->left_ki, pkt->left_kd,
             pkt->right_kp, pkt->right_ki, pkt->right_kd);
}

/**
 * @brief 发送BRAIN上行数据包
 */
static void brain_send_uplink(void)
{
    uart_uplink_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt)); // 清空数据包

    pkt.start_flag = SEND_PACKET_START_FLAG;
    pkt.chat_gpt_count = s_chat_gpt_count;

    // 获取电机PID状态
    const pid_ctrl_block_t *pid_a = app_pid_get_status(MOTOR_A);
    const pid_ctrl_block_t *pid_b = app_pid_get_status(MOTOR_B);

    if (pid_a != NULL)
    {
        pkt.left_target_speed = pid_a->state.target_speed;
        pkt.left_actual_speed = pid_a->state.actual_speed;
        pkt.left_kp = pid_a->params.kp;
        pkt.left_ki = pid_a->params.ki;
        pkt.left_kd = pid_a->params.kd;
    }

    if (pid_b != NULL)
    {
        pkt.right_target_speed = pid_b->state.target_speed;
        pkt.right_actual_speed = pid_b->state.actual_speed;
        pkt.right_kp = pid_b->params.kp;
        pkt.right_ki = pid_b->params.ki;
        pkt.right_kd = pid_b->params.kd;
    }

    pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000ULL);
    pkt.end_flag = SEND_PACKET_END_FLAG;
    uart_write_bytes(BRAIN_UART_NUM, (const char *)&pkt, sizeof(pkt));
}

/**
 * @brief 设置 ChatGPT 启用状态
 * 供外部模块（如音频播放）调用
 * @param enable true=启用, false=禁用
 */
void app_uart_set_chat_gpt_enable(bool enable)
{
    // chat_gpt_set(enable);
    // ESP_LOGI(TAG, "外部设置 chat_gpt_enable = %d", enable);
}
