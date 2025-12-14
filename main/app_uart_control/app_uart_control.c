#include "app_uart_control.h"

static const char *TAG = "APP_UART";

static bool s_chat_gpt_enable = false;
static uint32_t s_chat_gpt_expire_ms = 0;

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
static void chat_gpt_timeout_check(void);
static inline void chat_gpt_set(bool enable)
{
    if (enable)
    {
        s_chat_gpt_enable = true;
        s_chat_gpt_expire_ms = (uint32_t)(esp_timer_get_time() / 1000ULL) + 10000U;
    }
    else
    {
        s_chat_gpt_enable = false;
        s_chat_gpt_expire_ms = 0;
    }
}
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
                            ESP_LOGI(TAG, "收到指令: 交互 (0x%02X)", byte);
                            break;
                        case WAKE_EXIT:
                            ESP_LOGI(TAG, "收到指令: 退出唤醒 (0x%02X)", byte);
                            break;
                        default:
                            ESP_LOGW(TAG, "未知指令: 0x%02X", byte);
                            break;
                        }
                        chat_gpt_set(cmd == CHAT_GPT);
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
        chat_gpt_timeout_check();
        brain_send_uplink();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void chat_gpt_timeout_check(void)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (s_chat_gpt_enable && s_chat_gpt_expire_ms != 0 && now_ms >= s_chat_gpt_expire_ms)
    {
        s_chat_gpt_enable = false;
        s_chat_gpt_expire_ms = 0;
        ESP_LOGI(TAG, "交互超时, 自动关闭chat_gpt_enable");
    }
}

/**
 * @brief 解析 BRAIN 模块下行的串口数据包
 *
 * 本函数从接收缓冲区中逐字节扫描，查找合法的数据包：
 * 1. 以 RECEIVE_PACKET_START_FLAG 开头；
 * 2. 第 2 字节为总长度 L，且 L ≥ 7（最小合法长度）；
 * 3. 包尾字节为 RECEIVE_PACKET_END_FLAG；
 * 4. 长度足够，避免越界。
 * 找到合法包后，提取时间戳并调用 brain_execute_command() 执行对应指令。
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
            uint8_t L = buf[i + 1]; // 第 2 字节为数据包总长度
            /* 长度字段必须 ≥ 7，否则视为非法包 */
            if (L >= 7)
            {
                int remain = len - i; // 剩余未处理字节数

                /* 缓冲区剩余字节数 ≥ 声明的总长度 L，才继续解析 */
                if (remain >= L)
                {
                    /* 检查包尾标志是否匹配 */
                    if (buf[i + L - 1] == RECEIVE_PACKET_END_FLAG)
                    {
                        uart_downlink_packet_t pkt;

                        /* 填充包头、长度、包尾 */
                        pkt.start_flag = buf[i];
                        pkt.length = L;
                        pkt.end_flag = buf[i + L - 1];

                        /* 提取 4 字节时间戳（位于包头后第 2~5 字节） */
                        uint32_t ts = 0;
                        memcpy(&ts, buf + i + 2, sizeof(uint32_t));
                        pkt.timestamp = ts;

                        /* 执行对应指令 */
                        brain_execute_command(&pkt);

                        /* 跳过本包剩余字节，直接到包尾后一位 */
                        i += L - 1;
                    }
                }
            }
        }
    }
}

static void brain_execute_command(const uart_downlink_packet_t *pkt)
{
    ESP_LOGI(TAG, "收到控制指令 时间戳=%u", (unsigned)pkt->timestamp);
}

/**
 * @brief 发送BRAIN上行数据包
 */
static void brain_send_uplink(void)
{
    uart_uplink_packet_t pkt;
    pkt.start_flag = SEND_PACKET_START_FLAG;
    pkt.length = sizeof(uart_uplink_packet_t);
    pkt.chat_gpt_enable = s_chat_gpt_enable;
    pkt.timestamp = (uint32_t)(esp_timer_get_time() / 1000ULL);
    pkt.end_flag = SEND_PACKET_END_FLAG;
    uart_write_bytes(BRAIN_UART_NUM, (const char *)&pkt, sizeof(pkt));
}
