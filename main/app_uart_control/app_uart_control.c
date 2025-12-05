#include "app_uart_control.h"

static const char *TAG = "APP_UART";
TaskHandle_t CI03T_UART_MONITOR_task_handle = NULL;
static void CI03T_UART_MONITOR_task(void *pvParameters);
static QueueHandle_t s_ci03t_uart_queue = NULL;
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
                        switch ((CI_03T_CMD)byte)
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
