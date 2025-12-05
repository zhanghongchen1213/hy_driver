#include "app_audio.h"

static const char *TAG = "APP_AUDIO";
TaskHandle_t audio_player_task_handle;

static void audio_player_task(void *pvParameters);

QueueHandle_t audio_queue = NULL; // 音频队列句
void rtos_audio_control_init(void)
{
    audio_queue = xQueueCreate(5, sizeof(uint8_t));
    BaseType_t result = pdPASS;
    result = xTaskCreatePinnedToCore(audio_player_task, "audio_player_task", 3 * 1024, NULL, 3, &audio_player_task_handle, 1);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "语音播报任务创建失败");
    }
}

static void audio_player_task(void *pvParameters)
{
    while (1)
    {
        uint8_t audio_data;
        uint8_t latest_audio_data = 0;
        bool has_audio = false;
        // 循环读取队列中的所有音频数据
        while (xQueueReceive(audio_queue, &audio_data, 0) == pdPASS)
        {
            latest_audio_data = audio_data;
            has_audio = true;
        }
        // 如果没有立即可用的音频数据，等待新数据
        if (!has_audio)
        {
            if (xQueueReceive(audio_queue, &latest_audio_data, pdMS_TO_TICKS(1000)) == pdPASS)
            {
                has_audio = true;
            }
        }
        // 播放最新的音频数据
        if (has_audio)
        {
            safe_play_index(latest_audio_data);
        }
    }
}