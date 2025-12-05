/**
 * @file studio.c
 * @brief ESP32-S3音频编解码器驱动实现文件
 *
 * 本文件实现ESP32-S3平台上音频编解码器(ES8311)的驱动功能，
 * 包括设备初始化、音频参数配置、数据传输等核心功能。
 *
 * @author ZHC
 * @date 2025
 * @version 1.0
 *
 * @note 实现特性:
 *       - ES8311 DAC音频输出驱动
 *       - 动态音频参数配置
 *       - 音量和静音控制
 *
 * @warning 所有公共函数调用前需确保bsp_codec_init()已成功执行
 */

#include "studio.h"

/* ========================= 全局变量区 ========================= */

/** @defgroup GLOBAL_HANDLES 设备句柄全局变量
 * @{
 */
static const char *TAG = "STUDIO";
static esp_codec_dev_handle_t play_dev_handle = NULL;          /**< 音频播放设备句柄(ES8311) */
static i2s_chan_handle_t i2s_tx_chan = NULL;                   /**< I2S发送通道句柄 */
static const audio_codec_data_if_t *i2s_data_if_output = NULL; /**< DAC输出数据接口(ES8311) */
static file_iterator_instance_t *file_iterator = NULL;         /**< 文件迭代器实例 */
static audio_player_config_t player_config = {0};              /**< 音频播放器配置 */
static SemaphoreHandle_t flash_mutex = NULL;
uint8_t g_sys_volume = VOLUME_DEFAULT;
/** @} */

/* ========================= 内部函数声明区 ========================= */
static esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);
static esp_err_t bsp_audio_output_init(void); /**< ES8311 DAC输出接口初始化 */
static void pa_en_init(void);
static int compare_files_numeric(const void *a, const void *b);
static void sort_file_iterator_list(file_iterator_instance_t *iterator);
static esp_err_t _audio_player_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting);
static esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);
static esp_err_t _audio_player_std_clock(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);
static void _audio_player_callback(audio_player_cb_ctx_t *ctx);
/* ========================= 内部函数实现区 ========================= */
/**
 * @brief 音频静音控制函数
 * @param setting 静音设置参数，AUDIO_PLAYER_MUTE表示静音，AUDIO_PLAYER_UNMUTE表示取消静音
 * @return esp_err_t 返回操作结果，ESP_OK表示成功
 * @note 该函数用于控制音频编解码器的静音状态，优化后减少不必要的音量设置操作
 */
static esp_err_t _audio_player_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting)
{
    esp_err_t ret = ESP_OK;
    static bool volume_initialized = false;
    static uint8_t last_volume = 0;

    // 设置编解码器静音状态
    bsp_codec_mute_set(setting == AUDIO_PLAYER_MUTE ? true : false);

    // 只在第一次取消静音或音量发生变化时设置音量，避免频繁I2C操作
    if (setting == AUDIO_PLAYER_UNMUTE && (!volume_initialized || last_volume != g_sys_volume))
    {
        // 使用全局变量g_sys_volume设置音量
        bsp_codec_volume_set(g_sys_volume, NULL);
        last_volume = g_sys_volume;
        volume_initialized = true;
    }

    return ret;
}

/**
 * @brief 音频数据写入函数
 * @param audio_buffer 音频数据缓冲区指针
 * @param len 要写入的数据长度
 * @param bytes_written 实际写入的字节数
 * @param timeout_ms 写入超时时间（毫秒）
 * @return esp_err_t 返回操作结果，ESP_OK表示成功
 * @note 该函数用于将音频数据通过I2S接口写入音频设备，在播放过程中会不断被调用
 * 播放一帧，进入一次，在播放的时会不断进入
 */
static esp_err_t _audio_player_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;

    // 调用I2S接口写入音频数据
    ret = bsp_i2s_write(audio_buffer, len, bytes_written, timeout_ms);

    return ret;
}

/**
 * @brief 设置音频采样率函数
 * @param rate 采样率值，单位Hz
 * @param bits_cfg 位宽配置，表示音频数据的位宽
 * @param ch 声道模式，表示I2S的声道配置
 * @return esp_err_t 返回操作结果，ESP_OK表示成功
 * @note 该函数在音频播放时调用一次，用于配置音频编解码器的采样率、位宽和声道模式，据 mp3 文件信息，设置采样率，位宽，通道数。每换一首歌曲，都会调用一次
 */
static esp_err_t _audio_player_std_clock(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    // 调用编解码器接口设置采样率、位宽和声道模式
    ret = bsp_codec_set_output_fs(rate, bits_cfg, ch);

    return ret;
}

/**
 * @brief 音频播放器回调函数
 * @param ctx 回调上下文，包含音频事件信息
 * @note 该函数在播放器每次状态变化时被调用，用于处理不同播放事件
 */
static void _audio_player_callback(audio_player_cb_ctx_t *ctx)
{
    // 打印当前音频事件
    ESP_LOGI(TAG, "ctx->audio_event = %d", ctx->audio_event);

    // 根据音频事件类型进行不同处理
    switch (ctx->audio_event)
    {
    case AUDIO_PLAYER_CALLBACK_EVENT_IDLE: // 播放完成事件
    {
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_IDLE");
        break;
    }
    case AUDIO_PLAYER_CALLBACK_EVENT_PLAYING: // 开始播放事件
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PLAY");
        gpio_set_level(PA_EN_GPIO_PIN, 1); // 开启音频功放
        break;
    case AUDIO_PLAYER_CALLBACK_EVENT_PAUSE: // 暂停播放事件
        ESP_LOGI(TAG, "AUDIO_PLAYER_REQUEST_PAUSE");
        gpio_set_level(PA_EN_GPIO_PIN, 0); // 关闭音频功放
        break;
    default: // 其他未处理事件
        break;
    }
}

static int compare_files_numeric(const void *a, const void *b)
{
    const char *file1 = *(const char **)a;
    const char *file2 = *(const char **)b;
    int num1 = atoi(file1);
    int num2 = atoi(file2);
    return num1 - num2;
}

// 自定义文件排序函数
static void sort_file_iterator_list(file_iterator_instance_t *iterator)
{
    if (iterator == NULL || iterator->list == NULL || iterator->count == 0)
    {
        return;
    }

    // 使用qsort对文件列表进行排序
    qsort(iterator->list, iterator->count, sizeof(char *), compare_files_numeric);
}
static void pa_en_init(void)
{
    gpio_config_t gpio_init_struct = {0};

    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;         /* 失能引脚中断 */
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT;         /* 输入输出模式 */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;       /* 使能上拉 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;  /* 失能下拉 */
    gpio_init_struct.pin_bit_mask = 1ull << PA_EN_GPIO_PIN; /* 设置的引脚的位掩码 */
    gpio_config(&gpio_init_struct);                         /* 配置GPIO */
}

/**
 * @brief 初始化ES8311 DAC音频输出接口
 *
 * 配置I2S输出通道，用于ES8311音频数据播放。
 * 使用独立的I2S外设和GPIO引脚组。
 *
 * @return
 *     - ESP_OK: 初始化成功
 *     - ESP_FAIL: 初始化失败
 *
 * @note 该函数专门为ES8311 DAC设计，使用输出引脚组
 */
static esp_err_t bsp_audio_output_init(void)
{
    esp_err_t ret = ESP_FAIL;

    /* 检查输出数据接口是否已初始化 */
    if (i2s_tx_chan && i2s_data_if_output)
    {
        /* 输出接口已初始化 */
        return ESP_OK;
    }

    /* 配置I2S输出通道 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM_OUT, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; /* 自动清除DMA缓冲区中的遗留数据 */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL));

    /* 配置I2S输出标准模式参数 */
    const i2s_std_config_t std_cfg_output = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),                              /* 采样率16kHz */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(32, I2S_SLOT_MODE_STEREO), /* 32位立体声 */
        .gpio_cfg = {
            .mclk = GPIO_I2S_MCLK_OUT, /* 主时钟 */
            .bclk = GPIO_I2S_SCLK_OUT, /* 位时钟 */
            .ws = GPIO_I2S_LRCK_OUT,   /* 字选择时钟 */
            .dout = GPIO_I2S_DOUT,     /* 数据输出引脚 */
            .din = GPIO_NUM_NC,        /* 输出通道不需要输入引脚 */
        },
    };

    /* 初始化I2S发送通道 */
    if (i2s_tx_chan != NULL)
    {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg_output), err, "I2S", "I2S输出通道初始化失败");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, "I2S", "I2S输出通道使能失败");
    }

    /* 创建I2S输出数据接口 */
    audio_codec_i2s_cfg_t i2s_cfg_output = {
        .port = BSP_I2S_NUM_OUT,
        .rx_handle = NULL, /* 输出接口不需要接收通道 */
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if_output = audio_codec_new_i2s_data(&i2s_cfg_output);
    if (i2s_data_if_output == NULL)
    {
        goto err;
    }

    return ESP_OK;

err:
    /* 错误处理：清理已分配的资源 */
    if (i2s_tx_chan)
    {
        i2s_del_channel(i2s_tx_chan);
        i2s_tx_chan = NULL;
    }
    return ret;
}

static esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    /* 检查I2S输出数据接口是否已初始化，如未初始化则调用音频输出初始化函数 */
    if (i2s_data_if_output == NULL)
    {
        /* 配置ES8311 DAC的I2S输出接口 */
        ESP_ERROR_CHECK(bsp_audio_output_init());
    }
    assert(i2s_data_if_output); /* 确保I2S输出数据接口有效 */

    /* 创建GPIO接口，用于控制编解码器的GPIO引脚 */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    /* 配置I2C控制接口参数 */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,               /* I2C端口号 */
        .addr = ES8311_CODEC_DEFAULT_ADDR, /* ES8311芯片I2C地址 */
    };
    /* 创建I2C控制接口 */
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    assert(i2c_ctrl_if); /* 确保I2C控制接口创建成功 */

    /* 配置硬件增益参数 */
    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,        /* 功放电压5.0V */
        .codec_dac_voltage = 3.3, /* 编解码器DAC电压3.3V */
    };

    /* 配置ES8311编解码器参数 */
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,                    /* I2C控制接口 */
        .gpio_if = gpio_if,                        /* GPIO接口 */
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC, /* 工作模式：DAC输出 */
        .pa_pin = GPIO_PWR_CTRL,                   /* 功放控制引脚 */
        .pa_reverted = false,                      /* 功放控制逻辑不反转 */
        .master_mode = false,                      /* 从模式(ESP32为主) */
        .use_mclk = true,                          /* 使用主时钟MCLK */
        .digital_mic = false,                      /* 不使用数字麦克风 */
        .invert_mclk = false,                      /* MCLK不反转 */
        .invert_sclk = false,                      /* SCLK不反转 */
        .hw_gain = gain,                           /* 硬件增益配置 */
    };
    /* 创建ES8311编解码器设备 */
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    assert(es8311_dev); /* 确保ES8311设备创建成功 */

    /* 配置编解码器设备参数 */
    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT, /* 设备类型：输出设备 */
        .codec_if = es8311_dev,             /* 编解码器接口 */
        .data_if = i2s_data_if_output,      /* I2S输出数据接口 */
    };
    /* 创建并返回编解码器设备句柄 */
    return esp_codec_dev_new(&codec_dev_cfg);
}

/* ========================= 公共函数实现区 ========================= */

esp_err_t bsp_codec_init(void)
{
    /* 初始化音频播放设备(ES8311) */
    play_dev_handle = bsp_audio_codec_speaker_init();
    assert((play_dev_handle) && "音频播放设备初始化失败");

    /* 设置默认音频参数：16kHz采样率，32位位宽，立体声 */
    bsp_codec_set_fs(CODEC_DEFAULT_SAMPLE_RATE, CODEC_DEFAULT_BIT_WIDTH, CODEC_DEFAULT_CHANNEL);

    /* 初始化功放使能引脚 */
    pa_en_init();

    return ESP_OK;
}

esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    esp_err_t ret = ESP_OK;

    /* 参数有效性检查 */
    if (audio_buffer == NULL || bytes_written == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* 检查播放设备是否已初始化 */
    if (play_dev_handle == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* 将音频数据写入播放设备 */
    ret = esp_codec_dev_write(play_dev_handle, audio_buffer, len);

    /* 设置实际写入的字节数 */
    *bytes_written = (ret == ESP_OK) ? len : 0;

    return ret;
}

esp_err_t bsp_codec_set_output_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    /* 配置音频采样参数结构体 */
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = rate,         /* 采样率 */
        .channel = ch,               /* 通道模式 */
        .bits_per_sample = bits_cfg, /* 位宽 */
    };

    /* 关闭当前播放设备以重新配置参数 */
    if (play_dev_handle)
    {
        ret = esp_codec_dev_close(play_dev_handle);
        /* 使用新参数重新打开播放设备 */
        ret |= esp_codec_dev_open(play_dev_handle, &fs);
    }
    else
    {
        ret = ESP_ERR_INVALID_STATE; /* 播放设备未初始化 */
    }

    return ret;
}

esp_err_t bsp_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    /* 配置输出设备 */
    ret = bsp_codec_set_output_fs(rate, bits_cfg, ch);

    return ret;
}

// 设置喇叭音量
esp_err_t bsp_codec_volume_set(int volume, int *volume_set)
{
    esp_err_t ret = ESP_OK;
    float v = volume;
    ret = esp_codec_dev_set_out_vol(play_dev_handle, (int)v);
    return ret;
}

esp_err_t bsp_codec_mute_set(bool enable)
{
    esp_err_t ret = ESP_OK;
    ret = esp_codec_dev_set_out_mute(play_dev_handle, enable);
    return ret;
}

void mp3_player_init(void)
{
    // 初始化文件迭代器，用于遍历指定挂载点下的音乐文件
    file_iterator = file_iterator_new("/spiffs");
    assert(file_iterator != NULL); // 确保文件迭代器初始化成功

    // 对文件列表进行数字排序
    sort_file_iterator_list(file_iterator);

    // 配置音频播放器参数
    player_config.mute_fn = _audio_player_mute_fn;      // 设置静音回调函数
    player_config.write_fn = _audio_player_write_fn;    // 设置音频数据写入回调函数
    player_config.clk_set_fn = _audio_player_std_clock; // 设置采样率配置回调函数
    player_config.priority = 6;                         // 设置任务优先级
    player_config.coreID = 1;                           // 设置运行核心

    // 创建音频播放器实例，并检查是否成功
    ESP_ERROR_CHECK(audio_player_new(player_config));

    // 注册音频播放状态回调函数，用于处理播放事件
    ESP_ERROR_CHECK(audio_player_callback_register(_audio_player_callback, NULL));

    // 初始化互斥锁
    if (flash_mutex == NULL)
        flash_mutex = xSemaphoreCreateMutex();
}

void safe_play_index(int index)
{
    // 记录日志，显示当前播放的索引号
    ESP_LOGI(TAG, "safe_play_index(%d)", index);

    // 定义文件路径缓冲区和文件指针
    char filename[128];
    FILE *fp = NULL;

    // 尝试获取互斥锁进行文件操作，等待时间500毫秒
    if (xSemaphoreTake(flash_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
    {
        // 通过索引号获取完整文件路径
        int retval = file_iterator_get_full_path_from_index(file_iterator, index, filename, sizeof(filename));
        if (retval != 0)
        {
            // 以二进制只读模式打开文件
            fp = fopen(filename, "rb");
            if (fp)
            {
                ESP_LOGI(TAG, "Playing '%s'", filename);
            }
            else
            {
                ESP_LOGE(TAG, "unable to open index %d, filename '%s'", index, filename);
            }
        }
        else
        {
            ESP_LOGE(TAG, "unable to retrieve filename for index %d", index);
        }

        // 立即释放互斥锁，不在播放过程中持有
        xSemaphoreGive(flash_mutex);

        // 如果文件打开成功，开始播放（此时已释放锁）
        if (fp)
        {
            audio_player_play(fp);
        }
    }
    else
    {
        // 获取互斥锁失败，记录错误日志
        ESP_LOGE(TAG, "Failed to get flash mutex for playback");
    }
}
