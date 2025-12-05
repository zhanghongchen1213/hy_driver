/**
 * @file studio.h
 * @brief ESP32-S3音频编解码器驱动头文件
 *
 * 本文件提供ESP32-S3平台上音频编解码器(ES8311)的驱动接口，
 * 支持音频播放功能，包括I2S接口配置、音频参数设置等。
 *
 * @author ZHC
 * @date 2025
 * @version 1.0
 *
 * @note 硬件配置:
 *       - 音频输出芯片: ES8311 (DAC)
 *       - I2S接口: 支持独立的输出接口配置
 *         * 输出接口: I2S_NUM_1 (ES8311 DAC)
 *       - I2C接口: 用于芯片寄存器配置
 *
 * @warning 使用前必须先调用bsp_codec_init()进行初始化
 */

#ifndef __STUDIO_H__
#define __STUDIO_H__

/* ========================= 系统头文件包含区 ========================= */
#include <string.h>
#include "math.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

#include "audio_player.h"
#include "file_iterator.h"
#include <dirent.h>
#include "studio.h"

/* ========================= 宏定义区 ========================= */

/** @defgroup I2C_CONFIG I2C总线配置
 * @{
 */
#define BSP_I2C_NUM (0) /**< I2C外设编号 */
/** @} */

/** @defgroup AUDIO_VOLUME 音频音量配置
 * @{
 */
#define VOLUME_DEFAULT 100 /**< 默认音量值 */
/** @} */

/** @defgroup CODEC_DEFAULT 编解码器默认参数
 * @{
 */
#define CODEC_DEFAULT_SAMPLE_RATE (16000) /**< 默认音频采样率(Hz) */
#define CODEC_DEFAULT_BIT_WIDTH (32)      /**< 默认音频位宽(bit) */
#define CODEC_DEFAULT_CHANNEL (2)         /**< 默认音频通道数(立体声) */
/** @} */

/** @defgroup I2S_CONFIG I2S接口配置
 * @{
 */
#define BSP_I2S_NUM_OUT I2S_NUM_1 /**< I2S输出外设编号 */
/** @} */

/** @defgroup GPIO_I2S_OUTPUT I2S输出引脚定义
 * @{
 */
#define GPIO_I2S_LRCK_OUT (GPIO_NUM_15) /**< I2S左右声道时钟(输出) */
#define GPIO_I2S_MCLK_OUT (GPIO_NUM_16) /**< I2S主时钟(输出) */
#define GPIO_I2S_SCLK_OUT (GPIO_NUM_7)  /**< I2S串行时钟(输出) */
#define GPIO_I2S_DOUT (GPIO_NUM_6)      /**< I2S串行数据输出 */
/** @} */

/** @defgroup GPIO_POWER 电源控制引脚
 * @{
 */
#define GPIO_PWR_CTRL (GPIO_NUM_NC) /**< 功放电源控制引脚(未连接) */
#define PA_EN_GPIO_PIN GPIO_NUM_5
/** @} */

/* ========================= 函数声明区 ========================= */

/** @defgroup CODEC_INIT 编解码器初始化函数
 * @{
 */

/**
 * @brief 初始化音频编解码器
 *
 * 初始化ES8311(DAC)音频编解码器，配置I2S接口和默认音频参数。
 * 该函数会自动配置音频输出设备，
 * 设置默认采样率、位宽和通道数。
 *
 * @return
 *     - ESP_OK: 初始化成功
 *     - ESP_FAIL: 初始化失败
 *
 * @note 使用音频功能前必须先调用此函数
 * @warning 确保I2C和I2S硬件连接正确
 */
esp_err_t bsp_codec_init(void);

/** @} */

/** @defgroup AUDIO_IO 音频输入输出函数
 * @{
 */

/**
 * @brief 写入音频数据到播放设备
 *
 * 将音频数据写入到ES8311 DAC进行播放输出。
 *
 * @param[in] audio_buffer 音频数据缓冲区指针
 * @param[in] len 要写入的数据长度(字节)
 * @param[out] bytes_written 实际写入的字节数
 * @param[in] timeout_ms 超时时间(毫秒，当前未使用)
 *
 * @return
 *     - ESP_OK: 写入成功
 *     - ESP_FAIL: 写入失败
 *
 * @note 音频数据格式需与当前设置的采样率、位宽、通道数匹配
 */
esp_err_t bsp_i2s_write(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms);

/** @} */

/** @defgroup AUDIO_CONFIG 音频参数配置函数
 * @{
 */

/**
 * @brief 专门配置ES8311 DAC音频输出设备的采样参数
 *
 * 此函数仅配置音频输出设备(ES8311 DAC)的采样率、位宽和通道数，
 * 适用于播放不同音频文件时需要动态调整参数的场景。
 *
 * @param[in] rate 采样率 (Hz)
 * @param[in] bits_cfg 位宽配置
 * @param[in] ch 通道模式
 *
 * @return
 *     - ESP_OK: 配置成功
 *     - ESP_ERR_INVALID_STATE: 播放设备未初始化
 *     - ESP_FAIL: 配置失败
 *
 * @note 仅影响音频输出设备，不会改变输入设备的配置
 */
esp_err_t bsp_codec_set_output_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

/**
 * @brief 动态设置音频编解码器的采样率、位宽和通道数(兼容性接口)
 *
 * 此函数为保持向后兼容性而保留，内部调用bsp_codec_set_output_fs函数配置输出设备。
 * 建议新代码使用专门的输出配置函数以获得更好的控制。
 *
 * @param[in] rate 采样率(Hz)，如8000, 16000, 44100, 48000等
 * @param[in] bits_cfg 音频位宽(bit)，如16, 24, 32
 * @param[in] ch 通道模式，如I2S_SLOT_MODE_MONO(单声道)或I2S_SLOT_MODE_STEREO(立体声)
 *
 * @return
 *     - ESP_OK: 设置成功
 *     - ESP_FAIL: 设置失败
 *
 * @note 同时配置输入和输出设备，建议使用专门的函数进行独立配置
 * @warning 确保参数组合被硬件支持
 */
esp_err_t bsp_codec_set_fs(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch);

/**
 * @brief 设置音频输出音量
 *
 * 调节音频输出的音量大小。
 *
 * @param[in] volume 目标音量值(0-100)
 * @param[out] volume_set 实际设置的音量值
 *
 * @return
 *     - ESP_OK: 设置成功
 *     - ESP_FAIL: 设置失败
 *
 * @note 音量范围0-100，对应编解码器的实际增益范围
 */
esp_err_t bsp_codec_volume_set(int volume, int *volume_set);

/**
 * @brief 设置音频输出静音
 *
 * 启用或禁用音频输出的静音功能。
 *
 * @param[in] enable true: 静音, false: 取消静音
 *
 * @return
 *     - ESP_OK: 设置成功
 *     - ESP_FAIL: 设置失败
 *
 * @note 调用此函数后，音频输出将停止播放
 */
esp_err_t bsp_codec_mute_set(bool enable);

/**
 * @brief 初始化MP3播放器
 *
 * 初始化MP3播放器，配置I2S接口和默认音频参数。
 * 该函数会自动配置音频输出设备，
 * 设置默认采样率、位宽和通道数。
 *
 * @return
 *     - ESP_OK: 初始化成功
 *     - ESP_FAIL: 初始化失败
 *
 * @note 使用音频功能前必须先调用此函数
 * @warning 确保I2C和I2S硬件连接正确
 */
void mp3_player_init(void);

/**
 * @brief 安全播放MP3文件索引
 *
 * 安全播放指定索引的MP3文件，确保在播放过程中不会中断。
 *
 * @param[in] index MP3文件索引(从0开始)
 *
 * @note 调用前确保MP3播放器已初始化
 */
void safe_play_index(int index);
/** @} */
#endif /* __STUDIO_H__ */