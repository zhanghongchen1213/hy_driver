/**
 * @file motor.c
 * @brief TB6612FNG双电机控制实现 - 基于GPIO和MCPWM直接控制
 * @author ZHC
 * @date 2025
 * @version 1.0
 */

#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor.h"

static const char *TAG = "motor";

// 全局双电机控制结构体
static dual_motor_t g_dual_motor = {
    .tb6612fng = {
        .ain1_pin = AIN1,
        .ain2_pin = AIN2,
        .pwma_pin = PWMA,
        .bin1_pin = BIN1,
        .bin2_pin = BIN2,
        .pwmb_pin = PWMB,
        .timer_handle = NULL,
        .operator_handle = NULL,
        .cmpr_a_handle = NULL,
        .cmpr_b_handle = NULL,
        .gen_a_handle = NULL,
        .gen_b_handle = NULL,
        .is_initialized = false,
        .motor_a_enabled = false,
        .motor_b_enabled = false},
    .encoder_a = {.pcnt_unit = NULL, .pulse_count = 0, .speed_rpm = 0.0f, .mode = ENCODER_MODE_X4, .is_initialized = false},
    .encoder_b = {.pcnt_unit = NULL, .pulse_count = 0, .speed_rpm = 0.0f, .mode = ENCODER_MODE_X4, .is_initialized = false},
    .is_initialized = false};

static esp_err_t tb6612fng_init(tb6612fng_motor_t *motor);
static bool IRAM_ATTR encoder_overflow_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
static esp_err_t encoder_get_count(encoder_t *encoder, int32_t *count);
static esp_err_t encoder_clear_count(encoder_t *encoder);
static esp_err_t encoder_init_single(encoder_t *encoder, int phase_a_pin, int phase_b_pin, int unit_id);
static esp_err_t encode_init(void);
//*=========== 内部函数 ==============*//
static bool IRAM_ATTR encoder_overflow_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    encoder_t *encoder = (encoder_t *)user_ctx;
    if (encoder != NULL)
    {
        // 处理溢出事件
        if (edata->watch_point_value == PCNT_UNIT_MAX_COUNT)
        {
            encoder->pulse_count += PCNT_UNIT_MAX_COUNT;
        }
        else if (edata->watch_point_value == PCNT_UNIT_MIN_COUNT)
        {
            encoder->pulse_count += PCNT_UNIT_MIN_COUNT;
        }
    }
    return false;
}

/**
 * @brief 获取当前脉冲计数值
 * @param encoder 编码器结构体指针
 * @param count 输出计数值
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t encoder_get_count(encoder_t *encoder, int32_t *count)
{
    if (encoder == NULL || count == NULL || !encoder->is_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int pulse_count = 0;
    esp_err_t ret = pcnt_unit_get_count(encoder->pcnt_unit, &pulse_count);
    if (ret == ESP_OK)
    {
        // 直接返回PCNT硬件计数值，加上累积的溢出计数
        *count = encoder->pulse_count + pulse_count;
        ESP_LOGD(TAG, "编码器计数: 硬件=%d, 累积=%d, 总计=%d", pulse_count, encoder->pulse_count, *count);
    }
    else
    {
        ESP_LOGW(TAG, "读取PCNT计数失败: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief 清除计数器
 * @param encoder 编码器结构体指针
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t encoder_clear_count(encoder_t *encoder)
{
    if (encoder == NULL || !encoder->is_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    encoder->pulse_count = 0;
    return pcnt_unit_clear_count(encoder->pcnt_unit);
}

/**
 * @brief 初始化单个编码器
 * @param encoder 编码器结构体指针
 * @param phase_a_pin A相引脚
 * @param phase_b_pin B相引脚
 * @param unit_id PCNT单元ID
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t encoder_init_single(encoder_t *encoder, int phase_a_pin, int phase_b_pin, int unit_id)
{
    if (encoder == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 配置GPIO引脚 - 编码器信号不使用内部上拉/下拉电阻
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << phase_a_pin) | (1ULL << phase_b_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,     // 禁用内部上拉电阻，让编码器提供信号
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // 禁用内部下拉电阻
        .intr_type = GPIO_INTR_DISABLE};
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "配置编码器GPIO引脚失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "编码器GPIO配置完成 - A相(GPIO%d), B相(GPIO%d)", phase_a_pin, phase_b_pin);

    // 配置PCNT单元
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_UNIT_MAX_COUNT,
        .low_limit = PCNT_UNIT_MIN_COUNT,
        .flags.accum_count = true, // 启用累计计数
    };

    ret = pcnt_new_unit(&unit_config, &encoder->pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建PCNT单元%d失败: %s", unit_id, esp_err_to_name(ret));
        return ret;
    }

    // 配置通道0 (A相作为脉冲输入，B相作为方向控制)
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = phase_a_pin,
        .level_gpio_num = phase_b_pin,
        .flags.invert_edge_input = false,
        .flags.invert_level_input = false,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ret = pcnt_new_channel(encoder->pcnt_unit, &chan_a_config, &pcnt_chan_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建PCNT通道A失败: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        return ret;
    }

    // 配置通道1 (B相作为脉冲输入，A相作为方向控制)
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = phase_b_pin,
        .level_gpio_num = phase_a_pin,
        .flags.invert_edge_input = false,
        .flags.invert_level_input = false,
    };

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ret = pcnt_new_channel(encoder->pcnt_unit, &chan_b_config, &pcnt_chan_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建PCNT通道B失败: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        return ret;
    }

    // 设置边沿和电平动作 - 根据ESP-IDF官方示例实现4倍频正交编码
    // 通道A: A相边沿触发，B相电平控制方向 - 正确配置
    ret = pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置通道A边沿动作失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置通道A电平动作失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 通道B: B相边沿触发，A相电平控制方向 - 修正为与官方示例一致
    ret = pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置通道B边沿动作失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置通道B电平动作失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PCNT通道配置完成 - 4倍频正交编码模式");

    // 设置毛刺滤波器 - 滤除小于1us的毛刺信号
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, // 1微秒
    };
    ret = pcnt_unit_set_glitch_filter(encoder->pcnt_unit, &filter_config);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "设置PCNT毛刺滤波器失败: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "PCNT毛刺滤波器配置成功 (1us)");
    }

    // 添加监视点用于溢出检测
    ret = pcnt_unit_add_watch_point(encoder->pcnt_unit, PCNT_UNIT_MAX_COUNT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "添加最大值监视点失败: %s", esp_err_to_name(ret));
    }

    ret = pcnt_unit_add_watch_point(encoder->pcnt_unit, PCNT_UNIT_MIN_COUNT);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "添加最小值监视点失败: %s", esp_err_to_name(ret));
    }

    // 注册事件回调
    pcnt_event_callbacks_t cbs = {
        .on_reach = encoder_overflow_callback,
    };
    ret = pcnt_unit_register_event_callbacks(encoder->pcnt_unit, &cbs, encoder);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "注册事件回调失败: %s", esp_err_to_name(ret));
    }

    // 启用PCNT单元
    ret = pcnt_unit_enable(encoder->pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "启用PCNT单元失败: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        return ret;
    }

    // 清除计数器并启动
    ret = pcnt_unit_clear_count(encoder->pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "清零PCNT单元%d计数失败: %s", unit_id, esp_err_to_name(ret));
    }

    ret = pcnt_unit_start(encoder->pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "开始PCNT单元%d计数失败: %s", unit_id, esp_err_to_name(ret));
        pcnt_unit_disable(encoder->pcnt_unit);
        pcnt_del_unit(encoder->pcnt_unit);
        return ret;
    }

    // 初始化编码器结构体
    encoder->pulse_count = 0;
    encoder->speed_rpm = 0.0f;
    encoder->mode = ENCODER_MODE_X4; // 默认4倍频模式
    encoder->is_initialized = true;

    ESP_LOGI(TAG, "编码器%d初始化成功 (A相:%d, B相:%d)", unit_id, phase_a_pin, phase_b_pin);

    // 测试读取初始计数值
    int test_count = 0;
    ret = pcnt_unit_get_count(encoder->pcnt_unit, &test_count);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "编码器%d初始计数值: %d", unit_id, test_count);
    }
    else
    {
        ESP_LOGE(TAG, "读取编码器%d初始计数失败: %s", unit_id, esp_err_to_name(ret));
    }

    // 检查GPIO引脚状态
    int level_a = gpio_get_level(phase_a_pin);
    int level_b = gpio_get_level(phase_b_pin);
    ESP_LOGI(TAG, "编码器%d引脚状态 - A相(GPIO%d): %d, B相(GPIO%d): %d",
             unit_id, phase_a_pin, level_a, phase_b_pin, level_b);

    return ESP_OK;
}

/**
 * @brief 双电机系统初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
/**
 * @brief TB6612FNG电机驱动器初始化
 *
 * 本函数负责初始化控制TB6612FNG驱动芯片所需的所有硬件资源，包括GPIO和MCPWM外设。
 * 初始化流程如下：
 * 1. 配置方向控制GPIO：AIN1/AIN2, BIN1/BIN2
 * 2. 配置PWM控制资源（使用MCPWM外设）：
 *    - 创建定时器 (Timer)：提供1kHz的PWM基准频率
 *    - 创建操作器 (Operator)：连接定时器与比较器/生成器
 *    - 创建比较器 (Comparator)：用于生成PWM占空比
 *    - 创建生成器 (Generator)：将比较结果输出到PWM引脚 (PWMA/PWMB)
 * 3. 配置PWM波形生成策略
 * 4. 启动PWM定时器
 *
 * @param motor 指向TB6612FNG电机结构体的指针
 * @return esp_err_t 初始化结果 (ESP_OK: 成功, 其他: 失败)
 */
static esp_err_t tb6612fng_init(tb6612fng_motor_t *motor)
{
    esp_err_t ret;

    // =========================================================================
    // 1. GPIO 初始化 (方向控制引脚)
    // =========================================================================

    // 配置GPIO引脚参数
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE, // 禁用中断
        .mode = GPIO_MODE_OUTPUT,       // 设置为输出模式
        .pin_bit_mask = (1ULL << motor->ain1_pin) | (1ULL << motor->ain2_pin) |
                        (1ULL << motor->bin1_pin) | (1ULL << motor->bin2_pin), // 选中所有方向引脚
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                                 // 禁用内部下拉
        .pull_up_en = GPIO_PULLUP_DISABLE,                                     // 禁用内部上拉
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化所有方向引脚为低电平 (电机停止状态)
    gpio_set_level(motor->ain1_pin, 0);
    gpio_set_level(motor->ain2_pin, 0);
    gpio_set_level(motor->bin1_pin, 0);
    gpio_set_level(motor->bin2_pin, 0);

    // =========================================================================
    // 2. MCPWM 定时器配置 (PWM时基)
    // =========================================================================

    // 配置MCPWM定时器参数
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,                           // 使用MCPWM组0
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,  // 使用默认时钟源 (通常是APB时钟)
        .resolution_hz = 1000000,                // 计数器分辨率: 1MHz (1 tick = 1us)
        .period_ticks = 1000,                    // 周期tick数: 1000 (PWM周期 = 1000us = 1ms)
                                                 // PWM频率 = 1MHz / 1000 = 1kHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP, // 向上计数模式 (0 -> 1000 -> 0)
    };
    ret = mcpwm_new_timer(&timer_config, &motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器创建失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // 3. MCPWM 操作器配置 (Operator)
    // =========================================================================

    // 配置MCPWM操作器参数
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // 必须与定时器在同一组
    };
    ret = mcpwm_new_operator(&operator_config, &motor->operator_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM操作器创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_timer(motor->timer_handle); // 回滚：删除已创建的定时器
        return ret;
    }

    // 将定时器连接到操作器
    ret = mcpwm_operator_connect_timer(motor->operator_handle, motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器和操作器连接失败: %s", esp_err_to_name(ret));
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // =========================================================================
    // 4. MCPWM 比较器配置 (Comparator) - 用于占空比控制
    // =========================================================================

    // 配置比较器参数
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true, // 在计数器归零(TEZ)时更新比较值，防止波形毛刺
    };

    // 创建比较器A (控制电机A速度)
    ret = mcpwm_new_comparator(motor->operator_handle, &comparator_config, &motor->cmpr_a_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM比较器A创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 创建比较器B (控制电机B速度)
    ret = mcpwm_new_comparator(motor->operator_handle, &comparator_config, &motor->cmpr_b_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM比较器B创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_comparator(motor->cmpr_a_handle);
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // =========================================================================
    // 5. MCPWM 生成器配置 (Generator) - 用于PWM波形输出
    // =========================================================================

    // 配置生成器A (输出到 PWMA 引脚)
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = motor->pwma_pin,
    };
    ret = mcpwm_new_generator(motor->operator_handle, &generator_config, &motor->gen_a_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM生成器A创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_comparator(motor->cmpr_b_handle);
        mcpwm_del_comparator(motor->cmpr_a_handle);
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 配置生成器B (输出到 PWMB 引脚)
    generator_config.gen_gpio_num = motor->pwmb_pin;
    ret = mcpwm_new_generator(motor->operator_handle, &generator_config, &motor->gen_b_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM生成器B创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_generator(motor->gen_a_handle);
        mcpwm_del_comparator(motor->cmpr_b_handle);
        mcpwm_del_comparator(motor->cmpr_a_handle);
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // =========================================================================
    // 6. 配置 PWM 波形生成动作
    // =========================================================================
    // 策略：高电平有效
    // - 计数器值 < 比较值：输出高电平
    // - 计数器值 > 比较值：输出低电平

    // 设置生成器A动作：
    // 1. 计数器归零 (Timer Empty) -> 输出高电平
    ret = mcpwm_generator_set_action_on_timer_event(motor->gen_a_handle,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器A定时器事件失败: %s", esp_err_to_name(ret));
        return ret;
    }
    // 2. 计数器值等于比较值 (Compare Match) -> 输出低电平
    ret = mcpwm_generator_set_action_on_compare_event(motor->gen_a_handle,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpr_a_handle, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器A比较事件失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置生成器B动作 (同上)
    ret = mcpwm_generator_set_action_on_timer_event(motor->gen_b_handle,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器B定时器事件失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_generator_set_action_on_compare_event(motor->gen_b_handle,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpr_b_handle, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器B比较事件失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // =========================================================================
    // 7. 启动 PWM 输出
    // =========================================================================

    // 使能定时器
    ret = mcpwm_timer_enable(motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 启动定时器运行 (不停止)
    ret = mcpwm_timer_start_stop(motor->timer_handle, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器开始失败: %s", esp_err_to_name(ret));
        return ret;
    }

    motor->is_initialized = true;
    return ESP_OK;
}

/**
 * @brief 激活双电机使能状态
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_enable(void)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "正在使能双电机系统...");

    g_dual_motor.tb6612fng.motor_a_enabled = true;
    g_dual_motor.tb6612fng.motor_b_enabled = true;

    ESP_LOGI(TAG, "双电机系统使能成功");
    return ESP_OK;
}

/**
 * @brief 编码器初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
static esp_err_t encode_init(void)
{
    ESP_LOGI(TAG, "正在初始化编码器系统...");

    if (g_dual_motor.encoder_a.is_initialized && g_dual_motor.encoder_b.is_initialized)
    {
        ESP_LOGW(TAG, "编码器系统已经初始化完成");
        return ESP_OK;
    }

    esp_err_t ret;

    // 初始化电机A编码器
    ret = encoder_init_single(&g_dual_motor.encoder_a, ENCODER_A_PHASE_A_PIN, ENCODER_A_PHASE_B_PIN, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机A编码器初始化失败");
        // 清理电机A编码器资源
        if (g_dual_motor.encoder_a.is_initialized)
        {
            pcnt_unit_stop(g_dual_motor.encoder_a.pcnt_unit);
            pcnt_unit_disable(g_dual_motor.encoder_a.pcnt_unit);
            pcnt_del_unit(g_dual_motor.encoder_a.pcnt_unit);
            g_dual_motor.encoder_a.is_initialized = false;
        }
        return ret;
    }

    // 初始化电机B编码器
    // 注意：此处交换A/B相引脚顺序，修正计数方向
    ret = encoder_init_single(&g_dual_motor.encoder_b, ENCODER_B_PHASE_B_PIN, ENCODER_B_PHASE_A_PIN, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机B编码器初始化失败");
        // 清理电机B编码器资源
        if (g_dual_motor.encoder_b.is_initialized)
        {
            pcnt_unit_stop(g_dual_motor.encoder_b.pcnt_unit);
            pcnt_unit_disable(g_dual_motor.encoder_b.pcnt_unit);
            pcnt_del_unit(g_dual_motor.encoder_b.pcnt_unit);
            g_dual_motor.encoder_b.is_initialized = false;
        }
        return ret;
    }

    ESP_LOGI(TAG, "编码器系统初始化完成");
    return ESP_OK;
}

//*=========== 外部函数 ==============*//
esp_err_t motor_init(void)
{
    ESP_LOGI(TAG, "正在初始化双电机系统...");

    if (g_dual_motor.is_initialized)
    {
        ESP_LOGW(TAG, "电机系统已经初始化完成");
        return ESP_OK;
    }

    esp_err_t ret;

    // 初始化TB6612FNG驱动器
    ret = tb6612fng_init(&g_dual_motor.tb6612fng);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TB6612FNG初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    g_dual_motor.is_initialized = true;
    ESP_LOGI(TAG, "双电机系统初始化完成");

    // 初始化双电机编码器
    encode_init();
    return ESP_OK;
}

/**
 * @brief 控制双电机停止
 * @param motor_a_stop_mode 电机A停止方式
 * @param motor_b_stop_mode 电机B停止方式
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_stop(motor_stop_mode_t motor_a_stop_mode, motor_stop_mode_t motor_b_stop_mode)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "正在停止电机 - A:%s, B:%s",
             (motor_a_stop_mode == MOTOR_STOP_COAST) ? "滑行" : "刹车",
             (motor_b_stop_mode == MOTOR_STOP_COAST) ? "滑行" : "刹车");

    // 停止电机A
    if (g_dual_motor.tb6612fng.motor_a_enabled)
    {
        if (motor_a_stop_mode == MOTOR_STOP_COAST)
        {
            // 惯性停止：AIN1=0, AIN2=0
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 0);
            ESP_LOGI(TAG, "电机A惯性停止");
        }
        else
        {
            // 刹车停止：AIN1=1, AIN2=1
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 1);
            ESP_LOGI(TAG, "电机A刹车停止");
        }
        // 停止PWM输出
        mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_a_handle, 0);
    }

    // 停止电机B
    if (g_dual_motor.tb6612fng.motor_b_enabled)
    {
        if (motor_b_stop_mode == MOTOR_STOP_COAST)
        {
            // 惯性停止：BIN1=0, BIN2=0
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 0);
            ESP_LOGI(TAG, "电机B惯性停止");
        }
        else
        {
            // 刹车停止：BIN1=1, BIN2=1
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 1);
            ESP_LOGI(TAG, "电机B刹车停止");
        }
        // 停止PWM输出
        mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_b_handle, 0);
    }

    ESP_LOGI(TAG, "双电机停止成功");
    return ESP_OK;
}

/**
 * @brief 解除双电机使能状态
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_disable(void)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "正在禁用双电机系统...");

    // 停止所有电机
    motor_stop(MOTOR_STOP_COAST, MOTOR_STOP_COAST);

    g_dual_motor.tb6612fng.motor_a_enabled = false;
    g_dual_motor.tb6612fng.motor_b_enabled = false;

    ESP_LOGI(TAG, "双电机系统禁用成功");
    return ESP_OK;
}

/**
 * @brief 设置指定电机转速和方向
 * @param motor_id 目标电机标识
 * @param speed 设定的转速值（0-100，对应0%-100%占空比）
 * @param direction 运动方向（前进/后退）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_set_speed_and_dir(motor_id_t motor_id, uint32_t speed, motor_direction_t direction)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 参数范围检查
    if (speed > 100)
    {
        ESP_LOGE(TAG, "无效的速度值: %lu (最大值: 100)", speed);
        return ESP_ERR_INVALID_ARG;
    }

    // 计算PWM比较值 (0-100% 映射到 0-1000)
    uint32_t compare_value = (speed * 1000) / 100;
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "设置电机%s方向为%s, 速度为%lu%%",
             (motor_id == MOTOR_A) ? "A" : (motor_id == MOTOR_B) ? "B"
                                                                 : "双",
             (direction == MOTOR_DIRECTION_FORWARD) ? "正转" : "反转",
             speed);

    switch (motor_id)
    {
    case MOTOR_A:
        if (!g_dual_motor.tb6612fng.motor_a_enabled)
        {
            ESP_LOGE(TAG, "电机A未使能");
            return ESP_ERR_INVALID_STATE;
        }
        // 设置方向
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 0);
        }
        else
        {
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 1);
        }
        // 设置速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_a_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机A速度失败: %s", esp_err_to_name(ret));
        }
        break;

    case MOTOR_B:
        if (!g_dual_motor.tb6612fng.motor_b_enabled)
        {
            ESP_LOGE(TAG, "电机B未使能");
            return ESP_ERR_INVALID_STATE;
        }
        // 设置方向
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 1);
        }
        else
        {
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 0);
        }
        // 设置速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_b_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机B速度失败: %s", esp_err_to_name(ret));
        }
        break;

    case MOTOR_BOTH:
        if (!g_dual_motor.tb6612fng.motor_a_enabled || !g_dual_motor.tb6612fng.motor_b_enabled)
        {
            ESP_LOGE(TAG, "双电机未完全使能");
            return ESP_ERR_INVALID_STATE;
        }
        // 设置电机A方向
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 0);
        }
        else
        {
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 1);
        }
        // 设置电机A速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_a_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机A速度失败: %s", esp_err_to_name(ret));
            return ret;
        }

        // 设置电机B方向
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 1);
        }
        else
        {
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 0);
        }
        // 设置电机B速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_b_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机B速度失败: %s", esp_err_to_name(ret));
            return ret;
        }
        break;

    default:
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ret;
}

/**
 * @brief 获取电机的实际转速
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param rpm 输出转速值（转/分钟），正值(+)：表示正转（计数增加方向），负值(-)：表示反转（计数减少方向）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_get_rpm(motor_id_t motor_id, float *rpm)
{
    if (rpm == NULL)
    {
        ESP_LOGE(TAG, "RPM输出指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    encoder_t *encoder = NULL;
    const char *motor_name = NULL;
    static int32_t last_count_a = 0, last_count_b = 0;
    static int64_t last_time_a = 0, last_time_b = 0;
    int32_t *last_count = NULL;
    int64_t *last_time = NULL;

    switch (motor_id)
    {
    case MOTOR_A:
        encoder = &g_dual_motor.encoder_a;
        motor_name = "电机A";
        last_count = &last_count_a;
        last_time = &last_time_a;
        break;
    case MOTOR_B:
        encoder = &g_dual_motor.encoder_b;
        motor_name = "电机B";
        last_count = &last_count_b;
        last_time = &last_time_b;
        break;
    default:
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (!encoder->is_initialized)
    {
        ESP_LOGE(TAG, "%s编码器未初始化", motor_name);
        return ESP_ERR_INVALID_STATE;
    }

    // 获取当前计数值
    int32_t current_count = 0;
    esp_err_t ret = encoder_get_count(encoder, &current_count);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取%s编码器计数失败: %s", motor_name, esp_err_to_name(ret));
        return ret;
    }

    int64_t current_time = esp_timer_get_time(); // 获取当前系统时间(微秒)

    // 首次调用初始化
    if (*last_time == 0)
    {
        *last_time = current_time;
        *last_count = current_count;
        *rpm = 0.0f;
        return ESP_OK;
    }

    int64_t time_diff_us = current_time - *last_time;
    // 如果时间间隔太短（例如小于10ms），则不进行计算，直接返回上次的转速
    // 这可以避免由于时间间隔过小导致的速度计算抖动
    if (time_diff_us < ENCODER_SAMPLE_TIME_MS * 1000)
    {
        *rpm = encoder->speed_rpm;
        return ESP_OK;
    }

    int32_t count_diff = current_count - *last_count;
    *last_count = current_count;
    *last_time = current_time;

    // 计算转速 - N20电机官方参数
    float time_sec = (float)time_diff_us / 1000000.0f;
    int multiplier = (encoder->mode == ENCODER_MODE_X1) ? 1 : (encoder->mode == ENCODER_MODE_X2) ? 2
                                                                                                 : 4;

    // 1.计算电机轴实际转速（转/秒）
    // 电机轴每转计数 = 基础脉冲数 * 倍频系数
    float motor_counts_per_rev = ENCODER_PPR * multiplier;
    // 电机轴转速(RPS)
    float motor_rps = ((float)count_diff / time_sec) / motor_counts_per_rev;

    // 2.计算输出轴实际转速（转/秒）
    float real_gear_ratio = ENCODER_OUTPUT_PPR / ENCODER_PPR;
    // 输出轴转速(RPS) = 电机轴转速(RPS) / 实际减速比
    float output_rps = motor_rps / real_gear_ratio;

    // 3.计算输出轴实际转速（转/分钟）
    // 输出轴转速(RPM) = 输出轴转速(RPS) * 60
    encoder->speed_rpm = output_rps * 60.0f;
    *rpm = encoder->speed_rpm;

    static float last_output_rpm = 0.0f;
    if (*rpm != last_output_rpm)
    {
        last_output_rpm = *rpm;
        ESP_LOGD(TAG, "%s转速详情: 计数差=%ld, 时间间隔=%.3fs, 电机RPS=%.2f, 输出RPS=%.2f, 输出RPM=%.2f", motor_name, count_diff, time_sec, motor_rps, output_rps, *rpm);
    }
    return ESP_OK;
}