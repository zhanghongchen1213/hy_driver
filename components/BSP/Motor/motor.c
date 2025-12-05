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

// 里程计相关全局变量
static float g_total_distance_mm = 0.0f;   ///< 累积行驶距离（毫米）
static float g_vehicle_angle_deg = 0.0f;   ///< 车体转角（度）
static int32_t g_last_encoder_a_count = 0; ///< 上次电机A编码器计数
static int32_t g_last_encoder_b_count = 0; ///< 上次电机B编码器计数
static esp_err_t tb6612fng_init(tb6612fng_motor_t *motor);
static bool IRAM_ATTR encoder_overflow_callback(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
static esp_err_t encoder_get_count(encoder_t *encoder, int32_t *count);
static esp_err_t encoder_clear_count(encoder_t *encoder);
static esp_err_t encoder_init_single(encoder_t *encoder, int phase_a_pin, int phase_b_pin, int unit_id);

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
 */
static esp_err_t tb6612fng_init(tb6612fng_motor_t *motor)
{
    esp_err_t ret;

    // 配置GPIO引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << motor->ain1_pin) | (1ULL << motor->ain2_pin) |
                        (1ULL << motor->bin1_pin) | (1ULL << motor->bin2_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 初始化方向控制引脚为低电平
    gpio_set_level(motor->ain1_pin, 0);
    gpio_set_level(motor->ain2_pin, 0);
    gpio_set_level(motor->bin1_pin, 0);
    gpio_set_level(motor->bin2_pin, 0);

    // 配置MCPWM定时器
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
        .period_ticks = 1000,     // 1000us = 1kHz PWM频率
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ret = mcpwm_new_timer(&timer_config, &motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器创建失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置MCPWM操作器
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ret = mcpwm_new_operator(&operator_config, &motor->operator_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM操作器创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 连接定时器和操作器
    ret = mcpwm_operator_connect_timer(motor->operator_handle, motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器和操作器连接失败: %s", esp_err_to_name(ret));
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 配置比较器A
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ret = mcpwm_new_comparator(motor->operator_handle, &comparator_config, &motor->cmpr_a_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM比较器A创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 配置比较器B
    ret = mcpwm_new_comparator(motor->operator_handle, &comparator_config, &motor->cmpr_b_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM比较器B创建失败: %s", esp_err_to_name(ret));
        mcpwm_del_comparator(motor->cmpr_a_handle);
        mcpwm_del_operator(motor->operator_handle);
        mcpwm_del_timer(motor->timer_handle);
        return ret;
    }

    // 配置生成器A
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

    // 配置生成器B
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

    // 设置生成器A的PWM波形
    ret = mcpwm_generator_set_action_on_timer_event(motor->gen_a_handle,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器A定时器事件失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_generator_set_action_on_compare_event(motor->gen_a_handle,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpr_a_handle, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置生成器A比较事件失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置生成器B的PWM波形
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

    // 启动定时器
    ret = mcpwm_timer_enable(motor->timer_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器启动失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_start_stop(motor->timer_handle, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MCPWM定时器开始失败: %s", esp_err_to_name(ret));
        return ret;
    }

    motor->is_initialized = true;
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

    encode_init();
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
 * @brief 设置指定电机转速
 * @param motor_id 目标电机标识
 * @param speed 设定的转速值（0-100，对应0%-100%占空比）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_set_speed(motor_id_t motor_id, uint32_t speed)
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

    switch (motor_id)
    {
    case MOTOR_A:
        if (!g_dual_motor.tb6612fng.motor_a_enabled)
        {
            ESP_LOGE(TAG, "电机A未使能");
            return ESP_ERR_INVALID_STATE;
        }
        // 设置MCPWM比较器值
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_a_handle, compare_value);
        if (ret == ESP_OK)
        {
            ESP_LOGD(TAG, "电机A速度设置为 %lu%% (PWM: %lu)", speed, compare_value);
        }
        else
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
        // 设置MCPWM比较器值
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_b_handle, compare_value);
        if (ret == ESP_OK)
        {
            ESP_LOGD(TAG, "电机B速度设置为 %lu%% (PWM: %lu)", speed, compare_value);
        }
        else
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
        // 设置电机A速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_a_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机A速度失败: %s", esp_err_to_name(ret));
            return ret;
        }

        // 设置电机B速度
        ret = mcpwm_comparator_set_compare_value(g_dual_motor.tb6612fng.cmpr_b_handle, compare_value);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "设置电机B速度失败: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGD(TAG, "双电机速度设置为 %lu%% (PWM: %lu)", speed, compare_value);
        break;

    default:
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ret;
}

/**
 * @brief 控制指定电机运动方向
 * @param motor_id 目标电机标识
 * @param direction 运动方向（前进/后退）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_set_direction(motor_id_t motor_id, motor_direction_t direction)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGD(TAG, "设置电机%s方向为%s",
             (motor_id == MOTOR_A) ? "A" : (motor_id == MOTOR_B) ? "B"
                                                                 : "双",
             (direction == MOTOR_DIRECTION_FORWARD) ? "正转" : "反转");

    switch (motor_id)
    {
    case MOTOR_A:
        if (!g_dual_motor.tb6612fng.motor_a_enabled)
        {
            ESP_LOGE(TAG, "电机A未使能");
            return ESP_ERR_INVALID_STATE;
        }
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            // 正转：AIN1=1, AIN2=0
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 0);
        }
        else
        {
            // 反转：AIN1=0, AIN2=1
            gpio_set_level(g_dual_motor.tb6612fng.ain1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.ain2_pin, 1);
        }
        ESP_LOGD(TAG, "电机A方向设置成功: %s",
                 (direction == MOTOR_DIRECTION_FORWARD) ? "正转" : "反转");
        break;

    case MOTOR_B:
        if (!g_dual_motor.tb6612fng.motor_b_enabled)
        {
            ESP_LOGE(TAG, "电机B未使能");
            return ESP_ERR_INVALID_STATE;
        }
        if (direction == MOTOR_DIRECTION_FORWARD)
        {
            // 正转：BIN1=0, BIN2=1
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 0);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 1);
        }
        else
        {
            // 反转：BIN1=1, BIN2=0
            gpio_set_level(g_dual_motor.tb6612fng.bin1_pin, 1);
            gpio_set_level(g_dual_motor.tb6612fng.bin2_pin, 0);
        }
        ESP_LOGD(TAG, "电机B方向设置成功: %s",
                 (direction == MOTOR_DIRECTION_FORWARD) ? "正转" : "反转");
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

        ESP_LOGD(TAG, "双电机方向设置为 %s", (direction == MOTOR_DIRECTION_FORWARD) ? "正转" : "反转");
        break;

    default:
        ESP_LOGE(TAG, "无效的电机ID: %d", motor_id);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief 编码器初始化
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t encode_init(void)
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
        return ret;
    }

    // 初始化电机B编码器
    ret = encoder_init_single(&g_dual_motor.encoder_b, ENCODER_B_PHASE_A_PIN, ENCODER_B_PHASE_B_PIN, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "电机B编码器初始化失败");
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

    ESP_LOGI(TAG, "编码器系统初始化完成");
    return ESP_OK;
}

/**
 * @brief 获取指定电机的转速
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param rpm 输出转速值（转/分钟）
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
    static uint32_t last_time_a = 0, last_time_b = 0;
    int32_t *last_count = NULL;
    uint32_t *last_time = NULL;

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

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 首次调用初始化
    if (*last_time == 0)
    {
        *last_time = current_time;
        *last_count = current_count;
        *rpm = 0.0f;
        return ESP_OK;
    }

    uint32_t time_diff = current_time - *last_time;
    if (time_diff < ENCODER_SAMPLE_TIME_MS)
    {
        // 采样时间间隔太短，返回上次计算的转速
        *rpm = encoder->speed_rpm;
        return ESP_OK;
    }

    int32_t count_diff = current_count - *last_count;
    *last_count = current_count;
    *last_time = current_time;

    // 计算转速 - N20电机官方参数
    float time_sec = (float)time_diff / 1000.0f;
    int multiplier = (encoder->mode == ENCODER_MODE_X1) ? 1 : (encoder->mode == ENCODER_MODE_X2) ? 2
                                                                                                 : 4;

    // 使用官方标准：减速后输出轴357.70线/转，4倍频后为1430.8计数/转
    float counts_per_output_rev = ENCODER_OUTPUT_PPR * multiplier; // 输出轴每转的计数数

    // 输出轴转速 = 计数频率 / 每转计数数 * 60
    encoder->speed_rpm = (float)count_diff / (time_sec * counts_per_output_rev) * 60.0f;
    *rpm = encoder->speed_rpm;

    ESP_LOGD(TAG, "%s转速: %.2f RPM (计数差值: %ld, 时间差: %lu ms, 当前计数: %ld)",
             motor_name, *rpm, count_diff, time_diff, current_count);

    return ESP_OK;
}

/**
 * @brief 获取车体累积行驶距离
 * @param distance 输出累积行驶距离（毫米）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 基于双轮编码器数据计算的累积行驶距离
 */
esp_err_t motor_get_travel_distance(float *distance)
{
    if (distance == NULL)
    {
        ESP_LOGE(TAG, "距离输出指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 获取当前编码器计数
    int32_t current_count_a, current_count_b;
    if (encoder_get_count(&g_dual_motor.encoder_a, &current_count_a) != ESP_OK ||
        encoder_get_count(&g_dual_motor.encoder_b, &current_count_b) != ESP_OK)
    {
        ESP_LOGE(TAG, "获取编码器计数失败");
        return ESP_FAIL;
    }

    // 计算编码器计数差值
    int32_t delta_count_a = current_count_a - g_last_encoder_a_count;
    int32_t delta_count_b = current_count_b - g_last_encoder_b_count;

    // 更新上次计数值
    g_last_encoder_a_count = current_count_a;
    g_last_encoder_b_count = current_count_b;

    // 计算轮子转动距离（毫米）
    // 4倍频模式下，每转计数 = ENCODER_OUTPUT_PPR * 4
    float counts_per_rev = ENCODER_OUTPUT_PPR * 4.0f;
    float distance_a = (float)delta_count_a / counts_per_rev * WHEEL_CIRCUMFERENCE_MM;
    float distance_b = (float)delta_count_b / counts_per_rev * WHEEL_CIRCUMFERENCE_MM;

    // 车体行驶距离为两轮平均距离
    float delta_distance = (distance_a + distance_b) / 2.0f;
    g_total_distance_mm += delta_distance;

    *distance = g_total_distance_mm;
    ESP_LOGD(TAG, "累积行驶距离: %.2f mm", *distance);
    return ESP_OK;
}

/**
 * @brief 获取车体转角
 * @param angle 输出车体转角（度）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 基于差分驱动模型计算的车体转角，正值为逆时针转向
 */
esp_err_t motor_get_vehicle_angle(float *angle)
{
    if (angle == NULL)
    {
        ESP_LOGE(TAG, "角度输出指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 获取当前编码器计数
    int32_t current_count_a, current_count_b;
    if (encoder_get_count(&g_dual_motor.encoder_a, &current_count_a) != ESP_OK ||
        encoder_get_count(&g_dual_motor.encoder_b, &current_count_b) != ESP_OK)
    {
        ESP_LOGE(TAG, "获取编码器计数失败");
        return ESP_FAIL;
    }

    // 计算编码器计数差值
    int32_t delta_count_a = current_count_a - g_last_encoder_a_count;
    int32_t delta_count_b = current_count_b - g_last_encoder_b_count;

    // 更新上次计数值
    g_last_encoder_a_count = current_count_a;
    g_last_encoder_b_count = current_count_b;

    // 计算轮子转动距离（毫米）
    float counts_per_rev = ENCODER_OUTPUT_PPR * 4.0f;
    float distance_a = (float)delta_count_a / counts_per_rev * WHEEL_CIRCUMFERENCE_MM;
    float distance_b = (float)delta_count_b / counts_per_rev * WHEEL_CIRCUMFERENCE_MM;

    // 差分驱动转角计算：θ = (distance_b - distance_a) / wheelbase
    // 正值表示逆时针转向（右轮比左轮走得远）
    float delta_angle_rad = (distance_b - distance_a) / WHEEL_BASE_MM;
    float delta_angle_deg = delta_angle_rad * 180.0f / M_PI;
    g_vehicle_angle_deg += delta_angle_deg;

    // 角度归一化到 [-180, 180] 度范围
    while (g_vehicle_angle_deg > 180.0f)
        g_vehicle_angle_deg -= 360.0f;
    while (g_vehicle_angle_deg < -180.0f)
        g_vehicle_angle_deg += 360.0f;

    *angle = g_vehicle_angle_deg;
    ESP_LOGD(TAG, "车体转角: %.2f 度", *angle);
    return ESP_OK;
}

/**
 * @brief 重置里程计数据
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 * @note 清零累积行驶距离和车体转角数据
 */
esp_err_t motor_reset_odometry(void)
{
    if (!g_dual_motor.is_initialized)
    {
        ESP_LOGE(TAG, "电机系统未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // 重置里程计数据
    g_total_distance_mm = 0.0f;
    g_vehicle_angle_deg = 0.0f;

    // 获取当前编码器计数作为新的基准
    if (encoder_get_count(&g_dual_motor.encoder_a, &g_last_encoder_a_count) != ESP_OK ||
        encoder_get_count(&g_dual_motor.encoder_b, &g_last_encoder_b_count) != ESP_OK)
    {
        ESP_LOGE(TAG, "获取编码器计数失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "里程计数据已重置");
    return ESP_OK;
}

/**
 * @brief 获取指定电机的位置角度
 * @param motor_id 目标电机标识（MOTOR_A或MOTOR_B）
 * @param position 输出位置角度值（度）
 * @retval ESP_OK 成功
 * @retval ESP_FAIL 失败
 */
esp_err_t motor_get_position(motor_id_t motor_id, float *position)
{
    if (position == NULL)
    {
        ESP_LOGE(TAG, "位置输出指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    encoder_t *encoder = NULL;
    const char *motor_name = NULL;

    switch (motor_id)
    {
    case MOTOR_A:
        encoder = &g_dual_motor.encoder_a;
        motor_name = "电机A";
        break;
    case MOTOR_B:
        encoder = &g_dual_motor.encoder_b;
        motor_name = "电机B";
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

    // 计算位置角度 - N20电机官方参数
    int multiplier = (encoder->mode == ENCODER_MODE_X1) ? 1 : (encoder->mode == ENCODER_MODE_X2) ? 2
                                                                                                 : 4;

    // 使用官方标准：减速后输出轴357.70线/转，4倍频后为1430.8计数/转
    float counts_per_output_rev = ENCODER_OUTPUT_PPR * multiplier; // 输出轴每转的计数数

    // 输出轴角度 = 计数值 * 360° / 每转计数数
    *position = (float)current_count * 360.0f / counts_per_output_rev;

    ESP_LOGD(TAG, "%s位置: %.2f° (脉冲计数: %ld)", motor_name, *position, current_count);

    return ESP_OK;
}

/**
 * @brief 获取完整的车体运动状态
 * @param[out] motion_state 车体运动状态结构体指针
 * @retval ESP_OK 成功
 * @retval ESP_ERR_INVALID_ARG 参数无效
 * @retval ESP_ERR_INVALID_STATE 编码器未初始化
 */
esp_err_t motor_get_motion_state(vehicle_motion_state_t *motion_state)
{
    // 参数检查
    if (motion_state == NULL)
    {
        ESP_LOGE(TAG, "运动状态结构体指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    // 编码器初始化检查
    if (!g_dual_motor.encoder_a.is_initialized || !g_dual_motor.encoder_b.is_initialized)
    {
        ESP_LOGE(TAG, "编码器未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    // 初始化结构体
    memset(motion_state, 0, sizeof(vehicle_motion_state_t));
    motion_state->data_valid = false;

    // 获取电机A转速
    ret = motor_get_rpm(MOTOR_A, &motion_state->motor_a_rpm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取电机A转速失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取电机B转速
    ret = motor_get_rpm(MOTOR_B, &motion_state->motor_b_rpm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取电机B转速失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取电机A位置角度
    ret = motor_get_position(MOTOR_A, &motion_state->motor_a_position_deg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取电机A位置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取电机B位置角度
    ret = motor_get_position(MOTOR_B, &motion_state->motor_b_position_deg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取电机B位置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取累积行驶距离
    ret = motor_get_travel_distance(&motion_state->travel_distance_mm);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取行驶距离失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 获取车体转角
    ret = motor_get_vehicle_angle(&motion_state->vehicle_angle_deg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "获取车体转角失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置时间戳（毫秒）
    motion_state->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // 标记数据有效
    motion_state->data_valid = true;

    ESP_LOGD(TAG, "运动状态 - 电机A: %.2f RPM, %.2f°; 电机B: %.2f RPM, %.2f°; 距离: %.2f mm; 转角: %.2f°",
             motion_state->motor_a_rpm, motion_state->motor_a_position_deg,
             motion_state->motor_b_rpm, motion_state->motor_b_position_deg,
             motion_state->travel_distance_mm, motion_state->vehicle_angle_deg);

    return ESP_OK;
}