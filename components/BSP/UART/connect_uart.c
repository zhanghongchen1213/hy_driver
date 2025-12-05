#include "connect_uart.h"
#include "esp_timer.h"

static const char *TAG = "UART_COM";

// =============================================================================
// 全局变量定义
// =============================================================================

static uart_comm_control_t g_uart_comm;         ///< UART通信控制结构
static uart_rx_callback_t g_rx_callback = NULL; ///< 接收回调函数
static void *g_user_data = NULL;                ///< 用户数据指针

// =============================================================================
// 内部函数声明
// =============================================================================

static void uart_rx_task(void *pvParameters);
static void uart_tx_task(void *pvParameters);

static esp_err_t uart_hardware_init(void);
static esp_err_t uart_create_tasks(void);
static void uart_packet_parser_reset(packet_parser_t *parser);
static bool uart_parse_byte(packet_parser_t *parser, uint8_t byte);

static void uart_update_error_stats(uart_error_type_t error_type);

// =============================================================================
// 数据包验证实现
// =============================================================================

/**
 * @brief 验证上行数据包格式
 * @param packet 上行数据包指针
 * @retval true 格式正确
 * @retval false 格式错误
 */
bool uart_comm_validate_uplink_packet(const uart_uplink_packet_t *packet)
{
    if (packet == NULL)
    {
        return false;
    }

    // 检查起始和结束标志
    if (packet->start_flag != UART_PACKET_START_FLAG ||
        packet->end_flag != UART_PACKET_END_FLAG)
    {
        return false;
    }

    // 检查数据包长度
    if (packet->length != UART_UPLINK_DATA_SIZE)
    {
        ESP_LOGW(TAG, "上行数据包长度错误: %d，期望: %d", packet->length, UART_UPLINK_DATA_SIZE);
        return false;
    }

    return true;
}

/**
 * @brief 验证下行数据包格式
 * @param packet 下行数据包指针
 * @retval true 格式正确
 * @retval false 格式错误
 */
bool uart_comm_validate_downlink_packet(const uart_downlink_packet_t *packet)
{
    if (packet == NULL)
    {
        return false;
    }

    // 检查起始和结束标志
    if (packet->start_flag != UART_PACKET_START_FLAG ||
        packet->end_flag != UART_PACKET_END_FLAG)
    {
        return false;
    }

    // 检查数据包长度
    if (packet->length != UART_DOWNLINK_DATA_SIZE)
    {
        ESP_LOGW(TAG, "下行数据包长度错误: %d，期望: %d", packet->length, UART_DOWNLINK_DATA_SIZE);
        return false;
    }

    return true;
}

// =============================================================================
// 数据包处理函数
// =============================================================================

/**
 * @brief 重置数据包解析器
 * @param parser 解析器指针
 */
static void uart_packet_parser_reset(packet_parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    parser->state = PACKET_STATE_WAIT_START;
    parser->data_index = 0;
    parser->timeout_start = 0;
    memset(&parser->packet, 0, sizeof(uart_packet_union_t));
}

/**
 * @brief 解析单个字节
 * @param parser 解析器指针
 * @param byte 接收到的字节
 * @retval true 数据包解析完成
 * @retval false 数据包未完成
 */
static bool uart_parse_byte(packet_parser_t *parser, uint8_t byte)
{
    if (parser == NULL)
    {
        return false;
    }

    uint32_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒

    // 超时检查
    if (parser->state != PACKET_STATE_WAIT_START &&
        parser->timeout_start > 0 &&
        (current_time - parser->timeout_start) > UART_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "数据包解析超时，重置解析器");
        uart_packet_parser_reset(parser);
        uart_update_error_stats(UART_ERROR_TIMEOUT);
    }

    switch (parser->state)
    {
    case PACKET_STATE_WAIT_START:
        if (byte == UART_PACKET_START_FLAG)
        {
            parser->packet.downlink.start_flag = byte;
            parser->state = PACKET_STATE_WAIT_LENGTH;
            parser->timeout_start = current_time;
        }
        break;

    case PACKET_STATE_WAIT_LENGTH:
        parser->packet.downlink.length = byte;
        if (byte == UART_DOWNLINK_DATA_SIZE)
        {
            parser->state = PACKET_STATE_WAIT_DATA;
            parser->data_index = 0;
        }
        else
        {
            ESP_LOGW(TAG, "数据包长度错误: %d", byte);
            uart_packet_parser_reset(parser);
            uart_update_error_stats(UART_ERROR_LENGTH);
        }
        break;

    case PACKET_STATE_WAIT_DATA:
        parser->packet.raw_data[2 + parser->data_index++] = byte;
        if (parser->data_index >= UART_DOWNLINK_DATA_SIZE)
        {
            parser->state = PACKET_STATE_WAIT_END;
        }
        break;

    case PACKET_STATE_WAIT_END:
        parser->packet.downlink.end_flag = byte;
        if (byte == UART_PACKET_END_FLAG)
        {
            parser->state = PACKET_STATE_COMPLETE;
            return true; // 数据包解析完成
        }
        else
        {
            ESP_LOGW(TAG, "结束标志错误: 0x%02X", byte);
            uart_packet_parser_reset(parser);
            uart_update_error_stats(UART_ERROR_FORMAT);
        }
        break;

    default:
        uart_packet_parser_reset(parser);
        break;
    }

    return false;
}

// =============================================================================
// 错误统计函数
// =============================================================================

/**
 * @brief 更新错误统计
 * @param error_type 错误类型
 */
static void uart_update_error_stats(uart_error_type_t error_type)
{
    g_uart_comm.error_stats.last_error = error_type;

    switch (error_type)
    {
    case UART_ERROR_TIMEOUT:
        g_uart_comm.error_stats.timeout_count++;
        break;
    case UART_ERROR_FORMAT:
    case UART_ERROR_LENGTH:
        g_uart_comm.error_stats.format_error_count++;
        break;
    default:
        break;
    }
}

// =============================================================================
// UART硬件初始化
// =============================================================================

/**
 * @brief UART硬件初始化
 * @retval ESP_OK 初始化成功
 * @retval ESP_FAIL 初始化失败
 */
static esp_err_t uart_hardware_init(void)
{
    esp_err_t ret;

    // UART配置参数
    uart_config_t uart_config = {
        .baud_rate = UART_COMM_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 配置UART参数
    ret = uart_param_config(UART_COMM_PORT_NUM, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 设置UART引脚
    ret = uart_set_pin(UART_COMM_PORT_NUM, UART_COMM_TXD_PIN, UART_COMM_RXD_PIN,
                       UART_COMM_RTS_PIN, UART_COMM_CTS_PIN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART引脚设置失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 安装UART驱动，启用DMA
    ret = uart_driver_install(UART_COMM_PORT_NUM, UART_COMM_BUF_SIZE, UART_COMM_BUF_SIZE,
                              UART_COMM_QUEUE_SIZE, &g_uart_comm.uart_queue, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART硬件初始化成功 - 端口:%d, 波特率:%d, TXD:%d, RXD:%d",
             UART_COMM_PORT_NUM, UART_COMM_BAUD_RATE, UART_COMM_TXD_PIN, UART_COMM_RXD_PIN);

    return ESP_OK;
}

// =============================================================================
// 任务函数实现
// =============================================================================

/**
 * @brief UART接收任务
 * @param pvParameters 任务参数
 */
static void uart_rx_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *rx_buffer = malloc(UART_COMM_BUF_SIZE);

    if (rx_buffer == NULL)
    {
        ESP_LOGE(TAG, "接收缓冲区内存分配失败");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART接收任务启动");

    while (1)
    {
        // 等待UART事件
        if (xQueueReceive(g_uart_comm.uart_queue, &event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                // 读取数据
                int len = uart_read_bytes(UART_COMM_PORT_NUM, rx_buffer, event.size, 0);
                if (len > 0)
                {
                    // 逐字节解析数据包
                    for (int i = 0; i < len; i++)
                    {
                        if (uart_parse_byte(&g_uart_comm.parser, rx_buffer[i]))
                        {
                            // 数据包解析完成
                            if (uart_comm_validate_downlink_packet(&g_uart_comm.parser.packet.downlink))
                            {
                                g_uart_comm.error_stats.total_rx_count++;

                                // 调用接收回调函数
                                if (g_rx_callback != NULL)
                                {
                                    g_rx_callback(&g_uart_comm.parser.packet.downlink, g_user_data);
                                }

                                ESP_LOGD(TAG, "接收到有效下行数据包 - 线速度:%.2f, 角速度:%.2f",
                                         g_uart_comm.parser.packet.downlink.linear_velocity,
                                         g_uart_comm.parser.packet.downlink.angular_velocity);
                            }
                            else
                            {
                                ESP_LOGW(TAG, "数据包格式失败");
                                uart_update_error_stats(UART_ERROR_FORMAT);
                            }

                            // 重置解析器
                            uart_packet_parser_reset(&g_uart_comm.parser);
                        }
                    }
                }
            }
            break;

            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "UART FIFO溢出");
                uart_flush_input(UART_COMM_PORT_NUM);
                uart_update_error_stats(UART_ERROR_BUFFER_FULL);
                break;

            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART缓冲区满");
                uart_flush_input(UART_COMM_PORT_NUM);
                uart_update_error_stats(UART_ERROR_BUFFER_FULL);
                break;

            case UART_BREAK:
                ESP_LOGW(TAG, "UART接收到BREAK信号");
                break;

            case UART_PARITY_ERR:
                ESP_LOGW(TAG, "UART奇偶校验错误");
                uart_update_error_stats(UART_ERROR_HARDWARE);
                break;

            case UART_FRAME_ERR:
                ESP_LOGW(TAG, "UART帧错误");
                uart_update_error_stats(UART_ERROR_HARDWARE);
                break;

            default:
                ESP_LOGD(TAG, "未知UART事件: %d", event.type);
                break;
            }
        }
    }

    free(rx_buffer);
    vTaskDelete(NULL);
}

/**
 * @brief UART发送任务
 * @param pvParameters 任务参数
 */
static void uart_tx_task(void *pvParameters)
{
    uart_uplink_packet_t tx_packet;

    ESP_LOGI(TAG, "UART发送任务启动");

    while (1)
    {
        // 等待发送队列中的数据包
        if (xQueueReceive(g_uart_comm.tx_queue, &tx_packet, portMAX_DELAY))
        {
            // 获取发送互斥锁
            if (xSemaphoreTake(g_uart_comm.tx_mutex, pdMS_TO_TICKS(UART_TIMEOUT_MS)) == pdTRUE)
            {
                g_uart_comm.state = UART_STATE_SENDING;

                // 发送前调试信息
                ESP_LOGI(TAG, "准备发送数据包，大小:%d字节", sizeof(uart_uplink_packet_t));
                ESP_LOGI(TAG, "数据包内容: 起始=0x%02X, 长度=0x%02X, 结束=0x%02X", 
                         tx_packet.start_flag, tx_packet.length, tx_packet.end_flag);
                
                // 打印发送前的完整原始数据
                ESP_LOG_BUFFER_HEXDUMP(TAG, &tx_packet, sizeof(uart_uplink_packet_t), ESP_LOG_INFO);
                
                // 清空UART发送缓冲区
                uart_flush(UART_COMM_PORT_NUM);
                
                // 发送数据包 - 根据调试配置选择发送方式
                int sent = 0;
                
#if UART_DEBUG_BYTE_BY_BYTE
                // 逐字节发送调试模式
                uint8_t *data_ptr = (uint8_t *)&tx_packet;
                ESP_LOGI(TAG, "=== 开始逐字节发送调试 ===");
                
                for (int i = 0; i < sizeof(uart_uplink_packet_t); i++)
                {
                    int byte_sent = uart_write_bytes(UART_COMM_PORT_NUM, &data_ptr[i], 1);
                    if (byte_sent == 1)
                    {
                        sent++;
                        ESP_LOGI(TAG, "[%02d] 0x%02X ✓", i, data_ptr[i]);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "[%02d] 0x%02X ✗ 发送失败", i, data_ptr[i]);
                        break;
                    }
                    
                    // 等待当前字节发送完成
                    uart_wait_tx_done(UART_COMM_PORT_NUM, pdMS_TO_TICKS(10));
                    
                    // 添加延时确保稳定发送
                    if (UART_DEBUG_SEND_DELAY_MS > 0)
                    {
                        vTaskDelay(pdMS_TO_TICKS(UART_DEBUG_SEND_DELAY_MS));
                    }
                }
                ESP_LOGI(TAG, "=== 逐字节发送完成，共%d字节 ===", sent);
#else
                // 批量发送模式
                ESP_LOGI(TAG, "使用批量发送模式");
                sent = uart_write_bytes(UART_COMM_PORT_NUM, &tx_packet, sizeof(uart_uplink_packet_t));
#endif
                
                // 等待发送完成
                esp_err_t wait_result = uart_wait_tx_done(UART_COMM_PORT_NUM, pdMS_TO_TICKS(100));
                
                ESP_LOGI(TAG, "逐字节发送完成，总共发送:%d字节", sent);
                
                if (sent == sizeof(uart_uplink_packet_t))
                {
                    g_uart_comm.error_stats.total_tx_count++;
                    ESP_LOGI(TAG, "数据包发送成功，已发送:%d字节，等待结果:%s", 
                             sent, esp_err_to_name(wait_result));
                }
                else
                {
                    ESP_LOGE(TAG, "发送数据包失败 - 期望:%d, 实际:%d, 等待结果:%s", 
                             sizeof(uart_uplink_packet_t), sent, esp_err_to_name(wait_result));
                    uart_update_error_stats(UART_ERROR_HARDWARE);
                }

                g_uart_comm.state = UART_STATE_IDLE;
                xSemaphoreGive(g_uart_comm.tx_mutex);
            }
            else
            {
                ESP_LOGW(TAG, "获取发送互斥锁超时");
                uart_update_error_stats(UART_ERROR_TIMEOUT);
            }
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief 创建UART任务
 * @retval ESP_OK 创建成功
 * @retval ESP_FAIL 创建失败
 */
static esp_err_t uart_create_tasks(void)
{
    BaseType_t ret;

    // 创建接收任务
    ret = xTaskCreate(uart_rx_task, "uart_rx_task", UART_COMM_TASK_STACK_SIZE,
                      NULL, UART_COMM_TASK_PRIORITY, &g_uart_comm.rx_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建接收任务失败");
        return ESP_FAIL;
    }

    // 创建发送任务
    ret = xTaskCreate(uart_tx_task, "uart_tx_task", UART_COMM_TASK_STACK_SIZE,
                      NULL, UART_COMM_TASK_PRIORITY, &g_uart_comm.tx_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建发送任务失败");
        vTaskDelete(g_uart_comm.rx_task_handle);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART任务创建成功");
    return ESP_OK;
}

// =============================================================================
// API函数实现
// =============================================================================

/**
 * @brief 初始化UART通信模块
 * @param rx_callback 接收回调函数
 * @param user_data 用户数据指针
 * @retval ESP_OK 初始化成功
 * @retval ESP_FAIL 初始化失败
 */
esp_err_t uart_comm_init(uart_rx_callback_t rx_callback, void *user_data)
{
    esp_err_t ret;

    // 检查是否已经初始化
    if (g_uart_comm.is_initialized)
    {
        ESP_LOGW(TAG, "UART通信模块已经初始化");
        return ESP_OK;
    }

    // 清零控制结构
    memset(&g_uart_comm, 0, sizeof(uart_comm_control_t));

    // 设置回调函数和用户数据
    g_rx_callback = rx_callback;
    g_user_data = user_data;

    // 初始化解析器
    uart_packet_parser_reset(&g_uart_comm.parser);

    // 创建互斥锁
    g_uart_comm.tx_mutex = xSemaphoreCreateMutex();
    if (g_uart_comm.tx_mutex == NULL)
    {
        ESP_LOGE(TAG, "创建发送互斥锁失败");
        return ESP_FAIL;
    }

    g_uart_comm.rx_mutex = xSemaphoreCreateMutex();
    if (g_uart_comm.rx_mutex == NULL)
    {
        ESP_LOGE(TAG, "创建接收互斥锁失败");
        vSemaphoreDelete(g_uart_comm.tx_mutex);
        return ESP_FAIL;
    }

    // 创建发送队列
    g_uart_comm.tx_queue = xQueueCreate(10, sizeof(uart_uplink_packet_t));
    if (g_uart_comm.tx_queue == NULL)
    {
        ESP_LOGE(TAG, "创建发送队列失败");
        vSemaphoreDelete(g_uart_comm.tx_mutex);
        vSemaphoreDelete(g_uart_comm.rx_mutex);
        return ESP_FAIL;
    }

    // 初始化UART硬件
    ret = uart_hardware_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "UART硬件初始化失败");
        vQueueDelete(g_uart_comm.tx_queue);
        vSemaphoreDelete(g_uart_comm.tx_mutex);
        vSemaphoreDelete(g_uart_comm.rx_mutex);
        return ret;
    }

    // 创建任务
    ret = uart_create_tasks();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建UART任务失败");
        uart_driver_delete(UART_COMM_PORT_NUM);
        vQueueDelete(g_uart_comm.tx_queue);
        vSemaphoreDelete(g_uart_comm.tx_mutex);
        vSemaphoreDelete(g_uart_comm.rx_mutex);
        return ret;
    }

    // 设置初始状态
    g_uart_comm.state = UART_STATE_IDLE;
    g_uart_comm.is_initialized = true;

    ESP_LOGI(TAG, "UART通信模块初始化成功");
    return ESP_OK;
}

/**
 * @brief 反初始化UART通信模块
 * @retval ESP_OK 反初始化成功
 * @retval ESP_FAIL 反初始化失败
 */
esp_err_t uart_comm_deinit(void)
{
    if (!g_uart_comm.is_initialized)
    {
        ESP_LOGW(TAG, "UART通信模块未初始化");
        return ESP_OK;
    }

    // 删除任务
    if (g_uart_comm.rx_task_handle != NULL)
    {
        vTaskDelete(g_uart_comm.rx_task_handle);
        g_uart_comm.rx_task_handle = NULL;
    }

    if (g_uart_comm.tx_task_handle != NULL)
    {
        vTaskDelete(g_uart_comm.tx_task_handle);
        g_uart_comm.tx_task_handle = NULL;
    }

    // 删除UART驱动
    uart_driver_delete(UART_COMM_PORT_NUM);

    // 删除队列和互斥锁
    if (g_uart_comm.tx_queue != NULL)
    {
        vQueueDelete(g_uart_comm.tx_queue);
        g_uart_comm.tx_queue = NULL;
    }

    if (g_uart_comm.tx_mutex != NULL)
    {
        vSemaphoreDelete(g_uart_comm.tx_mutex);
        g_uart_comm.tx_mutex = NULL;
    }

    if (g_uart_comm.rx_mutex != NULL)
    {
        vSemaphoreDelete(g_uart_comm.rx_mutex);
        g_uart_comm.rx_mutex = NULL;
    }

    // 清零控制结构
    memset(&g_uart_comm, 0, sizeof(uart_comm_control_t));
    g_rx_callback = NULL;
    g_user_data = NULL;

    ESP_LOGI(TAG, "UART通信模块反初始化成功");
    return ESP_OK;
}

/**
 * @brief 发送上行数据包（非阻塞）
 * @param packet 上行数据包指针
 * @retval ESP_OK 发送成功
 * @retval ESP_ERR_INVALID_ARG 参数错误
 * @retval ESP_ERR_INVALID_STATE 模块未初始化
 * @retval ESP_ERR_NO_MEM 内存不足
 */
esp_err_t uart_comm_send_uplink_packet(const uart_uplink_packet_t *packet)
{
    if (!g_uart_comm.is_initialized)
    {
        ESP_LOGE(TAG, "UART通信模块未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    if (packet == NULL)
    {
        ESP_LOGE(TAG, "数据包指针为空");
        return ESP_ERR_INVALID_ARG;
    }

    // 创建发送数据包副本并设置协议字段
    uart_uplink_packet_t tx_packet = *packet;
    tx_packet.start_flag = UART_PACKET_START_FLAG;
    tx_packet.length = UART_UPLINK_DATA_SIZE; // 数据载荷长度（不包括包头包尾）
    tx_packet.end_flag = UART_PACKET_END_FLAG;

    // 将数据包放入发送队列
    if (xQueueSend(g_uart_comm.tx_queue, &tx_packet, pdMS_TO_TICKS(UART_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGW(TAG, "发送队列满，数据包丢弃");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief 获取通信状态
 * @retval uart_comm_state_t 当前通信状态
 */
uart_comm_state_t uart_comm_get_state(void)
{
    return g_uart_comm.state;
}

/**
 * @brief 获取错误统计信息
 * @param stats 错误统计结构指针
 * @retval ESP_OK 获取成功
 * @retval ESP_ERR_INVALID_ARG 参数错误
 */
esp_err_t uart_comm_get_error_stats(uart_error_stats_t *stats)
{
    if (stats == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(stats, &g_uart_comm.error_stats, sizeof(uart_error_stats_t));
    return ESP_OK;
}

/**
 * @brief 清除错误统计
 * @retval ESP_OK 清除成功
 */
esp_err_t uart_comm_clear_error_stats(void)
{
    memset(&g_uart_comm.error_stats, 0, sizeof(uart_error_stats_t));
    ESP_LOGI(TAG, "错误统计已清除");
    return ESP_OK;
}

/**
 * @brief 设置接收回调函数
 * @param rx_callback 接收回调函数
 * @param user_data 用户数据指针
 * @retval ESP_OK 设置成功
 * @retval ESP_ERR_INVALID_ARG 参数错误
 */
esp_err_t uart_comm_set_rx_callback(uart_rx_callback_t rx_callback, void *user_data)
{
    if (rx_callback == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    g_rx_callback = rx_callback;
    g_user_data = user_data;

    ESP_LOGI(TAG, "接收回调函数已更新");
    return ESP_OK;
}