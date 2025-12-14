/**
 * @file connect_wifi.c
 * @brief Wi-Fi 连接管理模块实现文件
 *
 * 实现 Wi-Fi Station 模式的初始化、连接、事件处理及重连机制。
 * 使用 FreeRTOS EventGroup 进行任务间的状态同步。
 *
 * @author ZHC
 * @date 2025
 * @version 1.0
 */

#include "connect_wifi.h"

/* ========================= 配置宏定义 ========================= */

/** @brief 目标 Wi-Fi SSID */
#define EXAMPLE_ESP_WIFI_SSID "ZHC_Web"
/** @brief 目标 Wi-Fi 密码 */
#define EXAMPLE_ESP_WIFI_PASS "580231580231"
/** @brief 最大连接重试次数 */
#define EXAMPLE_ESP_MAXIMUM_RETRY 5

/* 根据 Kconfig 配置选择 Wi-Fi 认证模式阈值 */
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#else
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#endif

/* ========================= 全局变量与事件组 ========================= */

/** @brief FreeRTOS 事件组句柄，用于通知连接状态 */
static EventGroupHandle_t s_wifi_event_group;

/** @brief 事件位：已连接并获取到 IP */
#define WIFI_CONNECTED_BIT BIT0
/** @brief 事件位：连接失败（达到最大重试次数） */
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi station";

/** @brief 当前重试次数计数器 */
static int s_retry_num = 0;

/* ========================= 函数实现 ========================= */

/**
 * @brief Wi-Fi 事件处理回调函数
 *
 * 处理 Wi-Fi 启动、断开连接和 IP 获取事件。
 *
 * @param arg 用户参数（未使用）
 * @param event_base 事件基类
 * @param event_id 事件 ID
 * @param event_data 事件数据
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    // 处理 Wi-Fi 启动事件
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // 开始连接 AP
    }
    // 处理 Wi-Fi 断开连接事件
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect(); // 尝试重连
            s_retry_num++;
            ESP_LOGI(TAG, "正在重试连接 AP (尝试 %d/%d)", s_retry_num, EXAMPLE_ESP_MAXIMUM_RETRY);
        }
        else
        {
            // 超过最大重试次数，设置失败标志位
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "连接 AP 失败");
    }
    // 处理获取 IP 地址事件
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "获取到 IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0; // 重置重试计数
        // 设置连接成功标志位
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief 初始化 Wi-Fi Station 模式并连接到 AP
 *
 * 1. 初始化 NVS（在 main.c 中通常已完成，但此模块依赖 NVS 存储配置）
 * 2. 创建事件组
 * 3. 初始化网络接口和事件循环
 * 4. 配置 Wi-Fi 参数（SSID, Password 等）
 * 5. 启动 Wi-Fi 并等待连接结果
 */
void wifi_init_sta(void)
{
    // 创建事件组
    s_wifi_event_group = xEventGroupCreate();

    // 初始化 TCP/IP 堆栈
    ESP_ERROR_CHECK(esp_netif_init());

    // 创建默认系统事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 创建默认 Wi-Fi Station 网络接口
    esp_netif_create_default_wifi_sta();

    // 初始化 Wi-Fi 驱动，使用默认配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件处理程序
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    // 注册所有 Wi-Fi 事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    // 注册获取 IP 事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // 配置 Wi-Fi 连接参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* 如果密码匹配 WPA2 标准（长度>=8），自动模式阈值默认为 WPA2。
             * 如果需要连接 WEP/WPA 网络，请手动设置阈值。
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH, // 支持 WPA3 SAE PWE
        },
    };

    // 设置为 Station 模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    // 应用配置
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    // 启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi 初始化完成，后台开始连接...");
}

bool wifi_wait_connected(uint32_t wait_ms)
{
    if (s_wifi_event_group == NULL)
    {
        return false;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           (wait_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(wait_ms));

    return (bits & WIFI_CONNECTED_BIT) != 0;
}
