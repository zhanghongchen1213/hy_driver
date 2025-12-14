/**
 * @file connect_wifi.h
 * @brief Wi-Fi 连接管理模块头文件
 *
 * 本模块负责处理 ESP32 的 Wi-Fi Station 模式初始化及连接逻辑。
 * 包含事件处理、重连机制和连接状态同步。
 *
 * @author ZHC
 * @date 2025
 * @version 1.0
 */

#ifndef __CONNECT_WIFI_H__
#define __CONNECT_WIFI_H__

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 初始化 Wi-Fi Station 模式并启动后台连接
     *
     * 本函数执行以下操作：
     * 1. 初始化 TCP/IP 适配器
     * 2. 创建默认事件循环
     * 3. 配置 Wi-Fi 为 Station 模式
     * 4. 注册事件处理程序以处理连接、断开和 IP 获取事件
     * 5. 启动 Wi-Fi
     *
     * @note 此函数为非阻塞，Wi-Fi 连接过程在后台进行。
     *       如需等待连接成功，请调用 wifi_wait_connected()。
     */
    void wifi_init_sta(void);

    /**
     * @brief 等待 Wi-Fi 连接成功
     *
     * @param wait_ms 等待超时时间 (毫秒)，使用 portMAX_DELAY 表示无限等待
     * @return true 连接成功
     * @return false 连接超时或失败
     */
    bool wifi_wait_connected(uint32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif /* __CONNECT_WIFI_H__ */
