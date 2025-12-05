#ifndef __GATTS_SERVER_H__
#define __GATTS_SERVER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

// Profile A 的GATT服务相关定义
#define GATTS_SERVICE_UUID_TEST_A 0x00FF // Profile A 的服务UUID
#define GATTS_CHAR_UUID_TEST_A 0xFF01    // Profile A 的特征值UUID
#define GATTS_DESCR_UUID_TEST_A 0x3333   // Profile A 的描述符UUID
#define GATTS_NUM_HANDLE_TEST_A 4        // Profile A 的句柄数量

// Profile B 的GATT服务相关定义
#define GATTS_SERVICE_UUID_TEST_B 0x00EE // Profile B 的服务UUID
#define GATTS_CHAR_UUID_TEST_B 0xEE01    // Profile B 的特征值UUID
#define GATTS_DESCR_UUID_TEST_B 0x2222   // Profile B 的描述符UUID
#define GATTS_NUM_HANDLE_TEST_B 4        // Profile B 的句柄数量

// 设备名称和厂商数据长度定义
#define TEST_DEVICE_NAME "HY_ESP32S3" // BLE设备名称
#define TEST_MANUFACTURER_DATA_LEN 17 // 厂商数据长度

// GATT特征值最大长度
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x190 // 特征值最大长度为400字节

// 准备写入缓冲区最大大小
#define PREPARE_BUF_MAX_SIZE 1024 // 准备写入缓冲区最大1024字节

/**
 * @brief 广播数据配置标志位
 * @note 用于标记广播数据配置状态
 */
#define adv_config_flag (1 << 0)

/**
 * @brief 扫描响应数据配置标志位
 * @note 用于标记扫描响应数据配置状态
 */
#define scan_rsp_config_flag (1 << 1)

#define PROFILE_NUM 2      // Profile数量
#define PROFILE_A_APP_ID 0 // Profile A的应用ID
#define PROFILE_B_APP_ID 1 // Profile B的应用ID

/**
 * @brief 准备写入环境结构体
 * @note 用于存储准备写入的数据缓冲区和长度
 */
typedef struct
{
    uint8_t *prepare_buf; // 准备写入的数据缓冲区指针
    int prepare_len;      // 准备写入的数据长度
} prepare_type_env_t;

#define RECEIVED_DATA_MAX_LEN GATTS_DEMO_CHAR_VAL_LEN_MAX
typedef struct
{
    uint8_t data[RECEIVED_DATA_MAX_LEN];
    size_t len;
} received_data_t;

/**
 * @brief GATT Profile实例结构体
 * @note 每个GATT-based profile对应一个app_id和一个gatts_if
 */
struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;       // GATT服务器回调函数
    uint16_t gatts_if;             // GATT接口
    uint16_t app_id;               // 应用ID
    uint16_t conn_id;              // 连接ID
    uint16_t service_handle;       // 服务句柄
    esp_gatt_srvc_id_t service_id; // 服务ID
    uint16_t char_handle;          // 特征值句柄
    esp_bt_uuid_t char_uuid;       // 特征值UUID
    esp_gatt_perm_t perm;          // 权限
    esp_gatt_char_prop_t property; // 属性
    uint16_t descr_handle;         // 描述符句柄
    esp_bt_uuid_t descr_uuid;      // 描述符UUID
};

#define DISCONNECTED_EVENT (1 << 0)
extern struct gatts_profile_inst gl_profile_tab[PROFILE_NUM];
extern EventGroupHandle_t connected_event_group;

void blue_init(void);

#endif
