#ifndef __ALL_INCLUDE_H__
#define __ALL_INCLUDE_H__

#include "bsp_drivers.h" // 硬件驱动库
#include "all_debug.h"   // 调试配置和版本定义

/* ========== 标准库 ========== */
#include "sys/time.h"
#include "esp_log.h"
#include "esp_srp.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_psram.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "errno.h"
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <dirent.h>
#include "sdkconfig.h"

/* ========== RTOS ========== */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

/* ========== 应用程序头文件包含 ========== */
#include "bsp_drivers.h"
#include "app_audio.h"
#include "app_blue_control.h"
#include "gatts_server.h"
#include "app_pid_control.h"
#include "app_move_control.h"
#include "app_uart_control.h"

#endif // __ALL_INCLUDE_H__
