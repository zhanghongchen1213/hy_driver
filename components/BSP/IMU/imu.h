#ifndef __IMU_H__
#define __IMU_H__

#pragma once

#include <stdio.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "string.h"
#include "esp_timer.h"
#include "esp_dsp.h"

#define BSP_I2C_SDA GPIO_NUM_1
#define BSP_I2C_SCL GPIO_NUM_2
#define BSP_I2C_NUM (0)
#define BSP_I2C_FREQ_HZ 400000
#define QMI8658_SENSOR_ADDR 0x6A // 8位地址模式
#define ACC_SCALE 4.0f           // ±4g
#define GYRO_SCALE 512.0f        // ±512dps
#define ADC_RESOLUTION 32768.0f

#ifdef _BV
#undef _BV
#endif
#define _BV(b) (1UL << (uint32_t)(b))

#ifndef lowByte
#define lowByte(w) ((uint8_t)((w) & 0xff))
#endif

#ifndef highByte
#define highByte(w) ((uint8_t)((w) >> 8))
#endif

// QMI8658寄存器地址
enum qmi8658_reg
{
    QMI8658_WHO_AM_I,
    QMI8658_REVISION_ID,
    QMI8658_CTRL1,
    QMI8658_CTRL2,
    QMI8658_CTRL3,
    QMI8658_CTRL4,
    QMI8658_CTRL5,
    QMI8658_CTRL6,
    QMI8658_CTRL7,
    QMI8658_CTRL8,
    QMI8658_CTRL9,
    QMI8658_CATL1_L,
    QMI8658_CATL1_H,
    QMI8658_CATL2_L,
    QMI8658_CATL2_H,
    QMI8658_CATL3_L,
    QMI8658_CATL3_H,
    QMI8658_CATL4_L,
    QMI8658_CATL4_H,
    QMI8658_FIFO_WTM_TH,
    QMI8658_FIFO_CTRL,
    QMI8658_FIFO_SMPL_CNT,
    QMI8658_FIFO_STATUS,
    QMI8658_FIFO_DATA,
    QMI8658_STATUSINT = 45,
    QMI8658_STATUS0,
    QMI8658_STATUS1,
    QMI8658_TIMESTAMP_LOW,
    QMI8658_TIMESTAMP_MID,
    QMI8658_TIMESTAMP_HIGH,
    QMI8658_TEMP_L,
    QMI8658_TEMP_H,
    QMI8658_AX_L,
    QMI8658_AX_H,
    QMI8658_AY_L,
    QMI8658_AY_H,
    QMI8658_AZ_L,
    QMI8658_AZ_H,
    QMI8658_GX_L,
    QMI8658_GX_H,
    QMI8658_GY_L,
    QMI8658_GY_H,
    QMI8658_GZ_L,
    QMI8658_GZ_H,
    QMI8658_COD_STATUS = 70,
    QMI8658_dQW_L = 73,
    QMI8658_dQW_H,
    QMI8658_dQX_L,
    QMI8658_dQX_H,
    QMI8658_dQY_L,
    QMI8658_dQY_H,
    QMI8658_dQZ_L,
    QMI8658_dQZ_H,
    QMI8658_dVX_L,
    QMI8658_dVX_H,
    QMI8658_dVY_L,
    QMI8658_dVY_H,
    QMI8658_dVZ_L,
    QMI8658_dVZ_H,
    QMI8658_TAP_STATUS = 89,
    QMI8658_STEP_CNT_LOW,
    QMI8658_STEP_CNT_MIDL,
    QMI8658_STEP_CNT_HIGH,
    QMI8658_RESET = 96
};

enum CommandTable
{
    CTRL_CMD_ACK = 0x00,
    CTRL_CMD_RST_FIFO = 0x04,
    CTRL_CMD_REQ_FIFO = 0x05,
    CTRL_CMD_WRITE_WOM_SETTING = 0x08,
    CTRL_CMD_ACCEL_HOST_DELTA_OFFSET = 0x09,
    CTRL_CMD_GYRO_HOST_DELTA_OFFSET = 0x0A,
    CTRL_CMD_CONFIGURE_TAP = 0x0C,
    CTRL_CMD_CONFIGURE_PEDOMETER = 0x0D,
    CTRL_CMD_CONFIGURE_MOTION = 0x0E,
    CTRL_CMD_RESET_PEDOMETER = 0x0F,
    CTRL_CMD_COPY_USID = 0x10,
    CTRL_CMD_SET_RPU = 0x11,
    CTRL_CMD_AHB_CLOCK_GATING = 0x12,
    CTRL_CMD_ON_DEMAND_CALIBRATION = 0xA2,
    CTRL_CMD_APPLY_GYRO_GAINS = 0xAA,
};

// 倾角结构体
typedef struct
{
    // 计算数据
    float acc_x; // 前后轴
    float acc_y; // 左右轴
    float acc_z; // 上下轴
    float gyr_x; // 前后轴
    float gyr_y; // 左右轴
    float gyr_z; // 上下轴
    float pitch; // 俯仰角
    float roll;  // 横滚角
    float yaw;   // 偏航角
} t_sQMI8658;

// 倾角结构体原始值
typedef struct
{
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;
    int16_t gyr_x_raw;
    int16_t gyr_y_raw;
    int16_t gyr_z_raw;
} t_sQMI8658_raw;

void imu_init(void);
void imu_data_resolution(t_sQMI8658 *p); // 数据采集
#endif
