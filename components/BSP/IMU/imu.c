#include "imu.h"

static const char *TAG = "QMI8658A";
static uint8_t g_qmi8658_addr = QMI8658_SENSOR_ADDR; // 允许初始化过程中回退到备选地址
static esp_err_t qmi8658_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t qmi8658_register_write_byte(uint8_t reg_addr, uint8_t data);
static void qmi8658_Read_AccAndGry(t_sQMI8658_raw *p);
static void imu_read_data(t_sQMI8658 *p, t_sQMI8658_raw *p_raw);
static void qmi8658_init(void);

/************外部接口函数 ************/
void imu_data_resolution(t_sQMI8658 *p)
{
    t_sQMI8658_raw p_raw;
    memset(&p_raw, 0, sizeof(t_sQMI8658_raw));
    // 读取IMU数据
    imu_read_data(p, &p_raw);
}

/************内部函数 ************/

// 读取QMI8658寄存器的值
static esp_err_t qmi8658_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BSP_I2C_NUM, g_qmi8658_addr, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

// 给QMI8658的寄存器写值
static esp_err_t qmi8658_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(BSP_I2C_NUM, g_qmi8658_addr, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
}

void imu_init(void)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_FREQ_HZ};
    i2c_param_config(BSP_I2C_NUM, &i2c_conf);

    i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0);
    qmi8658_init();
    t_sQMI8658 imu_data;
    memset(&imu_data, 0, sizeof(t_sQMI8658));
    imu_data_resolution(&imu_data);
    ESP_LOGI(TAG, "pitch:%d, yaw:%d, roll:%d", imu_data.pitch, imu_data.yaw, imu_data.roll);
}

static void qmi8658_init(void)
{
    uint8_t id = 0;                                  // 芯片的ID号
    qmi8658_register_read(QMI8658_WHO_AM_I, &id, 1); // 读芯片的ID号
    ESP_LOGI(TAG, "QMI8658 ID: %d", id);             // 打印ID号
    while (id != 0x05)                               // 判断读到的ID号是否是0x05
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);           // 延时1秒
        qmi8658_register_read(QMI8658_WHO_AM_I, &id, 1); // 读取ID号
        ESP_LOGI(TAG, "QMI8658 ID: %d", id);             // 打印ID号
    }
    ESP_LOGI(TAG, "QMI8658 OK!");                     // 打印信息
    qmi8658_register_write_byte(QMI8658_RESET, 0xb0); // 复位
    vTaskDelay(10 / portTICK_PERIOD_MS);              // 延时10ms
    qmi8658_register_write_byte(QMI8658_CTRL1, 0x40); // CTRL1 设置地址自动增加
    qmi8658_register_write_byte(QMI8658_CTRL7, 0x03); // CTRL7 允许加速度和陀螺仪
    qmi8658_register_write_byte(QMI8658_CTRL2, 0x95); // CTRL2 设置ACC 4g 250Hz
    qmi8658_register_write_byte(QMI8658_CTRL3, 0xd5); // CTRL3 设置GRY 512dps 250Hz
    qmi8658_register_write_byte(QMI8658_CTRL5, 0x11); // CTRL5 开启低通滤波
}

// 读取加速度和陀螺仪寄存器值
static void qmi8658_Read_AccAndGry(t_sQMI8658_raw *p)
{
    uint8_t status, data_ready = 0;
    int16_t buf[6];

    qmi8658_register_read(QMI8658_STATUS0, &status, 1); // 读状态寄存器
    if (status & 0x03)                                  // 判断加速度和陀螺仪数据是否可读
        data_ready = 1;
    if (data_ready == 1)
    { // 如果数据可读
        data_ready = 0;
        qmi8658_register_read(QMI8658_AX_L, (uint8_t *)buf, 12); // 读加速度和陀螺仪值
        p->acc_x_raw = buf[0];
        p->acc_y_raw = buf[1];
        p->acc_z_raw = buf[2];
        p->gyr_x_raw = buf[3];
        p->gyr_y_raw = buf[4];
        p->gyr_z_raw = buf[5];
    }
}

// 获取XYZ轴的倾角值
static void imu_read_data(t_sQMI8658 *p, t_sQMI8658_raw *p_raw)
{
    float temp;

    qmi8658_Read_AccAndGry(p_raw); // 读取加速度和陀螺仪的寄存器值

    // 根据寄存器值 计算倾角值 并把弧度转换成角度
    // 使用ESP-DSP优化的sqrt函数提升性能
    float y2_z2 = (float)p_raw->acc_y_raw * (float)p_raw->acc_y_raw + (float)p_raw->acc_z_raw * (float)p_raw->acc_z_raw;
    float sqrt_y2_z2 = dsps_sqrtf_f32_ansi(y2_z2); // ESP-DSP优化的sqrt函数
    temp = (float)p_raw->acc_x_raw / sqrt_y2_z2;
    p->pitch = atan(temp) * 57.29578f; // 使用快速atan函数，180/π=57.29578

    float x2_z2 = (float)p_raw->acc_x_raw * (float)p_raw->acc_x_raw + (float)p_raw->acc_z_raw * (float)p_raw->acc_z_raw;
    float sqrt_x2_z2 = dsps_sqrtf_f32_ansi(x2_z2); // ESP-DSP优化的sqrt函数
    temp = (float)p_raw->acc_y_raw / sqrt_x2_z2;
    p->roll = atan(temp) * 57.29578f; // 使用快速atan函数，180/π=57.29578

    float x2_y2 = (float)p_raw->acc_x_raw * (float)p_raw->acc_x_raw + (float)p_raw->acc_y_raw * (float)p_raw->acc_y_raw;
    float sqrt_x2_y2 = dsps_sqrtf_f32_ansi(x2_y2); // ESP-DSP优化的sqrt函数
    temp = sqrt_x2_y2 / (float)p_raw->acc_z_raw;
    p->yaw = atan(temp) * 57.29578f; // 使用快速atan函数，180/π=57.29578

    // 原始数据转换为物理量
    // 计算acc_x,acc_y,acc_z
    p->acc_x = p_raw->acc_x_raw * ACC_SCALE / ADC_RESOLUTION;
    p->acc_y = p_raw->acc_y_raw * ACC_SCALE / ADC_RESOLUTION;
    p->acc_z = p_raw->acc_z_raw * ACC_SCALE / ADC_RESOLUTION;

    // 计算gyr_x,gyr_y,gyr_z
    p->gyr_x = p_raw->gyr_x_raw * GYRO_SCALE / ADC_RESOLUTION;
    p->gyr_y = p_raw->gyr_y_raw * GYRO_SCALE / ADC_RESOLUTION;
    p->gyr_z = p_raw->gyr_z_raw * GYRO_SCALE / ADC_RESOLUTION;
}