# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作时提供指导。

## ⚠️ 重要规则：任务管理

**强制要求**：
- 当 Claude Code 处于 **Plan Mode（计划模式）** 时，所有最新的实施计划必须保存到 `.claude/todo.md` 文件中
- 计划必须以**待办清单格式**组织，包含清晰的任务项和状态标记
- 每完成一项任务后，必须立即更新 `todo.md` 文件中该任务的状态
- 任务状态标记：
  - `[ ]` - 待完成
  - `[进行中]` - 正在进行
  - `[x]` - 已完成
- 这确保了任务进度的可追踪性和透明度

**示例格式**：
```markdown
# 当前任务清单

## 添加 IMU 代码注释
- [ ] 阅读 imu.h 和 imu.c 文件
- [进行中] 为 imu.h 添加函数注释
- [x] 分析 IMU 初始化流程
```

## 项目概述

这是一个用于智能移动机器人平台的 **ESP32-S3 嵌入式机器人固件**。项目使用 ESP-IDF v5.x 和 FreeRTOS，主要使用 C 语言编写。

**项目名称**: `main_control` (hy_driver)
**目标平台**: ESP32-S3 微控制器
**开发框架**: ESP-IDF v5.x
**主要语言**: C 语言（部分 C++ 组件）

## 构建命令

### 基本构建操作
```bash
# 配置项目（首次设置）
idf.py set-target esp32s3

# 构建项目
idf.py build

# 烧录到设备
idf.py flash

# 监控串口输出
idf.py monitor

# 一条命令完成构建、烧录和监控
idf.py build flash monitor

# 清理构建产物
idf.py fullclean
```

### 配置管理
```bash
# 打开 menuconfig 进行项目配置
idf.py menuconfig

# 重新配置项目
idf.py reconfigure
```

### 分区管理
```bash
# 完全擦除 flash
idf.py erase-flash

# 仅烧录应用程序（开发时更快）
idf.py app-flash
```

## 架构概述

### 系统架构

固件采用**分层架构**：

```
应用层 (main/)
    ↓
硬件抽象层 (components/BSP/)
    ↓
ESP-IDF & FreeRTOS 内核
```

### 核心组件

1. **运动控制系统** ([main/app_move_control/](main/app_move_control/))
   - 正向和逆向运动学计算
   - 使用编码器 + IMU 融合的里程计
   - 位置跟踪（x, y, 角度, 速度）
   - 机械参数：轮径 60mm，轮距 125mm

2. **PID 电机控制** ([main/app_pid_control/](main/app_pid_control/))
   - 双路独立 PID 控制（左右电机）
   - RPM 速度控制（标称 34 RPM）
   - 抗积分饱和、死区、输出限幅

3. **通信系统**
   - **UART 协议** ([main/app_uart_control/](main/app_uart_control/))
     - UART1: 460800 波特率（外部大脑/AI 模块）
     - UART2: 115200 波特率（CI_03T 语音识别）
     - 结构化数据包格式：0xAA（起始）+ 数据 + 0x55/0x66（结束）
   - **蓝牙 BLE** ([main/app_blue_control/](main/app_blue_control/))
     - GATT 服务器用于无线控制
     - 线速度/角速度命令

4. **音频系统** ([main/app_audio_control/](main/app_audio_control/))
   - 支持 HTTP 客户端的 MP3 播放器
   - SPIFFS 文件系统存储音频文件
   - 音频编解码器和音量控制

5. **硬件驱动** ([components/BSP/](components/BSP/))
   - **电机**: TB6612FNG 驱动双 N20 电机（2654.1 PPR 编码器）
   - **IMU**: QMI8658 六轴传感器（I2C, 400kHz）
   - **舵机**: 3 个舵机（右臂、左臂、腰部）
   - **UART**: 基于 DMA 的通信和事件队列

### 系统初始化流程

来自 [main/main.c](main/main.c)：
```
app_main()
  ├─ NVS Flash 初始化
  ├─ WiFi 初始化
  ├─ IMU 初始化
  ├─ 音频硬件设置（编解码器、MP3 播放器）
  ├─ 播放开机音效
  ├─ 硬件初始化（UART、电机、舵机）
  ├─ 蓝牙初始化（如果启用 BLUE_CONTROL）
  └─ 任务初始化：
      ├─ 音频控制任务
      ├─ UART 通信任务
      ├─ PID 控制任务
      ├─ 运动控制任务（如果启用 UPDATE_ODOMETRY_DEBUG）
      └─ 蓝牙控制任务（如果启用 BLUE_CONTROL）
```

## 调试配置系统

全局调试标志在 [main/all_debug/all_debug.h](main/all_debug/all_debug.h) 中控制：

- `NVS_CLEAR_VERSION`: 启动时强制擦除 NVS 分区（1=调试，0=发布）
- `BLUE_CONTROL`: 启用/禁用蓝牙功能
- `UPDATE_ODOMETRY_DEBUG`: 启用里程计和运动控制
- `PID_DEBUG`: 启用 PID 调试输出
- `SERVO_DEBUG`: 启用舵机控制

**启用/禁用功能**: 编辑 `all_debug.h` 中的宏定义并重新构建。

## 关键文件参考

| 文件 | 用途 |
|------|---------|
| [main/main.c](main/main.c) | 系统初始化入口点 |
| [main/all_debug/all_debug.h](main/all_debug/all_debug.h) | 全局功能标志 |
| [main/app_move_control/app_move_control.h](main/app_move_control/app_move_control.h) | 运动学和里程计 API |
| [main/app_pid_control/app_pid_control.h](main/app_pid_control/app_pid_control.h) | 电机速度控制 API |
| [main/app_uart_control/app_uart_control.h](main/app_uart_control/app_uart_control.h) | 通信协议定义 |
| [components/BSP/Motor/motor.h](components/BSP/Motor/motor.h) | 电机和编码器驱动 API |
| [components/BSP/IMU/imu.h](components/BSP/IMU/imu.h) | IMU 传感器驱动 API |

## 硬件配置

**运动系统**:
- 2x N20 直流电机带编码器（2654.1 PPR 输出）
- TB6612FNG 电机驱动器
- 轮径：60mm，轮距：125mm

**传感器**:
- QMI8658 六轴 IMU（加速度计 + 陀螺仪）
- 双电机编码器用于里程计

**执行器**:
- 3x 舵机（右臂、左臂、腰部）
- 音频编解码器和扬声器

**通信**:
- UART1: 460800 波特率（连接外部大脑/AI 模块）
- UART2: 115200 波特率（CI_03T 语音识别模块）
- WiFi 连接
- 蓝牙 BLE 接口

**存储**:
- SPIFFS 文件系统存储 MP3 音频文件
- NVS（非易失性存储）存储配置

## 编码规范

本项目遵循严格的嵌入式 C 编码标准，详细文档见 [.trae/rules/project_rules.md](.trae/rules/project_rules.md)。关键要点：

### 命名规范
- 文件：小写字母加下划线（`sensor_driver.c`）
- 函数：小写字母加下划线（`gpio_init_pin`）
- 全局变量：`g_` 前缀（`g_system_state`）
- 静态变量：`s_` 前缀（`s_counter`）
- 常量/宏：全大写加下划线（`MAX_BUFFER_SIZE`）
- 结构体：`_t` 后缀（`device_config_t`）
- 指针：`p_` 前缀（`p_buffer`）

### 代码风格
- 缩进：4 个空格
- 大括号：独占一行
- 行长度：最多 80 字符
- 函数长度：最多 50 行
- 嵌套深度：最多 3 层

### ESP32-S3 特定要求
- ISR 函数必须使用 `IRAM_ATTR` 属性
- 频繁调用的函数应考虑使用 `IRAM_ATTR`
- 任务栈大小：建议最小 4KB
- 使用 `CONFIG_` 宏进行条件编译
- 日志消息应使用中文文本配合 `ESP_LOGI`、`ESP_LOGW`、`ESP_LOGE`

### 内存管理
- 优先使用栈分配而非堆分配
- 静态分配优于动态分配
- 使用 `heap_caps_malloc()` 指定内存类型（IRAM/DRAM/PSRAM）
- 始终配对使用 `malloc()` 和 `free()`
- 禁止在 ISR 上下文中分配内存

### 安全要求
- 所有全局变量必须初始化
- 所有系统调用必须检查返回值
- 资源分配/释放必须配对
- 中断必须正确清除标志
- 关键数据必须验证
- 任务栈大小必须正确配置

## 文档规范

使用 Doxygen 风格注释：

```c
/**
 * @brief  函数简要说明
 * @param  参数说明
 * @retval 返回值说明
 */
```

文件头应包含：
```c
/// @module 模块名称
/// @brief 模块功能描述
/// @warning 重要注意事项和限制条件
/// @note 使用说明和特殊要求
/// @author ZHC
/// @date 2025
/// @version 版本号
```

## 开发工作流程

1. **修改前**: 阅读现有代码以理解模式
2. **功能标志**: 使用 `all_debug.h` 在开发期间启用/禁用功能
3. **渐进式开发**: 小步提交，确保每次都能编译成功
4. **测试**: 关键功能必须有测试覆盖
5. **日志**: 使用适当的 ESP_LOG 级别（ERROR > WARN > INFO > DEBUG > VERBOSE）
6. **性能监控**: 使用 `esp_timer_get_time()` 计时，使用 `heap_caps_get_free_size()` 监控内存

## 常见开发模式

### 添加新硬件驱动
1. 在 `components/BSP/[DriverName]/` 中创建驱动
2. 实现 init、read、write、deinit 函数
3. 添加到 [main/main.c](main/main.c) 的 hardware_init() 中
4. 如果是可选功能，在 [main/all_debug/all_debug.h](main/all_debug/all_debug.h) 中添加调试标志

### 添加新控制任务
1. 在 `main/app_[module_name]/` 中创建模块
2. 实现具有适当优先级的任务函数
3. 在硬件设置后在 app_main() 中初始化
4. 使用 FreeRTOS 队列/信号量进行任务间通信

### 修改通信协议
1. 更新 [main/app_uart_control/app_uart_control.h](main/app_uart_control/app_uart_control.h) 中的数据包结构
2. 修改上行/下行数据包处理程序
3. 确保数据包验证（起始/结束标志、校验和）
4. 与外部通信伙伴进行测试


### 运行时问题
- 检查 NVS 分区：设置 `NVS_CLEAR_VERSION=1` 强制重置
- 监控日志：使用 `idf.py monitor` 并设置适当的日志级别
- 检查任务栈使用：`uxTaskGetStackHighWaterMark()`
- 监控堆内存：`heap_caps_get_free_size(MALLOC_CAP_8BIT)`

## 项目特定说明

- **里程计系统**: 最近重构，改进了单位系统和运动学计算
- **PID 调优**: 默认参数在 [main/app_pid_control/](main/app_pid_control/) 中，根据您的机械设置进行调整
- **语音识别**: CI_03T 模块通过 UART2 集成
- **音频播放**: 支持本地 SPIFFS 文件和 HTTPS 流媒体
- **通信协议**: 外部 AI/大脑模块通过 UART1 以 460800 波特率通信

## 语言说明

本项目使用**中文**：
- 日志消息（ESP_LOGI、ESP_LOGW、ESP_LOGE 输出）
- 代码注释
- project_rules.md 中的文档

在使用此代码库时，请与日志和注释中现有的中文文本保持一致。
