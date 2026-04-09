# ESP32 BLE 电机控制从机说明

## 功能概览

- 使用 NimBLE 作为 BLE 从机，广播设备名 `ESP32_BLE`，支持掉线后自动重连广播。
- 暴露 1 个自定义主服务 (UUID 0xABCD) 与 2 个特征：
  - UUID 0x1234（写）：接收电机控制指令。
  - UUID 0x1235（读）：返回当前电压与电流（格式 `V:x.xxx,I:x.xxx`）。
- 接收手机下发的 4 字节电机控制帧，解析后写入电机消息队列，自动裁剪到 -100~100。
- 断开连接后自动恢复广播。

## 代码结构

- [main/main.c](main/main.c)：初始化 NVS、motor_driver、adc_driver，启动 BLE 从机并进入主循环。
- [components/ble_server/ble_server.c](components/ble_server/ble_server.c)：配置 NimBLE、GATT 服务/特征；处理写回调并推送电机控制消息，处理读回调返回 ADC 数据。
- [components/ble_server/ble_server.h](components/ble_server/ble_server.h)：对外暴露 `ble_simple_start()`，以及设备名宏 `DEVICE_NAME`。
- [components/motor_driver/motor_driver.c](components/motor_driver/motor_driver.c)：LEDC PWM 电机驱动，4 路通道分别控制左右电机的正反转；提供 `motor_mailbox` 队列供 BLE 写回调投递。
- [components/adc_driver/adc_driver.c](components/adc_driver/adc_driver.c)：ADC 采样，提供 `adc_read_voltage()` 与 `adc_read_current()`。

## 指令协议（手机写特征 0x1234）

| 字节序 | 含义 | 取值 |
| --- | --- | --- |
| Byte0 | 帧头 | 固定 0xA5 |
| Byte1 | 左电机功率 | int8，期望 -100~100 |
| Byte2 | 右电机功率 | int8，期望 -100~100 |
| Byte3 | 帧尾 | 固定 0x5A |

规则：
- 仅接受长度恰好 4 字节的写入，长度错误返回 Invalid Attribute Value Length。
- 帧头/帧尾不符直接拒绝；合法帧被裁剪到 -100~100 后覆盖写入 `motor_mailbox`。
- 正值前进（LEDC_CHANNEL_0/1 输出 PWM），负值后退（LEDC_CHANNEL_2/3 输出 PWM），零停止。

## 使用方法

1) 环境：已安装 ESP-IDF（本项目目标芯片为 ESP32-S3）。
2) 首次可执行 `idf.py set-target esp32s3`，如需调整配置可 `idf.py menuconfig`。
3) 编译：`idf.py build`
4) 烧录并监视：`idf.py -p <PORT> flash monitor`
5) 仅查看日志：`idf.py monitor`

## 手机连接与测试步骤

- 打开手机蓝牙，使用 LightBlue（iOS）或 nRF Connect / BLE Scanner（Android）。
- 扫描并连接设备名称 `ESP32_BLE`。
- 进入自定义主服务 UUID 0xABCD：
  - 找到特征 UUID 0x1234（写），写入电机控制帧，例如 `A5 32 32 5A`（左右功率 +50）。
  - 找到特征 UUID 0x1235（读），读取当前电压与电流字符串。
- 断开后设备会重新广播。

## 文件说明

### ble_server.h
蓝牙服务器的头文件，定义了公共接口与宏：
- `DEVICE_NAME` — 广播设备名（默认 `"ESP32_BLE"`）
- `ble_simple_start()` — 初始化并启动 BLE 从机

### ble_server.c
蓝牙服务器的核心实现：
- GATT 服务定义（UUID: 0xABCD），包含两个特征：
  - 写特征 (0x1234)：`device_write_cb` 接收 4 字节电机帧并推送至 `motor_mailbox`
  - 读特征 (0x1235)：`device_read_cb` 通过 `adc_read_voltage()` / `adc_read_current()` 返回电压电流字符串
- GAP 事件处理（连接、断开并自动重新广播）
- NimBLE 协议栈初始化与主机任务

### motor_driver.c / motor_driver.h
电机驱动实现：
- 4 路 LEDC 通道（LEDC_CHANNEL_0~3）分别对应 GPIO 12/15（前进 PWM）和 GPIO 13/16（后退 PWM）
- `motor_init()` — 初始化 PWM 定时器、通道及 `motor_mailbox` 队列，启动 `motor_task`
- `motor_stop()` — 将全部通道占空比清零
- `motor_task` 监听 `motor_mailbox`，150 ms 无消息则自动停车；切换正反转前先停止再延时 100 ms
- 反转占空比计算：输入范围 -100~-1，公式 `(100 + power) * 1024 / 100` 将 power=-100 映射到占空比 0（停止）、power=-1 映射到约 1023（全速）；已修复此前反转无法调速的 bug

### adc_driver.c / adc_driver.h
ADC 采样：
- `adc_init()` — 初始化 ADC
- `adc_read_voltage()` — 读取电压（单位 V，返回 `float`）
- `adc_read_current()` — 读取电流（单位 A，返回 `float`）

### main.c
主应用程序：
- 依次初始化 NVS、`motor_driver`、`adc_driver`、BLE 从机
- 主循环模拟电机功率在 -100~100 之间步进变化，用于调试验证

## 常见问题与排查

- 无法写入：写入长度必须是 4 字节，且帧头/帧尾必须为 0xA5/0x5A。
- 电机不动：检查串口日志是否收到 "✅ 帧正确" 记录；确认功率值不为 0。
- 反转后无法调速：已在 `motor_back()` 中修复，反转占空比计算公式改为 `(100 + power) * 1024 / 100`。
- 设备名修改：编辑 `components/ble_server/ble_server.h` 中的 `DEVICE_NAME` 宏。

## 推荐的手机 APP

- **iOS**: LightBlue
- **Android**: nRF Connect 或 BLE Scanner

## 技术架构

```
┌─────────────────────────────────┐
│      手机蓝牙客户端              │
└────────────────┬────────────────┘
                 │ BLE (NimBLE)
┌────────────────▼────────────────┐
│      ESP32-S3 BLE 服务器         │
│  ┌──────────────────────────┐   │
│  │   GAP (广告 / 连接管理)  │   │
│  ├──────────────────────────┤   │
│  │   GATT 主服务 0xABCD     │   │
│  │   ├ 写特征 0x1234        │   │
│  │   └ 读特征 0x1235        │   │
│  ├──────────────────────────┤   │
│  │   motor_driver (LEDC)    │   │
│  ├──────────────────────────┤   │
│  │   adc_driver             │   │
│  └──────────────────────────┘   │
└─────────────────────────────────┘
```

## 后续可扩展方向

- 增加命令校验或 CRC，提升抗误写能力。
- 添加 Notify 特征主动推送电机状态 / 电池电量。
- 把设备名和 UUID 抽到 `menuconfig` 配置项，便于量产修改。
- 实现安全配对（Just Works / Passkey），保护写特征访问。
- 优化广播/功耗策略，增加休眠唤醒逻辑。
- 添加 OTA 固件升级功能。
