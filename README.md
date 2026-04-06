# ESP32 BLE 电机控制从机说明

## 功能概览

- 使用 NimBLE 作为 BLE 从机，广播设备名 ESP32-S3-TEST，支持掉线后自动重连广播。
- 暴露 1 个自定义主服务 (UUID 0xABCD) 与 1 个特征 (UUID 0x1234)，特征强制加密写入。
- 接收手机下发的 4 字节电机控制帧，解析后写入电机消息队列，自动裁剪到 -100~100。
- 采用 Just Works 配对并启用 Bonding，重复配对会覆盖旧记录。
- 断开连接后自动恢复广播。

## 代码结构

- [main/main.c](main/main.c)：初始化 NVS、motor_driver、adc_driver，启动 BLE 从机并进入主循环日志。
- [components/ble_server/ble_server.c](components/ble_server/ble_server.c)：配置 NimBLE、服务/特征、配对策略；处理写回调并推送电机控制消息。
- [components/ble_server/ble_server.h](components/ble_server/ble_server.h)：对外暴露 `ble_simple_start()`。
- motor_driver/adc_driver：电机控制与 ADC 采样，提供 `motor_mailbox` 队列供 BLE 写回调投递。

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
- 写操作要求已配对加密，否则手机写入会失败。

## 使用方法

1) 环境：已安装 ESP-IDF（本项目目标芯片为 ESP32-S3）。
2) 首次可执行 `idf.py set-target esp32s3`，如需调整配置可 `idf.py menuconfig`。
3) 编译：`idf.py build`
4) 烧录并监视：`idf.py -p <PORT> flash monitor`
5) 仅查看日志：`idf.py monitor`

## 手机连接与测试步骤

- 打开手机蓝牙，使用 LightBlue（iOS）或 nRF Connect / BLE Scanner（Android）。
- 扫描并连接设备名称 ESP32-S3-TEST；首次连接按提示完成配对。
- 进入自定义主服务 UUID 0xABCD，找到特征 UUID 0x1234（写）。
- 写入示例帧：A5 32 32 5A（左右功率 +50）。断开后设备会重新广播。

## 常见问题与排查

- 无法写入：确认已配对并处于加密链路；写入长度必须是 4 字节。
- 电机不动：检查帧头/帧尾是否 A5/5A，功率是否在 -100~100 范围；查看串口日志是否收到 “帧正确” 记录。
- 重新配对：若手机删除配对记录，重新连接会自动覆盖旧绑定；若仍失败，可在手机端先忘记设备再连接。

## 后续可扩展方向

- 添加 Notify 特征回传电机状态/电池电量。
- 增加命令校验或 CRC，提升抗误写能力。
- 把设备名和 UUID 抽到配置项，便于量产修改。
- 优化广播/功耗策略，增加休眠唤醒逻辑。

## 功能介绍

这个项目实现了一个ESP32 BLE（低功耗蓝牙）服务器，支持：
- ✅ 蓝牙广告与连接管理
- ✅ GATT服务与特征定义
- ✅ 接收手机客户端的指令
- ✅ 向手机客户端发送消息
- ✅ 连接状态监测

## 文件说明

### ble_server.h
蓝牙服务器的头文件，定义了公共接口：
- `ble_server_init()` - 初始化BLE服务器
- `ble_send_data()` - 发送数据给客户端
- `ble_is_connected()` - 检查连接状态

### ble_server.c
蓝牙服务器的核心实现：
- GATT服务定义（UUID: 0x181A）
- 两个GATT特征：
  - RX特征 (0x2A99)：用于接收客户端的数据
  - TX特征 (0x2A9A)：用于发送数据给客户端
- GAP事件处理（连接、断开连接）
- NimBLE协议栈初始化

### main.c
主应用程序：
- 初始化BLE服务器
- 创建命令处理任务
- 定期向客户端发送测试消息

## 使用方式

### 1. 配置和编译
```bash
# 进行菜单配置（可选，已预配置）
idf.py menuconfig

# 编译项目
idf.py build
```

### 2. 烧录固件
```bash
idf.py flash monitor
```

### 3. 手机连接
- 打开手机蓝牙设置
- 搜索设备 "ESP32_BLE_Server"
- 连接设备
- 查看数据发送/接收（使用支持BLE的APP）

## 修改指令与消息处理

### 接收客户端的指令

编辑 `ble_server.c` 中的 `ble_char_rx_write_callback` 函数：

```c
static int
ble_char_rx_write_callback(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        memcpy(rx_buffer, ctxt->om->om_data, ctxt->om->om_len);
        rx_buffer_len = ctxt->om->om_len;
        
        ESP_LOGI(TAG, "接收数据: %s", rx_buffer);
        
        // 在这里添加你的指令处理逻辑
        // 例如：控制LED、PWM等
        if (strstr((char *)rx_buffer, "LED_ON")) {
            // 打开LED的代码
            ESP_LOGI(TAG, "执行: 打开LED");
        } else if (strstr((char *)rx_buffer, "LED_OFF")) {
            // 关闭LED的代码
            ESP_LOGI(TAG, "执行: 关闭LED");
        }
    }
    return 0;
}
```

### 发送自定义消息

在 `main.c` 或其他地方调用：

```c
uint8_t message[] = "ESP32: Hello Phone";
ble_send_data(message, strlen((char *)message));
```

## 常见问题

### Q: 如何修改设备名称？
A: 编辑 `ble_server.c` 第 162 行：
```c
ble_svc_gap_device_name_set("YOUR_DEVICE_NAME");
```

### Q: 如何添加更多GATT特征？
A: 在 `ble_server.c` 中的 `gatt_svcs` 数组中添加新的特征定义:
```c
{
    .uuid = BLE_UUID16_DECLARE(0x2A9B),  // 新特征UUID
    .access_cb = ble_gatt_access_callback,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
},
```

### Q: 如何增加传输速率？
A: 在 `ble_command_task` 中调整发送间隔：
```c
vTaskDelay(pdMS_TO_TICKS(1000));  // 改为1秒发送一次
```

## 推荐的手机APP

- **iOS**: LightBlue
- **Android**: nRF Connect 或 BLE Scanner

这些APP可以显示BLE设备，读写特征值，监测连接状态。

## 技术架构

```
┌─────────────────────────────────┐
│      手机蓝牙客户端              │
└────────────────┬────────────────┘
                 │ BLE
┌────────────────▼────────────────┐
│      ESP32 BLE 服务器            │
│  ┌──────────────────────────┐   │
│  │   NimBLE 协议栈          │   │
│  ├──────────────────────────┤   │
│  │   GAP (广告, 连接)       │   │
│  ├──────────────────────────┤   │
│  │   GATT (服务, 特征)      │   │
│  ├──────────────────────────┤   │
│  │   Command Handler        │   │
│  └──────────────────────────┘   │
└─────────────────────────────────┘
```

## 下一步改进

- [ ] 添加更多GATT特征以支持不同功能
- [ ] 实现安全配对机制
- [ ] 添加OTA固件升级功能
- [ ] 优化功耗管理
- [ ] 增加错误处理和恢复机制
