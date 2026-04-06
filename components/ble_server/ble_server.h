#ifndef _BLE_SERVER_H_
#define _BLE_SERVER_H_

#define DEVICE_NAME "ESP32_BLE" // 设备名称
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

/**
 * @brief 初始化并启动简易 BLE 从机
 * 功能：广播、等待连接、接收手机写数据
 */
void ble_simple_start(void);

#endif // _BLE_SERVER_H_
