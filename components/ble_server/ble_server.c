#include "ble_server.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "motor_driver.h"
#include "adc_driver.h"
#include "freertos/queue.h"


/* NimBLE 核心头文件 */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/util/util.h"


extern QueueHandle_t motor_mailbox; // 电机控制消息队列


#define TAG "蓝牙"

/* =========================================================
 * 私有变量 (static 修饰，仅本文件可见)
 * ========================================================= */
static uint8_t own_addr_type;// 本设备的蓝牙地址类型
static uint8_t addr_val[6];// 存储本设备的蓝牙地址
static uint16_t motor_chr_val_handle;// 电机控制特征的句柄
static uint16_t batter_chr_val_handle;// 电池电量特征的句柄
static int ble_gap_event(struct ble_gap_event *event, void *arg);

/* Private functions */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

// 蓝牙写数据回调 (私有)，当手机写入数据时会调用这个函数
/* =========================================================
 * 1. GATT 写数据回调 (私有)
 * ========================================================= */
static int device_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 1. 只处理写入操作
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) 
    {
        return 0;
    }

    uint16_t total_len = OS_MBUF_PKTLEN(ctxt->om);
    if (total_len != 4) 
    {
        ESP_LOGW(TAG, "⚠️ 收到长度异常: %u (期望4字节)", total_len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t frame[4];
    os_mbuf_copydata(ctxt->om, 0, sizeof(frame), frame);

    if (frame[0] != 0xA5 || frame[3] != 0x5A) 
    {
        ESP_LOGW(TAG, "⚠️ 帧头/帧尾错误: 0x%02X ... 0x%02X", frame[0], frame[3]);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    int8_t left_raw = (int8_t)frame[1];
    int8_t right_raw = (int8_t)frame[2];

    // 保护性裁剪，确保在驱动允许的范围内
    int16_t left = left_raw;
    int16_t right = right_raw;
    if (left < -100) left = -100;
    if (left > 100) left = 100;
    if (right < -100) right = -100;
    if (right > 100) right = 100;

    dual_motor_msg_t msg = 
    {
        .left_power = left,
        .right_power = right,
    };
    xQueueOverwrite(motor_mailbox, &msg);  
    ESP_LOGI(TAG, "✅ 帧正确，左:%d 右:%d", left, right);
    return 0;
}

static int device_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 1. 只处理读取操作
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) 
    {
        return 0;
    }

    // 2. 获取电压和电流
    float v = adc_read_voltage();
    float i = adc_read_current();

    // 格式化为字符串 "V:数值,I:数值"
    char response_str[32];
    snprintf(response_str, sizeof(response_str), "V:%.2f,I:%.2f", v, i);

    // 3. 将字符串写入响应缓冲区
    os_mbuf_copyinto(ctxt->om, 0, response_str, strlen(response_str));

    ESP_LOGI(TAG, "✅ 电池电压读取: %.2fV, 电流读取: %.2fA", v, i);
    return 0;
}

/* =========================================================
 * 2. GATT 服务表定义 (私有)
 * ========================================================= */
static const struct ble_gatt_svc_def gatt_svcs[] = 
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // 主服务
        .uuid = BLE_UUID16_DECLARE(0xABCD), // 服务 UUID
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(0x1234), // 特征 UUID
                .access_cb = device_write_cb, // 写数据回调
                .flags = BLE_GATT_CHR_F_WRITE , // 允许写
                .val_handle = &motor_chr_val_handle, // 存储特征值句柄
            },
            {
                .uuid = BLE_UUID16_DECLARE(0x1235), // 特征 UUID
                .access_cb = device_read_cb, // 读数据回调
                .flags = BLE_GATT_CHR_F_READ  , // 允许读
                .val_handle = &batter_chr_val_handle, // 存储特征值句柄
            },
            {0}
        }
    },
    {0}
};



static void start_advertising(void) 
{
    /* Local variables */
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set device tx power */
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    /* Set device appearance */
    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    /* Set device LE role */
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set advertisement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set advertising interval */
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set undirected connectable and general discoverable mode */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}

// 蓝牙同步回调 (私有)，蓝牙准备好后会调用这个函数
static void ble_on_sync(void) 
{
    char addr_str[18] = {0};
    /* Make sure we have proper BT identity address set */
    ble_hs_util_ensure_addr(0);// 确保蓝牙地址已设置，如果没有，NimBLE 会自动生成一个随机地址并存储在非易失性存储中
    /* Figure out BT address to use while advertising */
    ble_hs_id_infer_auto(0, &own_addr_type);// 推断地址类型，通常是公共地址或随机地址
    /* Copy device address to addr_val */
    ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);// 将本设备的蓝牙地址复制到 addr_val 数组中，供后续使用

    format_addr(addr_str, addr_val);// 格式化地址为字符串形式，方便日志输出
    ESP_LOGI(TAG, "🔗 蓝牙已同步，地址: %s", addr_str);
    start_advertising();// 开启广播
}

// 蓝牙主任务 (私有)
static void ble_host_task(void *param) 
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* =========================================================
 * 安全管理回调 (处理配对/绑定事件)
 * ========================================================= */
static int ble_gap_event(struct ble_gap_event *event, void *arg) 
{
    switch (event->type) 
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) 
        {
            ESP_LOGI(TAG, "✅ 手机已连接");
        } else 
        {
            ESP_LOGE(TAG, "❌ 连接失败，重连中...");
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "📴 手机已断开，重新广播");
        start_advertising();
        break;

    }
    return 0;
}

/* =========================================================
 * 4. 对外公开的接口 (在 .h 中声明)
 * ========================================================= */
void ble_simple_start(void) 
{
    // 1. 初始化 NimBLE 端口
    nimble_port_init();
    // 2. 初始化 GAP 和 GATT 服务
    ble_svc_gap_init();   // 初始化标准 GAP 服务 (0x1800)
    
    // 3. 配置 GATT 表
    ble_svc_gap_device_name_set(DEVICE_NAME);// 设置设备名称
    /* 4. GATT service initialization */
    ble_svc_gatt_init();  // 初始化标准 GATT 服务 (0x1801)
    
    ble_gatts_count_cfg(gatt_svcs);// 计算服务数量
    ble_gatts_add_svcs(gatt_svcs);// 添加服务到 GATT 服务器

    // 5. 设置回调
    ble_hs_cfg.sync_cb = ble_on_sync;// 设置蓝牙同步回调，当蓝牙准备好后会调用这个函数

    // 6. 启动任务
    nimble_port_freertos_init(ble_host_task);
}
