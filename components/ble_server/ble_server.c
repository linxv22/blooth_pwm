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
/*ota 核心头文件*/
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_partition.h"

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
//ota 相关变量
static esp_ota_handle_t ota_handle = 0;              // OTA句柄
static const esp_partition_t *update_partition = NULL; // 准备写入的目标分区
static bool is_ota_updating = false;                 // OTA 状态标志位
static uint32_t total_received = 0;                  // 已累计接收的字节数

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

// 蓝牙读数据回调 电池电量特征被读取时会调用这个函数
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
    snprintf(response_str, sizeof(response_str), "V:%.3f,I:%.3f", v, i);

    // 3. 将字符串写入响应缓冲区
    os_mbuf_append(ctxt->om,response_str, strlen(response_str));
    return 0;
}

// OTA 控制特征写数据回调 (专门处理 START 0x01 和 END 0x02)
static int ota_ctrl_cb(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return 0;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd_buf[16];
    os_mbuf_copydata(ctxt->om, 0, len > sizeof(cmd_buf) ? sizeof(cmd_buf) : len, cmd_buf);
    uint8_t cmd = cmd_buf[0];

    if (cmd == 0x01) { // 收到 START 命令
        if (is_ota_updating) {
            ESP_LOGW(TAG, "OTA 已经在进行中...");
            return 0; // 忽略重复启动
        }
        ESP_LOGI(TAG, "===> 收到 OTA START 指令 <===");

        // 1. 查找下一个空闲的 OTA 分区
        update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition == NULL) {
            ESP_LOGE(TAG, "找不到空闲的 OTA 分区！请检查分区表。");
            return BLE_ATT_ERR_UNLIKELY;
        }
        ESP_LOGI(TAG, "目标分区: %s, 大小: %lu 字节", update_partition->label, update_partition->size);

        // 2. 开启 OTA 通道，准备连续写入
        esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin 失败 (%s)", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            return BLE_ATT_ERR_UNLIKELY;
        }

        is_ota_updating = true;
        total_received = 0;
        ESP_LOGI(TAG, "OTA 通道已开启，等待接收固件数据...");
    } 
    else if (cmd == 0x02) { // 收到 END 命令
        if (!is_ota_updating) {
            ESP_LOGW(TAG, "收到 OTA END，但当前并未处于更新状态");
            return 0;
        }
        ESP_LOGI(TAG, "===> 收到 OTA END 指令，准备校验重启 <===");
        
        is_ota_updating = false;

        // 1. 结束写入并计算校验和
        esp_err_t err = esp_ota_end(ota_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA 结束校验失败 (%s) 固件可能不完整", esp_err_to_name(err));
            return BLE_ATT_ERR_UNLIKELY;
        }

        // 2. 将下一次启动分区指向我们刚写好的这个分区
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "设置启动分区失败 (%s)", esp_err_to_name(err));
            return BLE_ATT_ERR_UNLIKELY;
        }

        ESP_LOGI(TAG, "✅ OTA 成功！收到总字节数: %lu，设备将在 5 秒后重启...", total_received);
        // 延时一下以便把日志打印完，顺便让 BLE 正常回复底层包
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "✅ OTA 成功！收到总字节数: %lu，设备将在 4 秒后重启...", total_received);
        // 延时一下以便把日志打印完，顺便让 BLE 正常回复底层包
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "✅ OTA 成功！收到总字节数: %lu，设备将在 3 秒后重启...", total_received);
        // 延时一下以便把日志打印完，顺便让 BLE 正常回复底层包
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "✅ OTA 成功！收到总字节数: %lu，设备将在 2 秒后重启...", total_received);
        // 延时一下以便把日志打印完，顺便让 BLE 正常回复底层包
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "✅ OTA 成功！收到总字节数: %lu，设备将在 1 秒后重启...", total_received);
        // 延时一下以便把日志打印完，顺便让 BLE 正常回复底层包
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart(); // ====== 重点：重启生效！ ======
    }
    else {
        ESP_LOGW(TAG, "未知 OTA 控制命令: 0x%02X", cmd);
    }
    return 0;
}

// OTA 数据特征写数据回调 (疯狂接收源源不断的 bin 文件片段)
static int ota_data_cb(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return 0;
    }

    if (!is_ota_updating) {
        ESP_LOGW(TAG, "拒收数据：尚未开启 OTA 状态 (未收到 START)");
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    uint8_t buffer[256]; 

    if (len > sizeof(buffer)) { // 保护栈溢出
        ESP_LOGE(TAG, "单次数据包超长: %u 限定: %u", len, sizeof(buffer));
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // 1. 从 BLE 底层链表结构拷贝出数据
    os_mbuf_copydata(ctxt->om, 0, len, buffer);

    // 2. 将数据切片写入 Flash
    esp_err_t err = esp_ota_write(ota_handle, (const void *)buffer, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "写入 Flash 失败: %s", esp_err_to_name(err));
        esp_ota_abort(ota_handle);
        is_ota_updating = false;
        return BLE_ATT_ERR_UNLIKELY;
    }

    // 3. 累加进度
    total_received += len;
    
    // (可选) 每收到约 10KB 打印一次进度，避免满屏被日志刷爆拖慢速度
    if (total_received % 10240 < len) {
        ESP_LOGI(TAG, "OTA 接收进度: %lu Bytes", total_received);
    }

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
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, // 允许写和无响应写
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
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY, // 主服务
        .uuid = BLE_UUID16_DECLARE(0xA000), // OTA 服务 UUID
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(0xA001), // OTA 特征 UUID
                .access_cb = ota_ctrl_cb, // 写数据回调
                .flags = BLE_GATT_CHR_F_WRITE  , // 允许写
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xA002), // OTA 特征 UUID
                .access_cb = ota_data_cb, // 写数据回调
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP, // 允许写和无响应写
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
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(300);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(310);

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

        struct ble_gap_upd_params params = {.itvl_min = 20/1.25,//最小连接间隔为20ms，单位为1.25ms
                                            .itvl_max = 100/1.25,//最大连接间隔为100ms，单位为1.25ms
                                            .latency = 3,//连接从机可以跳过3次连接事件
                                            .supervision_timeout =200};//连接监视超时为2000ms，单位为10ms
        ble_gap_update_params(event->connect.conn_handle, &params);// 请求连接参数更新，优化连接性能和功耗
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
