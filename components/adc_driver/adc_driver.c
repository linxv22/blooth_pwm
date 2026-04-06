#include "adc_driver.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


static const char *TAG = "电池采样";
// ADC sampling pins
#define I_ADC ADC_CHANNEL_3 //GPIO 4
#define V_ADC ADC_CHANNEL_4 //GPIO 5

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

// ADC calibration handles
adc_cali_handle_t i_cali_handle  = NULL;
adc_cali_handle_t v_cali_handle  = NULL;

// ADC 1 单位句柄
adc_oneshot_unit_handle_t adc1_handle;

bool do_calibration1_chan0 = false;
bool do_calibration1_chan1 = false;



void adc_init(void)
{
    // 初始化 ADC 1 单位 
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,// 使用 ADC1
        .ulp_mode = ADC_ULP_MODE_DISABLE,// 禁用 ULP 模式
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));// 创建 ADC1 单位句柄
    // 配置 ADC1 的通道属性
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 默认 12-bit 分辨率
        .atten = ADC_ATTEN_DB_12,         // 12dB 衰减，测量范围约 0 ~ 3.1V (注意旧版ESP-IDF可能是 ADC_ATTEN_DB_11)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, I_ADC, &config));// 配置 ADC1 的通道属性
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, V_ADC, &config));// 配置 ADC1 的通道属性
    // 初始化 ADC 校准句柄
    
    do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, I_ADC, ADC_ATTEN_DB_12, &i_cali_handle);// 初始化 ADC1 通道 0 的校准
    do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, V_ADC, ADC_ATTEN_DB_12, &v_cali_handle);// 初始化 ADC1 通道 1 的校准

    // TODO: configure ADC channels when available
    ESP_LOGI(TAG, "ADC init");
}

void adc_read(void)
{   
    // 读取 ADC1 通道 0 的电压值
    int i_adc_raw = 0, v_adc_raw = 0;
    int i_adc_voltage = 0, v_adc_voltage = 0;
    if (do_calibration1_chan0) {
        adc_oneshot_read(adc1_handle, I_ADC, &i_adc_raw);
        adc_cali_raw_to_voltage(i_cali_handle, i_adc_raw, &i_adc_voltage);    
    }
    ESP_LOGI(TAG, "I_ADC: %d", i_adc_voltage);      
    // 读取 ADC1 通道 1 的电压值
    if (do_calibration1_chan1) {
        adc_oneshot_read(adc1_handle, V_ADC, &v_adc_raw); 
        adc_cali_raw_to_voltage(v_cali_handle, v_adc_raw, &v_adc_voltage);
    }
    ESP_LOGI(TAG, "V_ADC: %d", v_adc_voltage);

}

float adc_read_current(void)
{   
    int i_adc_raw = 0;
    int i_adc_voltage = 0;
    float current = 0.0f;

    if (do_calibration1_chan0) 
    {
        adc_oneshot_read(adc1_handle, I_ADC, &i_adc_raw);
        adc_cali_raw_to_voltage(i_cali_handle, i_adc_raw, &i_adc_voltage);
        
        // ACS725LLCTR-20AB-T 芯片参数 (VCC = 3.3V):
        // 0A 对应电压为 VCC/2 = 1.65V (1650mV)
        // 灵敏度为 66mV/A
        current = (i_adc_voltage - 1650.0f) / 66.0f;
    } else 
    {
        ESP_LOGE(TAG, "电流ADC未校准");
    }

    return current;
}

float adc_read_voltage(void)
{
    int v_adc_raw = 0;
    int v_adc_voltage = 0; // mV
    float battery_voltage = 0.0f; // V

    if (do_calibration1_chan1) 
    {
        adc_oneshot_read(adc1_handle, V_ADC, &v_adc_raw);
        adc_cali_raw_to_voltage(v_cali_handle, v_adc_raw, &v_adc_voltage);

        // 硬件分压: R20 = 22kΩ, R21 = 10kΩ
        // V_ADC = BAT+ * (10 / 32) -> BAT+ = V_ADC * 3.2
        battery_voltage = (v_adc_voltage / 1000.0f) * 3.2f; // 将 mV 转换为 V，再乘以分压系数
    } else 
    {
        ESP_LOGE(TAG, "电压ADC未校准");
    }

    return battery_voltage;
}


// --------------------------------------------------
// 辅助函数：硬件校准初始化 (ESP-IDF 官方标准写法)
// --------------------------------------------------
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "ADC 校准方案: Curve Fitting (曲线拟合)");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC 通道[%d] 校准初始化成功", channel);
    } else {
        ESP_LOGE(TAG, "ADC 校准初始化失败");
    }
    return calibrated;
}
