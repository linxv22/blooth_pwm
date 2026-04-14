#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "ble_server.h"
#include "esp_log.h"
#include "motor_driver.h"
#include "adc_driver.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "应用程序启动");
    
    esp_err_t ret ;
       /* NVS flash initialization */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return;
    }
    

    
    motor_init();// 初始化电机控制
    adc_init();// 初始化 ADC 采样
    ble_simple_start();// 初始化并启动简易 BLE 从机

    ESP_LOGI(TAG, "==== 应用程序启动 OTA 更新成功4.14.442 ====");
    
    // int i = 0;
    // int count = -100;  
    // dual_motor_msg_t motor_msg = {0}; // 初始化电机控制消息结构体

    while (1) {
        // if (i % 40 == 0) // 每40次循环执行一次
        // {
        //     // 模拟接收一个电机控制消息，这里可以替换为实际的消息接收逻辑
        //     count+=10;
        //     motor_msg.left_power = count; // 模拟左电机推力在 -100-100 之间变化
        //     motor_msg.right_power = count; // 模拟右电机推力在 -100-100 之间变化
        //     ESP_LOGI(TAG, "模拟接收电机控制消息：左推力=%d, 右推力=%d", motor_msg.left_power, motor_msg.right_power);
        //     if(count % 100 == 0 && count != 0) // 每当计数器达到100的倍数时重置
        //     {
        //         count = -100; // 重置计数器
        //         ESP_LOGI(TAG, "计数到100");
        //     }
        //     i =0; // 重置循环计数器
        // }
        // // 将电机控制消息发送到消息队列
        // xQueueSend(motor_mailbox, &motor_msg, 100 / portTICK_PERIOD_MS);// 100ms 超时等待发送消息，如果发送成功则返回 pdTRUE     
        // i++;
      
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        
    }
}
