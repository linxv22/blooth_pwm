#include "motor_driver.h"
#include "adc_driver.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


static const char *TAG = "电机";

#define PWM1_IN1 GPIO_NUM_12
#define MOTOR_IN1 GPIO_NUM_13
#define PWM2_IN1 GPIO_NUM_15
#define MOTOR_IN2 GPIO_NUM_16

QueueHandle_t motor_mailbox = NULL;// 电机控制消息队列

static void motor_forward(dual_motor_msg_t msg);

static void motor_stop(void);

static void motor_control(dual_motor_msg_t msg)
{
    if(msg.left_power > 0 && msg.right_power > 0) 
    {
        motor_forward(msg);
    } 
    else
        motor_stop();
    
}


static void motor_task(void *pvParameters)
{
    
    dual_motor_msg_t msg;
    while (1)
    {
        if (xQueueReceive(motor_mailbox, &msg, 100 / portTICK_PERIOD_MS) == pdTRUE)
        // 100ms 超时等待接收消息，如果接收到消息则返回 pdTRUE 
        {
            motor_control(msg); // 根据接收到的消息控制电机
        }
        else 
        {
            motor_stop(); // 如果没有接收到消息，停止电机
        }
    }
}



void motor_init(void)
{
    // 初始化电机控制引脚为输出模式
    gpio_config_t io_conf = 
    {
        .intr_type = GPIO_INTR_DISABLE,// 禁止中断
        .mode = GPIO_MODE_OUTPUT,// 输出模式
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2),// 配置电机控制引脚
        .pull_down_en = 1,// 使能下拉电阻
        .pull_up_en = 0,// 禁用上拉电阻
    };
    // 调用配置函数设置 GPIO
    gpio_config(&io_conf);
    // 初始化电机控制引脚为低电平
    gpio_set_level(MOTOR_IN1, 0);// 初始化电机控制引脚13为低电平
    gpio_set_level(MOTOR_IN2, 0);// 初始化电机控制引脚16为低电平

    ledc_timer_config_t ledc_timer = 
    {
        .speed_mode       = LEDC_LOW_SPEED_MODE,// 选择低速模式
        .timer_num        = LEDC_TIMER_0,// 选择定时器 0
        .duty_resolution  = LEDC_TIMER_10_BIT,// 占空比分辨率
        .freq_hz          = 508,  // PWM 频率 508Hz
        .clk_cfg          = LEDC_AUTO_CLK// 自动选择时钟源
    };
    ledc_timer_config(&ledc_timer);// 配置定时器

    // 2. 配置通道 (ledc_channel_config_t)
    ledc_channel_config_t ledc_channel = 
    {
        .speed_mode     = LEDC_LOW_SPEED_MODE, // 选择低速模式
        .channel        = LEDC_CHANNEL_0,// 选择通道 0  
        .timer_sel      = LEDC_TIMER_0,// 选择定时器 0  
        .intr_type      = LEDC_INTR_DISABLE,// 禁用中断     
        .gpio_num       = PWM1_IN1, // 选择 GPIO 12 作为 PWM 输出引脚
        .duty           = 0, // 初始占空比为 0%
        .hpoint         = 0 // 初始 hpoint 为 0
    };
    ledc_channel_config(&ledc_channel);
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = PWM2_IN1;// 选择 GPIO 15 作为 PWM 输出引脚
    ledc_channel.duty = 0; // 初始占空比为 0%
    ledc_channel_config(&ledc_channel);

    ESP_LOGI(TAG, "引脚初始化完成");


    motor_mailbox = xQueueCreate(1, sizeof(dual_motor_msg_t)); // 创建一个消息队列，长度为 1，每个消息大小为 dual_motor_msg_t
    // 创建一个 FreeRTOS 任务来控制电机
    xTaskCreate(motor_task, "motor_task", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    
}

static void motor_forward(dual_motor_msg_t msg)
{
    // 这里可以添加电机正转的逻辑
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 , (msg.left_power * 1024) / 100);// 设置占空比
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 );// 更新占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 , (msg.right_power * 1024) / 100);// 设置占空比
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );// 更新占空比
    gpio_set_level(MOTOR_IN1, 0);// 初始化电机控制引脚13为低电平
    gpio_set_level(MOTOR_IN2, 0);// 初始化电机控制引脚16为低电平
}

static void motor_stop(void)
{
    // 这里可以添加电机停止的逻辑
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 , 0);// 设置占空比为 0%
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 );// 更新占空比
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 , 0);// 设置占空比为 0%
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );// 更新占空比
    gpio_set_level(MOTOR_IN1, 0);// 初始化电机控制引脚13为低电平
    gpio_set_level(MOTOR_IN2, 0);// 初始化电机控制引脚16为低电平

}
