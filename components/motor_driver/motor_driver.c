#include "motor_driver.h"
#include "adc_driver.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"


static const char *TAG = "电机";

#define PWM1_IN1 GPIO_NUM_12 //Y4
#define MOTOR_IN1 GPIO_NUM_13//Y3
#define PWM2_IN1 GPIO_NUM_15 //Y1
#define MOTOR_IN2 GPIO_NUM_16 //Y2

QueueHandle_t motor_mailbox = NULL;// 电机控制消息队列

void motor_stop(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 , 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0 );
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 , 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1 );
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2 , 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2 );
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3 , 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3 );
}

static void motor_task(void *pvParameters)
{
    dual_motor_msg_t msg = 
    {
        .left_power = 0,
        .right_power = 0
    }; // 用于接收电机控制消息的结构体
    int left_station = 0; // 0: 停止, 1: 前进, 2: 后退
    int right_station = 0; // 0: 停止, 1: 前进, 2: 后退
    int new_left_station = 0, new_right_station = 0; // 用于存储新的电机状态
    bool need_delay = false; // 是否需要在正反转切换时添加延时
    motor_stop(); // 确保电机初始状态为停止
    while (1)
    {
        if (xQueueReceive(motor_mailbox, &msg, 200 / portTICK_PERIOD_MS) == pdTRUE)
        {
            new_left_station = (msg.left_power > 0) ? 1 : ((msg.left_power < 0) ? 2 : 0);
            new_right_station = (msg.right_power > 0) ? 1 : ((msg.right_power < 0) ? 2 : 0);

            need_delay = false;
            // 检查左电机是否在正反转切换
            if (new_left_station != left_station && left_station != 0 && new_left_station != 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                need_delay = true;
            }
            // 检查右电机是否在正反转切换
            if (new_right_station != right_station && right_station != 0 && new_right_station != 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                need_delay = true;
            }

            if (need_delay) { // 等待 10ms 确保电机完全停止，避免烧毁电机驱动
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }

            left_station = new_left_station;
            right_station = new_right_station;

            // 设置左电机
            if (msg.left_power > 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (msg.left_power * 1024) / 100);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            } else if (msg.left_power < 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (-msg.left_power * 1024) / 100);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            } else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            }

            // 设置右电机
            if (msg.right_power > 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (msg.right_power * 1024) / 100);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            } else if (msg.right_power < 0) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (-msg.right_power * 1024) / 100);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            } else {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            }
        }
        else 
        {
            if (left_station != 0 || right_station != 0)
            {
                motor_stop(); // 如果没有接收到消息，停止电机
                left_station = 0;
                right_station = 0;
            }
        }
    }
}



void motor_init(void)
{
    

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

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = MOTOR_IN1;// 选择 GPIO 13 作为 PWM 输出引脚
    ledc_channel.duty = 0; // 初始占空比为 0%
    ledc_channel_config(&ledc_channel);

    
    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = MOTOR_IN2;// 选择 GPIO 16 作为 PWM 输出引脚
    ledc_channel.duty = 0; // 初始占空比为 0%
    ledc_channel_config(&ledc_channel);
    ESP_LOGI(TAG, "引脚初始化完成");


    motor_mailbox = xQueueCreate(1, sizeof(dual_motor_msg_t)); // 创建一个消息队列，长度为 1，每个消息大小为 dual_motor_msg_t
    // 创建一个 FreeRTOS 任务来控制电机
    xTaskCreate(motor_task, "motor_task", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    
}
