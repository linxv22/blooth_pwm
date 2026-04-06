#pragma once

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// 定义左右电机指令结构体 (共占用 4 Bytes)
typedef struct {
    int16_t left_power;   // 左电机推力：-100 到 100
    int16_t right_power;  // 右电机推力：-100 到 100
} dual_motor_msg_t;


extern QueueHandle_t motor_mailbox; // 电机控制消息队列



// 电机控制接口
void motor_init(void);

#ifdef __cplusplus
}
#endif
