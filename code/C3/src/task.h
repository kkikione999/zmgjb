#ifndef TASK_H__
#define TASK_H__

#include "main.h"
#include "myserial.h"
#include <AsyncWebSocket.h>
#include "vofa.h"

// WebSocket 对象在 webserver.cpp 中定义，这里 extern
extern AsyncWebSocket ws;

// 串口0缓存大小
#define SERIAL0_FRAME_SIZE 128

// WebSocket 队列消息结构
typedef struct 
{
    AsyncWebSocketClient *client;  // 哪个客户端发来的
    String data;                   // 文本数据（JSON 字符串）
} ws_message_t;



// ===== 队列句柄 =====
extern QueueHandle_t ws_queue;      // WebSocket 消息队列
extern QueueHandle_t tcp_queue;     // VOFA/TCP 队列
extern QueueHandle_t RCdata_queue;  // 遥杆数据队列
extern QueueHandle_t adc_queue;     // ADC 数据队列
extern QueueHandle_t pid_queue;     // PID 参数队列，与stm32端匹配

// PID 参数结构体（与STM32端匹配）
typedef enum {
    PID_AXIS_ROLL = 0,
    PID_AXIS_PITCH = 1,
    PID_AXIS_YAW = 2
} pid_axis_t;

typedef struct {
    float kp_angle;  // 角度环 P 增益
    float ki_angle;  // 角度环 I 增益
    float kd_angle;  // 角度环 D 增益
    float kp_rate;   // 角速度环 P 增益
    float ki_rate;   // 角速度环 I 增益
    float kd_rate;   // 角速度环 D 增益
} AxisPID;// 轴PID参数，与stm32端匹配

typedef struct {
    pid_axis_t axis;
    AxisPID pid;
} pid_params_t;// PID参数，与stm32端匹配

// 与STM32端的pid_update_msg_t保持一致
typedef pid_params_t pid_update_msg_t;

// 硬件初始化
void hardware_init();

// 任务创建入口
void Task_init();


void stm32_uart_rx_task(void *pvParameters);
void stm32_uart_tx_task(void *pvParameters);
void stm32_parse_task(void *pvParameters);
void serial0_task(void *pvParameters);
void pwr_task(void *pvParameters);
void vofa_task(void *pvParameters);

// WebSocket 相关任务
void websocket_task(void *pvParameters);        // 解析 WebSocket JSON
void websocket_status_task(void *pvParameters); // 可选状态上报
void udp_rx_task(void *pvParameters);           // 原有 UDP 测试任务（会改成可选）

#endif // TASK_H__
