#ifndef MYSERIAL_H
#define MYSERIAL_H

#include <Arduino.h>

// 串口配置
#define SERIAL_BAUDRATE 115200
#define SERIAL1_BAUDRATE 115200

// GPIO引脚配置
#define STM32_UART_RX_PIN 0  // GPIO0 作为STM32 UART接收
#define STM32_UART_TX_PIN 1  // GPIO1 作为STM32 UART发送

// 函数声明
void Serial_init();
void Serial1_init();
String Serial_readString();
bool Serial_available();
bool check_frame(const uint8_t* buf, size_t len);
void forward_to_serial1(const uint8_t* data, size_t len);
void forward_to_serial0(const uint8_t* data, size_t len);

// CMD_ID - 命令标识符定义
#define CMD_ID_QUATERNION 0x01  // 四元数命令

// 协议配置
#define MAX_FRAME_LEN 200       // 最大帧长度
#define MIN_FRAME_LEN 5         // 最小帧长度
#define BUFFER_SIZE 512         // 双缓冲大小
#define TEMP_BUFFER_SIZE 256    // 临时读取缓冲区大小
#define BUFFER_SIZE_WATER_MARK 500

// 数据帧格式 - 帧头帧尾标识
#define FRAME_HEADER  0x55
#define FRAME_TAIL    0xAA

// 双缓冲结构 - 用于数据接收的双缓冲机制
typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile size_t write_pos;
    volatile bool ready_for_parse;
} double_buffer_t;

// 解析器状态 - 协议解析状态机定义
typedef enum {
    STATE_WAIT_HEADER,   // 等待帧头
    STATE_CMD_ID,        // 命令ID
    STATE_LEN,           // 数据长度
    STATE_DATA,          // 数据内容
    STATE_SUM8,          // 校验和
    STATE_TAIL           // 帧尾
} parser_state_t;

// 解析器上下文 - 协议解析器上下文信息
typedef struct {
    parser_state_t state;        // 当前解析状态
    uint8_t cmd_id;              // 命令ID
    uint8_t data_len;            // 数据长度
    uint8_t data_received;       // 已接收数据长度
    uint8_t frame_buf[MAX_FRAME_LEN]; // 帧数据缓冲区
    uint8_t sum8;                // 校验和
    uint32_t error_count;        // 错误计数
} parser_context_t;// 解析器上下文

// 命令表 - 命令处理函数映射表
// 定义函数指针
typedef void (*CommandHandler)(uint8_t, uint8_t*, uint8_t);

// 定义命令表结构体
typedef struct 
{
    uint8_t Cmd_ID;                     // 命令ID
    CommandHandler func_handler;        // 处理函数句柄
    char desc[32];                      // 功能说明
    
}CommandList;

// 最大指令数
#define CommandList_MAX 128

// ======== 对象池数据 统一消息定义（小而固定） ========
// 传输消息结构体
typedef struct {
    uint8_t   cmd_id;    // 命令ID
    uint8_t  len;        // 数据长度
    uint8_t *payload;    // 指向数据缓冲
} tx_msg_t;

// 遥控器数据结构体
typedef struct {
    float Lx;   // 左摇杆X轴
    float Ly;   // 左摇杆Y轴
    float Rx;   // 右摇杆X轴
    float Ry;   // 右摇杆Y轴
} RCData_t;

// 全局变量声明
extern double_buffer_t g_double_buffer[2];      // 双缓冲区
extern SemaphoreHandle_t g_buffer_semaphore;    // 缓冲区信号量
extern QueueHandle_t g_quat_queue;              // 四元数队列
extern parser_context_t g_parser_ctx;           // 解析器上下文

// 函数声明
void stm32_serial_task(void *pvParameters);     // STM32串口任务
void stm32_parser_task(void *pvParameters);     // STM32解析任务
bool parse_buffer(uint8_t *buffer, size_t len); // 解析缓冲区数据
void reset_parser_context();                    // 重置解析器上下文
uint8_t calculate_sum8(uint8_t cmd_id, uint8_t len, uint8_t *data); // 计算校验和
void free_frame_slot(tx_msg_t* m);              // 释放帧槽
void TCPSEND(tx_msg_t * msg);                   // TCP发送消息
size_t pack_frame_u8(uint8_t cmd, uint8_t* data, uint8_t len, uint8_t* out, size_t cap); // 打包帧数据
void PID_check(uint8_t cmdID, uint8_t* data, uint8_t len); // PID检查并发送到VOFA

#endif // MYSERIAL_H
