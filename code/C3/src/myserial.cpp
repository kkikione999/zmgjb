#include "myserial.h"
#include "task.h"
#include "vofa.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// 外部
extern QueueHandle_t tcp_queue;

// 全局变量定义
#define VOFA_MODE    1       // use TCP output as a sort of debug
#define SERIAL_MODE  0       // use Serial output as a sort of debug

// 双缓冲
double_buffer_t g_double_buffer[2];         //双缓冲
SemaphoreHandle_t g_buffer_semaphore;       //
QueueHandle_t g_quat_queue;
parser_context_t g_parser_ctx;             // 解析器上下文

void STM32pause(uint8_t cmdID, uint8_t* data, uint8_t len);
void phase_quaternion(uint8_t cmdID, uint8_t* data, uint8_t len);
void parseJoystick(uint8_t cmdID, uint8_t* data, uint8_t len);
void setPitchPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len);
void setRollPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len);
void setYawPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len);
void software_restart(uint8_t cmdID, uint8_t* data, uint8_t len);



CommandList cmd_table[CommandList_MAX] = {
    {0x00, STM32pause,"null"},                                              // 暂停
    {0x01, phase_quaternion,"Quaternion"},                                  // 转发四元数
    {0x02, parseJoystick,  "Joystick data parse"},                          // 解析Joystick数据
	{0x03, setPitchPIDparameter,  "set Pitch PID parameter"},           // 设置Pitch PID参数        
	{0x04, setRollPIDparameter,  "set Row PID parameter"},              // 设置Roll PID参数
	{0x05, setYawPIDparameter,  "set Yaw PID parameter"},               // 设置Yaw PID参数
	{0x06, software_restart,  "software_restart"},                       //  软件重启
    {0x07,PID_check,"PID_check"},                                        //上位机回传确认
    /* 其余项留空默认是零，如果需要可以再补 */
};

// Slot对象池
// ======== 对象池：固定大小帧缓冲（32帧，每帧最多128字节） ========
#define FRAME_MAX   128
#define FRAME_POOL  32

static uint8_t  g_frame_buf[FRAME_POOL][FRAME_MAX];
static uint32_t g_frame_in_use = 0; // 简陋位图示例

static int alloc_frame_slot(void) {
    for (int i=0;i<FRAME_POOL;i++) {
        if (((g_frame_in_use>>i)&1)==0) { g_frame_in_use |= (1u<<i); return i; }
    }
    return -1; // 没空位：丢弃/覆盖/统计
}

static void free_frame_slot(int idx) {
    g_frame_in_use &= ~(1u<<idx); 
}

void free_frame_slot(tx_msg_t* m) { 
    intptr_t base = (intptr_t)m->payload;           // 本次消息的payload地址
    intptr_t pool0= (intptr_t)&g_frame_buf[0][0];   // 池起始地址
    intptr_t step = (intptr_t)&g_frame_buf[1][0] - pool0; // 每个槽的跨度（FRAME_MAX）
    int idx = (int)((base - pool0) / step);         // 反推槽位下标
    if (idx>=0 && idx<FRAME_POOL) g_frame_in_use &= ~(1u<<idx); 
}

/*
把数据发送到队列 ，什么数据？
*/
static inline void enqueue_raw(uint8_t cmd, const uint8_t* data, uint8_t len) 
{
    if (!tcp_queue || len>FRAME_MAX) 
    return;
    int slot = alloc_frame_slot();
    if (slot<0) return; // TODO: 统计丢包
    memcpy(g_frame_buf[slot], data, len);

    tx_msg_t m = { .cmd_id=cmd, .len=len, .payload=g_frame_buf[slot] };
    // 把数据发送到队列
    if (xQueueSend(tcp_queue, &m, 0) != pdPASS)                             // 发送数据到队列
    {
        // 队列满，释放槽位
        free_frame_slot(slot);
    }
}

void Serial_init() {
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial); // 等待串口0就绪
  delay(100);     // 短暂延时确保稳定
}

// 初始化串口1，使用GPIO0和GPIO1
void Serial1_init() {
  Serial1.begin(115200, SERIAL_8N1, 1, 0);
  Serial1.setTimeout(5);
  delay(100);     // 短暂延时确保稳定
}

String Serial_readString() {
  return Serial.readString();
}

bool Serial_available() {
  return Serial.available();
}

// 检查帧完整性
bool check_frame(const uint8_t* buf, size_t len) {
  if(len < MIN_FRAME_LEN) return false;
  return (buf[0] == FRAME_HEADER && buf[len-1] == FRAME_TAIL);
}

// 转发数据到串口1(STM32)
void forward_to_serial1(const uint8_t* data, size_t len) {
  if(check_frame(data, len)) {
    Serial1.write(data, len);
  }
}

// 转发数据到串口0(电脑)
void forward_to_serial0(const uint8_t* data, size_t len) {
  if(check_frame(data, len)) {
    Serial.write(data, len);
  }
}

// 计算SUM8校验
uint8_t calculate_sum8(uint8_t cmd_id, uint8_t len, uint8_t *data) {
    uint8_t sum = cmd_id + len;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum & 0xFF;
}

// 重置解析器上下文
void reset_parser_context() {
    g_parser_ctx.state = STATE_WAIT_HEADER;
    g_parser_ctx.cmd_id = 0;
    g_parser_ctx.data_len = 0;
    g_parser_ctx.data_received = 0;
    g_parser_ctx.sum8 = 0;
    memset(g_parser_ctx.frame_buf, 0, MAX_FRAME_LEN);
}


// 四元数接收回调
void phase_quaternion(uint8_t cmdID, uint8_t* data, uint8_t len) {
    float* quat = (float*) data;
    VOFA_print("VOFA Quaternion:");
    VOFA_print(quat[0]);
    VOFA_print(",");
    VOFA_print(quat[1]);
    VOFA_print(",");
    VOFA_print(quat[2]);
    VOFA_print(",");
    VOFA_print(quat[3]);
    VOFA_println();
    Serial.printf("VOFA Quaternion: q0=%.3f, q1=%.3f, q2=%.3f, q3=%.3f\n", 
                quat[0], quat[1], quat[2], quat[3]);
}

/*
解析缓冲区数据,解析字节流，当完整帧接收后，会调用 enqueue_raw 将帧放入 tcp_queue，然后由 TCPSEND 处理。
其中，“解析器上下文结构体”变量 g_parser_ctx 根据串口协议解析的状态机逐步填充数据，他作为解析过程的临时储存容器，记录当前的
解析状态、命令ID、数据长度、已经接受的数据、校验和等信息。

*/
bool parse_buffer(uint8_t *buffer, size_t len) 
{
    for (size_t i = 0; i < len; i++)//每次循环处理一个索引位置，对应一个字节
    {
        uint8_t byte = buffer[i];   //8位无符号整数，即1个字节
        switch (g_parser_ctx.state) 
        {
            case STATE_WAIT_HEADER: //因为初始化时调用了reset_parser_context()，所以这里会先执行一次
                if (byte == FRAME_HEADER) 
                {
                    g_parser_ctx.state = STATE_CMD_ID;
                }
                break;
                
            case STATE_CMD_ID:
                g_parser_ctx.cmd_id = byte;
                g_parser_ctx.state = STATE_LEN;
                break;
                
            case STATE_LEN:
                g_parser_ctx.data_len = byte;
                if (g_parser_ctx.data_len > MAX_FRAME_LEN) 
                {
                    // 数据长度超限，重置状态机
                    g_parser_ctx.error_count++;
                    reset_parser_context();
                } 
                else 
                {
                    g_parser_ctx.data_received = 0;
                    g_parser_ctx.state = STATE_DATA;
                }
                break;
                
            case STATE_DATA:
                if (g_parser_ctx.data_received < g_parser_ctx.data_len) 
                {
                    g_parser_ctx.frame_buf[g_parser_ctx.data_received] = byte;
                    g_parser_ctx.data_received++;
                    
                    if (g_parser_ctx.data_received == g_parser_ctx.data_len) 
                    {
                        g_parser_ctx.state = STATE_SUM8;
                    }
                }
                break;
                
            case STATE_SUM8:
                g_parser_ctx.sum8 = byte;
                g_parser_ctx.state = STATE_TAIL;
                break;
                
            case STATE_TAIL:
                if (byte == FRAME_TAIL) 
                {
                    // 完整帧接收完成，进行校验
                    uint8_t calculated_sum = calculate_sum8(
                        g_parser_ctx.cmd_id, 
                        g_parser_ctx.data_len, 
                        g_parser_ctx.frame_buf
                    );
                    if (calculated_sum == g_parser_ctx.sum8) {
                        if(g_parser_ctx.cmd_id > 128){
                            g_parser_ctx.error_count++;
                        }
                        else
                        {
                            // 查表法查找对应的函数执行对应的任务
                            if (cmd_table[g_parser_ctx.cmd_id].func_handler != nullptr)
                            {
                                enqueue_raw(g_parser_ctx.cmd_id, g_parser_ctx.frame_buf, g_parser_ctx.data_len);//将数据入队
                                //根据深入的探究，这里已经把数据入队了，入的是STM32数据队列 QueueHandle_t tcp_queue;
                            }
                        }
                    } else {
                        g_parser_ctx.error_count++;
                    }
                } 
                else 
                {
                    g_parser_ctx.error_count++;
                }
                // 无论成功与否，都重置状态机
                reset_parser_context();
                break;
        }
    }
    
    return true;
}

// 通过串口数据命令ID(cmd_ID)找到对应的函数进行处理
void TCPSEND(tx_msg_t * msg)
{
/*若处理函数存在，则调用它，传入三个参数：命令ID、数据载荷指针、数据长度。
例如，若 cmd_id 为 0x01，则调用 phase_quaternion 函数，该函数会将四元数数据通过 VOFA_print 发送到 TCP 连接；*/
    if (cmd_table[msg->cmd_id].func_handler != nullptr)//检查该命令对应的处理函数指针是否非空
    {
        cmd_table[msg->cmd_id].func_handler(msg->cmd_id, msg->payload, msg->len);
    }
    else
    {
        while(1)
        {
            Serial.println("vofa TCPSEND bug!");
        }
    }
    free_frame_slot(msg);
}

// 传入：cmd, data指针, data长度len, 输出缓冲out(你提供), out容量cap
// 返回：帧总字节数；若容量不足返回0
size_t pack_frame_u8(uint8_t cmd, uint8_t* data, uint8_t len,
                     uint8_t* out, size_t cap)
{
    size_t need = 1 + 1 + 1 + len + 1 + 1;
    if (cap < need) return 0;

    size_t w = 0;
    out[w++] = 0x55;          // HEADER
    out[w++] = cmd;           // CMD
    out[w++] = len;           // LEN
    if (len && data) {
        memcpy(&out[w], data, len);
        w += len;
    }
    out[w++] = calculate_sum8(cmd, len, data); // SUM
    out[w++] = 0xAA;                               // TAIL
    return w;
}

void STM32pause(uint8_t cmdID, uint8_t* data, uint8_t len)
{

}

void parseJoystick(uint8_t cmdID, uint8_t* data, uint8_t len)
{

}
void setPitchPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{

}
void setRollPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{

}
void setYawPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{

}
void software_restart(uint8_t cmdID, uint8_t* data, uint8_t len){

}


 void PID_check(uint8_t cmdID, uint8_t* data, uint8_t len)
{
    // 解析PID参数并发送到VOFA
    // 假设数据区包含18个浮点数（三个轴的PID参数，每个轴6个参数：角度环KP,KI,KD，角速度环KP,KI,KD）
    if (len == 72) 
    {
        float* pid_params = (float*)data;
        VOFA_print("PID:");
        for (int i = 0; i < 18; i++) {
            VOFA_print(pid_params[i]);
            if (i < 17) VOFA_print(",");
        }
        VOFA_println();
    } 
    
 else 
    {
        VOFA_print("PID_check: invalid len ");
        VOFA_print(len);
        VOFA_println();
    }
}
