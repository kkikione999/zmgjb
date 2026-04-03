/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define ESP32_TX_Success 1
#define ESP32_TX_False   0

// 命令表
// 定义函数指针
typedef void (*CommandHandler)(uint8_t, uint8_t*, uint8_t);//

// 定义命令表结构体
typedef struct {
    uint8_t Cmd_ID;                     // 命令ID
    CommandHandler func_handler;        // 处理函数句柄
    char desc[32];                      // 功能说明
}CommandList;

// CMD_ID
#define CMD_ID_QUATERNION 0x01  // 四元数命令
// 协议配置
#define MAX_FRAME_LEN 200       // 最大帧长度
#define MIN_FRAME_LEN 5         // 最小帧长度
#define BUFFER_SIZE 512         // 双缓冲大小
#define TEMP_BUFFER_SIZE 256    // 临时读取缓冲区大小
#define BUFFER_SIZE_WATER_MARK 500
#define CommandList_MAX 20

void parseJoystick(uint8_t cmdID, uint8_t* data, uint8_t len);
// 数据帧格式
#define FRAME_HEADER  0x55
#define FRAME_TAIL    0xAA

#define FRAME_MAX   128
#define FRAME_POOL  32             // 数据帧池大小（最大并发发送数）

extern uint8_t g_tx_frame_buf[FRAME_POOL][FRAME_MAX];
extern uint32_t g_frame_in_use;
extern osMessageQueueId_t g_txQueueHandle;
extern osSemaphoreId_t s_parse_sem;
extern volatile uint8_t s_cur_write_idx;
extern osMessageQueueId_t joystickQueueHandle;

// 全局串口消息队列类型
typedef struct 
{
  uint8_t cmd_id;
  uint8_t len;
	uint8_t* payload;
} g_msg_t;

typedef struct {
  float Lx;		//左遥感的x轴的值
  float Ly;		//左遥感的y轴的值
	float Rx;		//右遥感的x轴的值
	float Ry;		//右遥感的y轴的值
} joystick_t;

typedef struct {
  float throttle;   // 0~1
  float roll;       // -1~1
  float pitch;      // -1~1
  float yaw;        // -1~1
  uint32_t tick;    // 产生该指令的系统tick（用于超时保护）
} rc_cmd_t;

extern osMessageQueueId_t rcCmdQueueHandle;



#define FRAME_MIN_LEN           5       // 头(1)+CMD(1)+LEN(1)+SUM(1)+尾(1)  (当 LEN>=0，但建议 LEN>=1)
#define FRAME_MAX_LEN           200     // 你的上限（整帧）

// ====== 双缓冲与DMA缓冲参数 ======
#define RX_DMA_LEN              256     // HAL_UARTEx_ReceiveToIdle_DMA 使用的临时DMA接收区
#define RX_DBL_BUF_SIZE         512     // 双缓冲每块容量（根据吞吐调）
#define RX_WATERMARK            (RX_DBL_BUF_SIZE*3/4)



// ====== 双缓冲结构 ======
typedef struct {
    uint8_t  buf[RX_DBL_BUF_SIZE];
    uint16_t write_pos;
    uint8_t  ready_for_parse;           // 0/1 标记
} rx_dbl_buf_t;

extern rx_dbl_buf_t s_db[2];


/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void imu_send_euler_dma(float roll_deg, float pitch_deg, float yaw_deg);
void imu_send_Quaternion_to_ESP(float q1, float q2, float q3, float q4);
void g_tx_enqueue_raw(uint8_t cmd, uint8_t* data, uint8_t len);
void free_frame_slot(g_msg_t* m);
uint8_t esp_send_from_msg(const g_msg_t* m);
void parse_frames_chunk(const uint8_t* chunk, uint16_t chunk_len);
void software_restart(uint8_t cmdID, uint8_t* data, uint8_t len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

