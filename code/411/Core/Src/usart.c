
#include "usart.h"
#include "pid.h"
/* USER CODE BEGIN 0 */
#include "stdio.h"
#include <string.h>
#include "main.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern osMessageQueueId_t pidUpdateQueueHandle;

int16_t STM32pause_count=10;


// Printf DMA 缓冲区相关变量
static char printf_bufA[256], printf_bufB[256];  // 双缓冲区
static char *printf_cur_buf = printf_bufA;       // 当前使用的缓冲区
static volatile uint16_t printf_buf_index = 0;   // 当前缓冲区索引
static volatile uint8_t printf_dma_busy = 0;     // DMA忙标志
#define PRINTF_BUF_SIZE 256

// Printf 双缓冲切换函数
static inline char* printf_flip_buf(void)
{
    printf_cur_buf = (printf_cur_buf == printf_bufA) ? printf_bufB : printf_bufA;
    return printf_cur_buf;
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
uint8_t g_tx_frame_buf[FRAME_POOL][FRAME_MAX];
uint32_t g_frame_in_use = 0;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = Uart_computer_TX_Pin|Uart_computer_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_usart1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart1_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */
	
  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = Uart_ESP32_TX_Pin|Uart_ESP32_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Stream6;
    hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_usart2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_usart2_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, Uart_computer_TX_Pin|Uart_computer_RX_Pin);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, Uart_ESP32_TX_Pin|Uart_ESP32_RX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float apply_deadzone(float x, float dz)
{
  if (x > -dz && x < dz) return 0.0f;
  return x;
}

// 这里假设ESP32发来的摇杆 float 是 [-100, 100]，你可按真实量程改
#define RC_RAW_MAX   (100.0f)
#define RC_DEADZONE  (0.03f)   // 归一化后的死区


static void printf_send_dma(void)
{
    if (printf_buf_index == 0 || printf_dma_busy) return;
    
    // 切换到另一个缓冲区
    char *send_buf = printf_flip_buf();
    uint16_t send_len = printf_buf_index;
    
    // 复制当前缓冲区内容到发送缓冲区
    memcpy(send_buf, printf_cur_buf == printf_bufA ? printf_bufB : printf_bufA, send_len);
    
    // 重置当前缓冲区索引
    printf_buf_index = 0;
    
    // 启动DMA发送
    printf_dma_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)send_buf, send_len) != HAL_OK) {
        printf_dma_busy = 0;  // 发送失败，释放忙标志
    }
}

int fputc(int ch, FILE *f) 
{
    // 进入临界区
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    // 如果缓冲区满，先发送
    if (printf_buf_index >= (PRINTF_BUF_SIZE - 1)) {
        printf_send_dma();
//        // 等待DMA发送完成或超时
//        uint32_t timeout = HAL_GetTick() + 100; // 100ms超时
//        while (printf_dma_busy && (HAL_GetTick() < timeout)) {
//            // 等待DMA完成
//        }
        __set_PRIMASK(primask);          // 先开中断！
        // 这里不要 while 等；最多直接 return 或丢字符
        return ch; // 或者继续写新缓冲（需要更完整的双缓冲设计）	    
	    
    }
    
    // 将字符添加到缓冲区
    printf_cur_buf[printf_buf_index++] = (char)ch;
    
    // 如果遇到换行符，立即发送
    if (ch == '\n' || ch == '\r') {
        printf_send_dma();
    }
    
    // 退出临界区
    __set_PRIMASK(primask);
    
    return ch;
}

static char txbufA_stm[64], txbufB_stm[64];          // dubble buffer
static char *cur_buf_stm = txbufA_stm;
#define DEG2RAD 0.01745329251994329577f              // 角度转弧度

volatile uint8_t USART1_DMA_TX_ready = 1;
static inline char* flip_buf_stm(void)               //双缓冲切换函数
{             //
    cur_buf_stm = (cur_buf_stm == txbufA_stm) ? txbufB_stm : txbufA_stm;
    return cur_buf_stm;
}

void imu_send_euler_dma(float roll_deg, float pitch_deg, float yaw_deg)//给stm32发送欧拉角
{
    if (!USART1_DMA_TX_ready) return;                      // if busy, don't write, in case of overwrite
    char *buf = flip_buf_stm();

	// choose one: degree or rad
  //    int n = snprintf(buf, 64, "%.3f,%.3f,%.3f\r\n",
  //                     roll_deg, pitch_deg, yaw_deg);
    int n = snprintf(buf, 64, "%.3f,%.3f,%.3f\r\n",
                      roll_deg*DEG2RAD, pitch_deg*DEG2RAD, yaw_deg*DEG2RAD);

    if (n <= 0) return;

    USART1_DMA_TX_ready = 0;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, (uint16_t)n) != HAL_OK) {
        USART1_DMA_TX_ready = 1;  	//when error, lease resource                        
    }
}


static char txbufA_esp[64], txbufB_esp[64];          // dubble buffer
static char *cur_buf_esp = txbufA_esp;
volatile uint8_t USART2_DMA_TX_ready = 1;							// 1 = DMA is ok, 0=DMA is busy

static inline char* flip_buf_esp(void)                                //双缓冲切换函数                       
{
    cur_buf_esp = (cur_buf_esp == txbufA_esp) ? txbufB_esp : txbufA_esp;
    return cur_buf_esp;
}

void imu_send_Quaternion_to_ESP(float q1, float q2, float q3, float q4)//给esp32发送四元数
{
    if (!USART2_DMA_TX_ready) return;

    uint8_t *buf = (uint8_t*)flip_buf_esp();
    uint16_t w = 0;

    // ??/??/??
    buf[w++] = 0x55;      // HEADER
    buf[w++] = 0x01;      // CMD_ID = ???
    buf[w++] = 16;        // LEN = 16 ??

    memcpy(&buf[w], &q1, 4); w += 4;
    memcpy(&buf[w], &q2, 4); w += 4;
    memcpy(&buf[w], &q3, 4); w += 4;
    memcpy(&buf[w], &q4, 4); w += 4;

    uint32_t s = 0;
    for (uint16_t i = 1; i < 1 + 1 + 1 + 16; ++i) { 
        s += buf[i];
    }
    buf[w++] = (uint8_t)(s & 0xFF);

    buf[w++] = 0xAA;

    // DMA 发送
    USART2_DMA_TX_ready = 0;
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buf, w) != HAL_OK) {
        USART2_DMA_TX_ready = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
		if (huart->Instance == USART1) {
        USART1_DMA_TX_ready = 1;
        printf_dma_busy = 0;  // Printf DMA发送完成
    }
    if (huart->Instance == USART2) {
        USART2_DMA_TX_ready = 1; //
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
		if (huart->Instance == USART1) {
        USART1_DMA_TX_ready = 1; //	when uasrt transmit is in ERROR, lease resource.
        printf_dma_busy = 0;      // Printf DMA错误，释放忙标志
    }
    if (huart->Instance == USART2) {
        USART2_DMA_TX_ready = 1; //	when uasrt transmit is in ERROR, lease resource.
    }
}

static int alloc_frame_slot(void) {
    for (int i=0;i<FRAME_POOL;i++) {
        if (((g_frame_in_use>>i)&1)==0) {
					taskENTER_CRITICAL();
					g_frame_in_use |= (1u<<i);
					taskEXIT_CRITICAL();
					return i;
				}
    }
    return -1; // 没空位：丢弃/覆盖/统计
}

// 释放缓冲区
static void free_frame_slot_private(int idx) {
    g_frame_in_use &= ~(1u<<idx); 
}

// 释放缓冲区
void free_frame_slot(g_msg_t* m) { 
    intptr_t base = (intptr_t)m->payload;           // 本次消息的payload地址
    intptr_t pool0= (intptr_t)&g_tx_frame_buf[0][0];   // 池起始地址
    intptr_t step = (intptr_t)&g_tx_frame_buf[1][0] - pool0; // 每个槽的跨度（FRAME_MAX）
    int idx = (int)((base - pool0) / step);         // 反推槽位下标
    if (idx>=0 && idx<FRAME_POOL) g_frame_in_use &= ~(1u<<idx); 
}

// 在任务中把数据发送到队列g_txQueueHandle 从而把数据发送给ESP32
void g_tx_enqueue_raw(uint8_t cmd, uint8_t* data, uint8_t len) {
    if (!g_txQueueHandle || len>FRAME_MAX) return;
    int slot = alloc_frame_slot();
    if (slot<0) return; // TODO: 统计丢包
    memcpy(g_tx_frame_buf[slot], data, len);

    g_msg_t m = { .cmd_id=cmd, .len=len, .payload=g_tx_frame_buf[slot] };
    // 把数据发送到队列
    if (osMessageQueuePut(g_txQueueHandle, &m, 0, 0) != osOK) {
        // 队列满，释放槽位
        free_frame_slot_private(slot);
    }
}

// ---- sum8校验（按协议：仅对 CMD + LEN + DATA 求和）----
static inline uint8_t sum8_cmd_len_data(uint8_t cmd, uint8_t len, const uint8_t* data)
{
    uint32_t s = cmd + len;
    for (uint16_t i = 0; i < len; ++i) s += data[i];
    return (uint8_t)(s & 0xFF);
}


/**
 * @brief  按协议打包并经USART2 DMA发送一帧
 * @param  cmd   指令ID
 * @param  data  数据区首地址
 * @param  len   数据区长度（不含头尾与校验）
 * @return ESP32_TX_Success  发送已发起（DMA启动成功）
 * @return ESP32_TX_False 	 忙/参数非法/启动失败（可选择重试或丢包）
 */
static uint8_t esp_send_frame(uint8_t cmd, const uint8_t* data, uint16_t len)
{
    // 1) 检查长度边界：整帧 = 1(头)+1(cmd)+len(数据)+1(sum)+1(尾)
    uint16_t total = (uint16_t)(1 + 1 + 1 + len + 1 + 1);
    if (len == 0) return ESP32_TX_False;                // 至少1字节数据
    if (total < MIN_FRAME_LEN || total > MAX_FRAME_LEN) return ESP32_TX_False;

    // 2) UART DMA忙则直接返回（非阻塞风格）
    if (!USART2_DMA_TX_ready) return ESP32_TX_False;

    // 3) 申请发送缓冲
    uint8_t* buf = (uint8_t*)flip_buf_esp();
    if (!buf) return ESP32_TX_False;

    // 4) 按协议打包
    uint16_t w = 0;
    buf[w++] = FRAME_HEADER;   // 0x55
    buf[w++] = cmd;            // CMD
		buf[w++] = len;						 // LEN
    // DATA
		if (len && data) {
					memcpy(&buf[w], data, len);
					w += len;
		}
		
    // SUM8(CMD+DATA)
    buf[w++] = sum8_cmd_len_data(cmd, len, len ? data : NULL); // SUM8
    // TAIL
    buf[w++] = FRAME_TAIL;     // 0xAA

    // 5) 发起DMA发送
    USART2_DMA_TX_ready = 0;
    if (HAL_UART_Transmit_DMA(&huart2, buf, w) != HAL_OK) {
        USART2_DMA_TX_ready = 1; // 回滚忙标志
        return ESP32_TX_False;
    }
    return ESP32_TX_Success;
}

uint8_t esp_send_from_msg(const g_msg_t* m)
{
    if (!m || !m->payload || m->len == 0) return ESP32_TX_False;
    return esp_send_frame(m->cmd_id, m->payload, m->len);//调用底层发送函数
}

// 发送四元素
void quaternion(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	g_tx_enqueue_raw(cmdID, data, len);
}


// 解析遥控传感器数据
void parseJoystick(uint8_t cmdID, uint8_t* data, uint8_t len)
{
//	float* remoter = (float*)data;  /*将 data 数组强制转换为浮点数组，其中包含4个浮点数（每个4字节）*/
//	// 把数据放到 joystickQueueHandle 这个消息队列，xyl需要在任务中写一个osMessageQueueGet的消息队列去获取遥控的值
//	
//    // osMessageQueuePut(joystickQueueHandle, (joystick_t*)data, 0, 0); 
//	// 下面是把遥控数据打印出来，便于调试，实际生产不需要打印
//	printf("Lx = %d, Ly = %d, Rx = %d, Ry = %d\r\n", (int)remoter[0], (int)remoter[1], (int)remoter[2], (int)remoter[3]);
	 (void)cmdID;

  if (len != 16) {
    // 不打印也行，避免刷屏
    return;
  }

  // 读取4个float（注意：ESP32和STM32都为小端，一般没问题）
  const float *r = (const float *)data;
  float Lx = r[0];
  float Ly = r[1];
  float Rx = r[2];
  float Ry = r[3];

  // 归一化到 [-1, 1]
  float lx = clampf(Lx / RC_RAW_MAX, -1.0f, 1.0f);
  float ly = clampf(Ly / RC_RAW_MAX, -1.0f, 1.0f);
  float rx = clampf(Rx / RC_RAW_MAX, -1.0f, 1.0f);
  float ry = clampf(Ry / RC_RAW_MAX, -1.0f, 1.0f);

  // 死区
  lx = apply_deadzone(lx, RC_DEADZONE);
  ly = apply_deadzone(ly, RC_DEADZONE);
  rx = apply_deadzone(rx, RC_DEADZONE);
  ry = apply_deadzone(ry, RC_DEADZONE);

  // 你要的“初步映射”（示例：左Y做油门，左X做yaw，右X/右Y做roll/pitch）
  rc_cmd_t cmd;
  cmd.throttle = clampf((ly + 1.0f) * 0.5f, 0.0f, 1.0f); // [-1,1] -> [0,1]
  cmd.yaw      = lx;
  cmd.roll     = rx;
  cmd.pitch    = ry;
  cmd.tick     = osKernelGetTickCount();

  // 入队：队列满时丢弃旧的，保留最新
  if (osMessageQueuePut(rcCmdQueueHandle, &cmd, 0, 0) != osOK) {
    rc_cmd_t dump;
    osMessageQueueGet(rcCmdQueueHandle, &dump, NULL, 0);
    osMessageQueuePut(rcCmdQueueHandle, &cmd, 0, 0);
  }
}

/**
 * @brief set XXX PIDparameter()函数公用的子函数，用于更新对应轴的PID增益直
 * @param 轴枚举
 * @param 
 * @retval 
*/
static void enqueue_pid_update(pid_axis_t axis, uint8_t* data, uint8_t len)
{
    if (len < sizeof(AxisPID)) 
	{
        printf("[PID] len too small: %u (need %u)\r\n",
               len, (unsigned)sizeof(AxisPID));
        return;
    }

    pid_update_msg_t msg;                     //更新某一个轴的PID参数的消息结构体
    msg.axis = axis;                          //赋值，说明msg是哪一个轴
    memcpy(&msg.pid, data, sizeof(AxisPID));  //拷贝一份，避免后面缓冲复用，data是一坨而不是一个数据

    uint32_t lock = osKernelLock();
    osStatus_t st = osMessageQueuePut(pidUpdateQueueHandle, &msg, 0, 0);
    osKernelRestoreLock(lock);

    if (st != osOK) 
	{
        printf("[PID] queue full, drop update axis=%d\r\n", axis);
    }
}

// 设置Pitch PID参数
void setPitchPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	// 进入临界段（禁止任务调度，但不禁止中断）
	//uint32_t lock = osKernelLock();
	
	// 刘力把获取到的pid数据写进flash
	enqueue_pid_update(PID_AXIS_PITCH, data, len);
	
	// 退出临界段（恢复任务调度）
	//osKernelRestoreLock(lock);
}

// 设置Roll PID参数
void setRollPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	// 进入临界段（禁止任务调度，但不禁止中断）
	//uint32_t lock = osKernelLock();
	// 刘力把获取到的pid数据写进flash
	enqueue_pid_update(PID_AXIS_ROLL, data, len);
	// 退出临界段（恢复任务调度）
	//osKernelRestoreLock(lock);
}

// 设置Yaw PID参数
void setYawPIDparameter(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	// 进入临界段（禁止任务调度，但不禁止中断）
	//uint32_t lock = osKernelLock();
	// 刘力把获取到的pid数据写进flash,
	enqueue_pid_update(PID_AXIS_YAW, data, len);
	
	// 退出临界段（恢复任务调度）
	//osKernelRestoreLock(lock);
}

// 软件复位指令
void software_restart(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	
//	// 确保所有外设操作完成
//  __disable_irq();  // 关闭全局中断，防止复位过程中被打断
//	
//	// 执行系统复位
//  NVIC_SystemReset();
}

// STM32强制暂停
void STM32pause(uint8_t cmdID, uint8_t* data, uint8_t len)
{
	int16_t sum =(STM32pause_count%3);
	printf("sum:%d\r\n",sum);
	// 进入临界段（禁止任务调度，但不禁止中断）
	uint32_t lock = osKernelLock();
	// 关闭电机
	if(STM32pause_count<5000)
	{
		if(sum==1)
		{
		printf("$\r\n");
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 10);//  200
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);//     200
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);//     200
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 10);//     200
		}
		else 
		{
		printf("#\r\n");
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);  
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); 
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		
		}
	}
	else
	{
	STM32pause_count=10;
	}
	
	STM32pause_count++;
	osKernelRestoreLock(lock);/// 退出临界区
	// 进入死循环，需要手动复位
	// 可以设置灯的闪烁样子
	//while(1);

HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
}

// 下面是第一个参数是 cmd_id, 第二个参数是解析数据函数指针, 第三个参数是函数描述
CommandList cmd_table[CommandList_MAX] =              //创建了一个结构体数组？
 {
    {0x00, STM32pause,"null"},// 命令id为0x00, ESP32发送的暂停命令, 当进入暂停命令的时候
    {0x01, NULL,"Quaternion"},// 命令id为0x01, 把四元数发送到ESP32, ESP32从而无线发送到VOFA打印出来
    {0x02, parseJoystick,  "Joystick data parse"},// 命令id为0x02, 接收ESP32回传的遥感数据
		{0x03, setPitchPIDparameter,  "set Pitch PID parameter"},// 命令id为0x03, 接收ESP32回传的PID参数
		{0x04, setRollPIDparameter,  "set Row PID parameter"},// 命令id为0x04, 接收ESP32回传的PID参数
		{0x05, setYawPIDparameter,  "set Yaw PID parameter"},// 命令id为0x05, 接收ESP32回传的PID参数
		{0x06, software_restart,  "software_restart"},// 命令id为0x06, ESP32发送软件重启命令使STM32重启
    /* 其余项留空默认是零，如果需要可以再补 */
};


/* USER CODE END 1 */
