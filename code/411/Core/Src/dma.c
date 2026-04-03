/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "dma.h"
/* USER CODE BEGIN 0 */
#include "usart.h"
/* USER CODE END 0 */
/* USER CODE BEGIN 1 */

///* 环形队列缓冲 */
//uint8_t queue_buf[QUEUE_BUF_SIZE];

//ring_queue_t queue_rx = 
//{
//    .buf = queue_buf,
//    .size = QUEUE_BUF_SIZE,
//    .wrIdx = 0,                  // 写入索引
//    .rdIdx = 0,                  // 读取索引
//};//定义的环形队列，DMA中断和空闲终端中断不断往里面写入数据<uart_idle_isr()116行和dmarx_*_isr()>

uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];     // DMA原始缓冲
//uint16_t last_dma_rx_size = 0;         // 上次处理结束的位置
/* USER CODE END 1 */

void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USER CODE BEGIN 2 */

static uint16_t uart2_rd =0 ;

/**
 * @brief 计算 UART2 DMA 接收数据量
 * @retval返回已接收的数据量
*/
static inline uint16_t uart2_dma_wr(void)
{
    uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart2.hdmarx);/*获取DMA传输
	剩余的字节数*/  
    return (uint16_t)(DMA_RX_BUF_SIZE - ndtr);           /*返回已经传输
	的字节数*/
}

// 有新字节返回1，否则0
int uart2_rx_getc(uint8_t *out)
{
    uint16_t wr = uart2_dma_wr();
    if (uart2_rd == wr) return 0;

    *out = dma_rx_buf[uart2_rd++];
    if (uart2_rd >= DMA_RX_BUF_SIZE) uart2_rd = 0;
    return 1;
}

/* -------------------- 初始化与启动 -------------------- */
void usart2_dma_rx_start(void)
{
    // 启动DMA循环接收
    HAL_UART_Receive_DMA(&huart2, dma_rx_buf, DMA_RX_BUF_SIZE);

    // 开启空闲中断
    //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    // 开启DMA半传中断
    //__HAL_DMA_ENABLE_IT(huart2.hdmarx, DMA_IT_HT);
}
static __attribute__((unused)) uint16_t uart_get_dma_rx_buf_remain_size(void)
{
    // HAL宏的封装版（返回DMA剩余未用字节数）
    return __HAL_DMA_GET_COUNTER(huart2.hdmarx);
}

/* 写入环形队列 */
//static void write_block_queue(ring_queue_t *q, const uint8_t *data, uint16_t len)
//{
//    while (len--) {
//        q->buf[q->wrIdx++] = *data++;
//        if (q->wrIdx >= q->size) q->wrIdx = 0; // 回环
//    }
//}

/* -------------------- DMA事件处理函数 -------------------- */
/* 全传输完成：DMA指针绕回0 */
//static void dmarx_done_isr(void)
//{
//    uint16_t handle_size = DMA_RX_BUF_SIZE - last_dma_rx_size;
//    write_block_queue(&queue_rx, &dma_rx_buf[last_dma_rx_size], handle_size);
//    last_dma_rx_size = 0;
//}

/* 半传输完成：到达512位置 */
//static void dmarx_half_done_isr(void)
//{
//    uint16_t recv_total_size = DMA_RX_BUF_SIZE - uart_get_dma_rx_buf_remain_size();
//    uint16_t handle_size = recv_total_size - last_dma_rx_size;
//    write_block_queue(&queue_rx, &dma_rx_buf[last_dma_rx_size], handle_size);
//    last_dma_rx_size = recv_total_size;
//}

/* 串口空闲中断：一帧数据间隙 */
void uart_idle_isr(void)
{
//    uint16_t recv_total_size = DMA_RX_BUF_SIZE - uart_get_dma_rx_buf_remain_size();
//    uint16_t handle_size = recv_total_size - last_dma_rx_size;
//    write_block_queue(&queue_rx, &dma_rx_buf[last_dma_rx_size], handle_size);
//    last_dma_rx_size = recv_total_size;
}


void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {}
    //dmarx_half_done_isr();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {}
    //dmarx_done_isr();
}

/* USER CODE END 2 */

