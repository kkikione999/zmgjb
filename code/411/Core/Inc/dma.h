/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.h
  * @brief   This file contains all the function prototypes for
  *          the dma.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DMA_H__
#define __DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* DMA memory to memory transfer handles -------------------------------------*/

/* USER CODE BEGIN Includes */
#include "usart.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define DMA_RX_BUF_SIZE     1024       // DMA接收缓冲长度
#define QUEUE_BUF_SIZE      1024       // 环形队列大小
extern UART_HandleTypeDef huart2;


/* 环形队列结构体 */
typedef struct 
{
    uint8_t *buf;
    uint16_t size;
    volatile uint16_t wrIdx; // 写入索引
    volatile uint16_t rdIdx; // 读取索引
} ring_queue_t;

/* USER CODE END Private defines */

void MX_DMA_Init(void);

/* USER CODE BEGIN Prototypes */
void usart2_dma_rx_start(void);
void uart_idle_isr(void);
int uart2_rx_getc(uint8_t *out);
void usart2_dma_rx_start(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

