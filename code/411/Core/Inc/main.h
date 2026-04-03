/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Gyroscope_EXIT_Pin GPIO_PIN_13
#define Gyroscope_EXIT_GPIO_Port GPIOC
#define Uart_ESP32_TX_Pin GPIO_PIN_2
#define Uart_ESP32_TX_GPIO_Port GPIOA
#define Uart_ESP32_RX_Pin GPIO_PIN_3
#define Uart_ESP32_RX_GPIO_Port GPIOA
#define Gyroscope_SPI_Software_NSS_Pin GPIO_PIN_4
#define Gyroscope_SPI_Software_NSS_GPIO_Port GPIOA
#define Motor3_Pin GPIO_PIN_1
#define Motor3_GPIO_Port GPIOB
#define Motor4_Pin GPIO_PIN_10
#define Motor4_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOB
#define LED_L_Pin GPIO_PIN_15
#define LED_L_GPIO_Port GPIOB
#define Motor1_Pin GPIO_PIN_8
#define Motor1_GPIO_Port GPIOA
#define Uart_computer_TX_Pin GPIO_PIN_9
#define Uart_computer_TX_GPIO_Port GPIOA
#define Uart_computer_RX_Pin GPIO_PIN_10
#define Uart_computer_RX_GPIO_Port GPIOA
#define Motor2_Pin GPIO_PIN_11
#define Motor2_GPIO_Port GPIOA
#define Barometer_SPI_Software_NSS_Pin GPIO_PIN_15
#define Barometer_SPI_Software_NSS_GPIO_Port GPIOA
#define Barometer_SPI_SCK_Pin GPIO_PIN_3
#define Barometer_SPI_SCK_GPIO_Port GPIOB
#define Barometer_SPI_MISO_Pin GPIO_PIN_4
#define Barometer_SPI_MISO_GPIO_Port GPIOB
#define Barometer_SPI_MOSI_Pin GPIO_PIN_5
#define Barometer_SPI_MOSI_GPIO_Port GPIOB
#define Barometer_EXTI_Pin GPIO_PIN_9
#define Barometer_EXTI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define RATE_5_HZ			5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
#define MAIN_LOOP_RATE 	RATE_1000_HZ
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
