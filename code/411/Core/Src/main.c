/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ICM42688.h"
#include "QMC5883P.h"
#include "LPS22HBTR.h"
#include "AHRS_Mahony.h"
#include "system_params.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//宏定义、常量、类型定义
extern UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// 后期不需要测试可以删除
static inline HAL_StatusTypeDef ICM_SPI_Xfer(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t to_ms)
{

    return HAL_SPI_TransmitReceive(&hspi3, tx, rx, len, to_ms);
}
uint8_t SPI_Read_8BIT(uint8_t reg_address, uint8_t* read8bit)
{
    HAL_StatusTypeDef icm_err;
    uint8_t tx_buf[2] = { reg_address | 0x80, 0x00 };
    uint8_t rx_buf[2] = {0};
    HAL_GPIO_WritePin(Barometer_SPI_Software_NSS_GPIO_Port, Barometer_SPI_Software_NSS_Pin, GPIO_PIN_RESET);
    icm_err = ICM_SPI_Xfer(tx_buf, rx_buf, 2, 500);
    HAL_GPIO_WritePin(Barometer_SPI_Software_NSS_GPIO_Port, Barometer_SPI_Software_NSS_Pin, GPIO_PIN_SET);
    if (icm_err != HAL_OK) return 0;
    *read8bit = rx_buf[1];
    return 1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* git同步冲突测试点2 */

  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  
  /* USER CODE BEGIN 2 */
  Baro_Init();	                       // 加载flash中的PID参数
  usart2_dma_rx_start();	           // 开启DMA 接收中断
  ICM42688_init();                     // 初始化陀螺仪
  QMC_Init();
  mahony_ahrs_init(2.0f, 0.01f);    // 初始化Mahony AHRS (Kp=2.0, Ki=0.01) - 优化参数
  motor_all_start();                   //初始化四个电机

#if defined(UART1_SMOKE_TEST) && (UART1_SMOKE_TEST == 1)
  {
    static const uint8_t uart1_smoke_msg[] = "UART1 smoke test 460800\r\n";
    while (1)
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)uart1_smoke_msg, sizeof(uart1_smoke_msg) - 1U, 1000);
      HAL_Delay(500);
    }
  }
#endif

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void mag_pure_yaw_test(void)
{
    QMC5883P_Raw_Data_t raw_data, cal_data;
    float yaw_deg;
    float mag_strength_raw, mag_strength_cal;
    uint32_t last_print_time = 0;
    
    printf("\r\n");
    printf("===============================================\r\n");
    printf("        纯磁力计Yaw角测试\r\n");
    printf("===============================================\r\n\r\n");
    
    printf("测试说明：\r\n");
    printf("  - 只使用磁力计X/Y轴计算Yaw角\r\n");
    printf("  - 飞机保持水平，缓慢旋转360度\r\n");
    printf("  - 观察Yaw角是否平滑变化0~360度\r\n");
    printf("  - 按复位键退出测试\r\n\r\n");
    
    printf("开始测试...\r\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\r\n\r\n");
    
    HAL_Delay(1000);
    
    // 无限循环测试
    for(;;) {
        // 读取磁力计数据
        if(QMC_Read_Raw_DATA(&raw_data) == QMC_OK) {
            QMC_Read_Full_Calibrated_DATA(&cal_data);
            
            // 计算原始和校准后的磁场强度
            mag_strength_raw = sqrtf(raw_data.X * raw_data.X + 
                                     raw_data.Y * raw_data.Y + 
                                     raw_data.Z * raw_data.Z) * 0.000244140625f;
            
            mag_strength_cal = sqrtf(cal_data.X * cal_data.X + 
                                     cal_data.Y * cal_data.Y + 
                                     cal_data.Z * cal_data.Z) * 0.000244140625f;
            
            // 将磁力计数据转换到机体系（机体X←-MAG.Y，机体Y←MAG.X）
            float mag_x_body = (float)cal_data.X;
            float mag_y_body = -(float)cal_data.Y;

            // 使用机体系的X/Y计算Yaw角（水平面内的方位角）
            yaw_deg = atan2f(mag_y_body, mag_x_body) * 57.2957795f;  // rad to deg
            
            // 归一化到0~360度
            if(yaw_deg < 0.0f) {
                yaw_deg += 360.0f;
            }
            
            // 每200ms打印一次
            uint32_t current_time = HAL_GetTick();
            if((current_time - last_print_time) >= 200) {
                printf("原始: [%5d, %5d, %5d] %.3f G | ", 
                       raw_data.X, raw_data.Y, raw_data.Z, mag_strength_raw);
                printf("校准: [%5d, %5d, %5d] %.3f G | ", 
                       cal_data.X, cal_data.Y, cal_data.Z, mag_strength_cal);
                printf("Yaw: %6.1f°", yaw_deg);
                
                // 质量指示
                if(mag_strength_cal < 0.25f) {
                    printf(" [!] 磁场太弱");
                } else if(mag_strength_cal > 0.65f) {
                    printf(" [!] 磁场太强");
                } else {
                    printf(" [OK]");
                }
                
                printf("\r\n");
                last_print_time = current_time;
            }
        }
        
        HAL_Delay(10);
    }
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
