/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <cmsis_os2.h>  // 添加CMSIS-RTOS2头文件
#include "IMUsolution.h"
#include "ICM42688.h"
#include "QMC5883P.h"
#include "LPS22HBTR.h"
#include "AHRS_Mahony.h"
#include "usart.h"
#include "dma.h"
#include "pid.h"
#include "system_params.h"
#include "filter.h"  // 低通滤波器
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "tim.h"
/* USER CODE END Includes */

/* -------------------------------------Private typedef ---------------------------------------------------*/
/* USER CODE BEGIN PTD */
//宏定义、常量、类型定义
typedef struct                   // 定义传感器数据结构体（修正字段名）
{
    ICM42688_Raw_Data_t icm;     // 加速度计和陀螺仪数据
    QMC5883P_Raw_Data_t qmc;     // 磁力计数据
    int32_t Pressure;            // 气压计数据
} sensor_data_t;

extern system_params_t g_system_params;
extern CascadedPIDParams g_runtime_pid ;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;  // VOFA+使用UART1
extern uint8_t USART1_DMA_TX_ready;
// extern ring_queue_t queue_rx;   //不再使用它了
extern uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
extern CommandList cmd_table[CommandList_MAX];
extern lps22hb_t g_lps;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
//变量声明、函数原型声明
float pitch, row, yaw;
uint32_t err_times = 0;
int32_t lps_value;
HAL_UART_StateTypeDef uart_status;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define QUICK_TEST 0
#define PWM_MIN   0
#define PWM_MAX   80 //根据真实PWM周期改这个范围（比如ARR=1000/2000等）1000-》80
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
ICM42688_Raw_Data_t icm_raw_data;
QMC5883P_Raw_Data_t qmc_raw_data;
/* USER CODE END Variables */
/* Definitions for POSTURE */
/* osThreadAttr_t 结构体和 osPriority_t 枚举属于 CMSIS-RTOS2 标准接口 */
osThreadId_t POSTUREHandle;
const osThreadAttr_t POSTURE_attributes = {
  .name = "POSTURE",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SENSOR */
osThreadId_t SENSORHandle;											  //声明了一个任务句柄变量，用于后续操作该任务（例如启动、挂起、删除任务）。
const osThreadAttr_t SENSOR_attributes =                              //定义了任务的属性结构体，包含以下关键配置
	{
  .name = "SENSOR",                                                   //任务名称标识
  .stack_size = 450 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ESP32_RX */
osThreadId_t ESP32_RXHandle;
const osThreadAttr_t ESP32_RX_attributes = {
  .name = "ESP32_RX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal ,
};
/* Definitions for ESP32_TX */
osThreadId_t ESP32_TXHandle;
const osThreadAttr_t ESP32_TX_attributes = {
  .name = "ESP32_TX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t PID_UpdateHandle;
const osThreadAttr_t PID_Update_attributes = {
  .name       = "PID_Update",
  .stack_size = 512 * 4,                    // 2KB 避免栈溢出
  .priority   = (osPriority_t)osPriorityLow // 和 ESP32_RX 差不多就行
};

osThreadId_t MANEUVERHandle;
const osThreadAttr_t MANEUVER_attributes = {
  .name = "MANEUVER",
  .stack_size = 512 * 4,              // 先给 2KB，后面再按水位调小
  .priority = (osPriority_t)osPriorityAboveNormal,  // 控制输出建议略高于RX/PID保存
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
// 全局tx消息队列
osMessageQueueId_t g_txQueueHandle;
const osMessageQueueAttr_t g_txQueue_attributes = {
  .name = "globle_tx"
};

osMessageQueueId_t g_rxQueueHandle;
const osMessageQueueAttr_t g_rxQueue_attributes = {
  .name = "globle_rx"
};

osMessageQueueId_t joystickQueueHandle;
const osMessageQueueAttr_t joystickQueue_attributes = {
  .name = "joystick"
};

// 全局传感器数据队列句柄
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
    .name = "sensor_data_queue"
};

osMessageQueueId_t pidUpdateQueueHandle;             //用于PID参数的更新
const osMessageQueueAttr_t pidUpdateQueue_attributes = {
  .name = "pid_update"
};

osMessageQueueId_t rcCmdQueueHandle;
const osMessageQueueAttr_t rcCmdQueue_attributes = {
  .name = "rc_cmd"
};

/* USER CODE END FunctionPrototypes */

void posture_task(void *argument);
void sensor_task(void *argument);
void esp32_rx_task(void *argument);
void esp32_tx_task(void *argument);
void pid_update_task(void *argument);
void maneuver_task(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

static inline uint16_t pwm_from_01(float t01)
{
  if (t01 < 0) t01 = 0;
  if (t01 > 1) t01 = 1;
  return (uint16_t)(PWM_MIN + t01 * (PWM_MAX - PWM_MIN));
}

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) 
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  g_txQueueHandle = osMessageQueueNew (32, sizeof(g_msg_t), 
	  &g_txQueue_attributes);
  g_rxQueueHandle = osMessageQueueNew (32, sizeof(g_msg_t), 
	  &g_rxQueue_attributes);
  //joystickQueueHandle = osMessageQueueNew(16, sizeof(joystick_t), &g_rxQueue_attributes);
  joystickQueueHandle = osMessageQueueNew(16, sizeof(joystick_t), 
	  &joystickQueue_attributes);	
  sensorQueueHandle = osMessageQueueNew(10, sizeof(sensor_data_t), 
	  &sensorQueue_attributes); // 创建全局传感器数据队列
  pidUpdateQueueHandle = osMessageQueueNew(6, sizeof(pid_update_msg_t), 
	  &pidUpdateQueue_attributes);
  rcCmdQueueHandle = osMessageQueueNew(8, sizeof(rc_cmd_t), 
	  &rcCmdQueue_attributes);//遥控是“最新值覆盖旧值”的典型场景，没必要堆很多深度
 
	/* USER CODE END RTOS_QUEUES */
  /* Create the thread(s) */
  /* creation of POSTURE */
  POSTUREHandle = osThreadNew(posture_task, NULL, &POSTURE_attributes);
  /* creation of SENSOR */
  SENSORHandle = osThreadNew(sensor_task, NULL, &SENSOR_attributes);
  /* creation of ESP32_RX */
  ESP32_RXHandle = osThreadNew(esp32_rx_task, NULL, &ESP32_RX_attributes);
  /* creation of ESP32_TX */
//  ESP32_TXHandle = osThreadNew(esp32_tx_task, NULL, &ESP32_TX_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
 // PID_UpdateHandle = osThreadNew(pid_update_task, NULL, &PID_Update_attributes);//赋值，赋一个世纪的直
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  MANEUVERHandle = osThreadNew(maneuver_task, NULL, &MANEUVER_attributes);
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_posture_task */
/**
 * @brief 用于更新——运行时该用什么样的PID参数呢？通过这个任务更新
 * @param 
 * @param 
 * @retval 
*/
void pid_update_task(void *argument)
{
    pid_update_msg_t msg_from_queue;//消息结构体变量，用于接收从队列中获取的PID更新消息

    for (;;)/*所以，这个任务大部分时间是在“睡觉”的。 只有当你通过串口发 PID 参数、setXXXPIDparameter 往队列里塞了一条消息，它才会醒一次。*/
    {
        if (osMessageQueueGet(pidUpdateQueueHandle, &msg_from_queue, NULL, 10) == osOK)//从队列拿ESp32发来的新参数
        {
            // 1）更新运行时 PID
            AxisPID *runtime_pid = NULL;   // (里面是一个轴的六个增益)
            float   *flash_slot  = NULL;   // 指向 g_system_params 里对应位置

            switch (msg_from_queue.axis)//根据“这条消息是哪一个轴的”，把“待会需要修改的运行时 PID 地址”和“待会要写进 Flash 数组的起始地址”都计算出来。
            {
            case PID_AXIS_ROLL:
                runtime_pid = &g_runtime_pid.roll;//也就是说运行时PID先由这个g_runtime_pid提供，这有必要吗？g_system_params结构体不是已经有用于储存轴增益的数组了吗？
			    flash_slot  = &g_system_params.bank[PID_BANK_ROLL][0];//将flash_slot赋值为g_system_params结构体对应轴的增益数组初始位地址
                break;

            case PID_AXIS_PITCH:
                runtime_pid = &g_runtime_pid.pitch;
                flash_slot  =  &g_system_params.bank[PID_BANK_PITCH][0];
                break;

            case PID_AXIS_YAW:
                runtime_pid = &g_runtime_pid.yaw;
                flash_slot  = &g_system_params.bank[PID_BANK_YAW][0];
                break;

            default:
                break;
            }

            if (runtime_pid && flash_slot)/*如果上面的 switch 没有匹配到任何一个 case，两个指针会保持为 NULL；那肯定说明 msg.axis 是无效的值，继续下面的代码就会野指针；
                                           所以这里做了一层防御性检查：只要有一个为空，就直接跳过这次循环。*/
            {
                *runtime_pid = msg_from_queue.pid;						// 更新运行时 PID
                axisPID_to_bank_slot(flash_slot, &msg_from_queue.pid);  // 映射到 g_system_params 中（如果你有更直观的字段，这里改成赋值结构体即可）
                if (!SystemParams_Save())
                {
                    printf("[PID] SystemParams_Save FAILED!\r\n");
                }
                else
                {
                    printf("[PID] Updated axis=%d: "
                           "Ang(P=%.3f,I=%.3f,D=%.3f)  "
                           "Rate(P=%.3f,I=%.3f,D=%.3f)\r\n",
                           msg_from_queue.axis,
                           msg_from_queue.pid.kp_angle, msg_from_queue.pid.ki_angle, msg_from_queue.pid.kd_angle,
                           msg_from_queue.pid.kp_rate,  msg_from_queue.pid.ki_rate, msg_from_queue.pid.kd_rate);
                }
            }
        }
	//printf("p\r\n");
	//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);
	osDelay(1000);
    }
}
/**
  * @brief  姿态解算
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_posture_task */
void posture_task(void *argument)
{
  /* USER CODE BEGIN posture_task */
	sensor_data_t sensor_packet;
	float roll, pitch, yaw;  // 欧拉角

	printf("[POSTURE] Task started - AHRS enabled\r\n");

  /* Infinite loop */
  for(;;)
  {
		// 从传感器队列获取滤波后的数据
		if(osMessageQueueGet(sensorQueueHandle, &sensor_packet, 0, 5) == osOK)
		{
			// ========== 调用Mahony AHRS进行姿态解算 ==========
			// 将完整ICM数据转换为AHRS需要的分离结构体
			ICM42688_Acc_Raw_Data_t acc_data = {
				.accel_x = sensor_packet.icm.accel_x,
				.accel_y = sensor_packet.icm.accel_y,
				.accel_z = sensor_packet.icm.accel_z
			};
			ICM42688_Gyro_Raw_Data_t gyro_data = {
				.gyro_x = sensor_packet.icm.gyro_x,
				.gyro_y = sensor_packet.icm.gyro_y,
				.gyro_z = sensor_packet.icm.gyro_z
			};

			// 调用AHRS更新（包含磁力计）
			mahony_ahrs_update_mag(&acc_data, &gyro_data, &sensor_packet.qmc);

			// 获取欧拉角
			mahony_get_euler(&roll, &pitch, &yaw);

			// 打印姿态角（可选择启用）
			// printf("R: %.1f P: %.1f Y: %.1f\r\n", roll, pitch, yaw);

			// ========== 原有代码（已注释，可选择性启用）==========
			#if 0
			// 使用传感器数据进行姿态解算（这里需要调用实际的姿态解算函数）
			// 示例：MahonyAHRSupdate(gyro, accel, mag);
			quat_moni.q0 =sensor_packet.icm.accel_x;
			quat_moni.q1 =sensor_packet.icm.accel_y;
			quat_moni.q2 =sensor_packet.icm.accel_z;
			quat_moni.q3 =sensor_packet.icm.gyro_x;
			quat_moni.q4 =sensor_packet.icm.gyro_y;
			quat_moni.q5=sensor_packet.icm.gyro_z;
			quat_moni.q6=sensor_packet.qmc.X;
			quat_moni.q7=sensor_packet.qmc.Y;
			quat_moni.q8=sensor_packet.qmc.Z;
			quat_moni.q9=sensor_packet.Pressure;
			char buffer[256];
            snprintf(buffer, sizeof(buffer), "icm.accel : %.3f,%.3f,%.3f,\r\n,icm.gyro : %.3f,%.3f,%.3f,\r\n,qmc : %.3f,%.3f,%.3f,\r\n Pre : %d\r\n",
                     quat_moni.q0, quat_moni.q1, quat_moni.q2, quat_moni.q3,quat_moni.q4,quat_moni.q5,quat_moni.q6,quat_moni.q7,quat_moni.q8,quat_moni.q9);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
			HAL_UART_Transmit(&huart1,"posture_task\r\n",15,10);
			// 发送姿态数据到VOFA
			//g_tx_enqueue_raw(0x01, (uint8_t*)&quat, sizeof(quat));
			#endif
		}
		osDelay(20);  // 50Hz发送率，适合VOFA+显示
  }
  /* USER CODE END posture_task */
}

/* USER CODE BEGIN Header_sensor_task */
/**
* @brief 读取传感器数据
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_task */
void sensor_task(void *argument)
{
  /* USER CODE BEGIN sensor_task */
  /* Infinite loop */
	// 代码思路：	在sensor_task这个任务中读取传感器数据
	// 						然后通过消息队列去给不同的任务传递数据
	
	// ========== 初始化低通滤波器 ==========
	// 陀螺仪滤波器 (中等滤波, alpha=0.4)
	LowPassFilter_t gyro_x_filter, gyro_y_filter, gyro_z_filter;
	LowPassFilter_Init(&gyro_x_filter, 0.4f);
	LowPassFilter_Init(&gyro_y_filter, 0.4f);
	LowPassFilter_Init(&gyro_z_filter, 0.4f);
	
	// 加速度计滤波器 (强滤波, alpha=0.15)
	LowPassFilter_t accel_x_filter, accel_y_filter, accel_z_filter;
	LowPassFilter_Init(&accel_x_filter, 0.15f);
	LowPassFilter_Init(&accel_y_filter, 0.15f);
	LowPassFilter_Init(&accel_z_filter, 0.15f);
	
	// 磁力计滤波器 (很强滤波, alpha=0.08)
	LowPassFilter_t mag_x_filter, mag_y_filter, mag_z_filter;
	LowPassFilter_Init(&mag_x_filter, 0.08f); 
	LowPassFilter_Init(&mag_y_filter, 0.08f);
	LowPassFilter_Init(&mag_z_filter, 0.08f);
	
	printf("[SENSOR] 滤波器初始化完成\r\n");
	printf("[SENSOR] 陀螺仪滤波系数: %.2f\r\n", 0.4f);
	printf("[SENSOR] 加速度计滤波系数: %.2f\r\n", 0.15f);
	printf("[SENSOR] 磁力计滤波系数: %.2f\r\n", 0.08f);
	
  for(;;)
  {
	// ========== 1. 读取ICM42688原始数据 ==========
	// ICM_GET_Average_Raw_data已经做了3次采样平均，相当于硬件滤波
	ICM_GET_Average_Raw_data(&icm_raw_data, 3); 
	
	// ========== 2. 应用低通滤波 ==========
	// 陀螺仪滤波
	float gyro_x_filtered = LowPassFilter_Update(&gyro_x_filter, 
		(float)icm_raw_data.gyro_x);
	float gyro_y_filtered = LowPassFilter_Update(&gyro_y_filter, 
		(float)icm_raw_data.gyro_y);
	float gyro_z_filtered = LowPassFilter_Update(&gyro_z_filter,
		(float)icm_raw_data.gyro_z);
	
	// 加速度计滤波
	float accel_x_filtered = LowPassFilter_Update(&accel_x_filter,
		(float)icm_raw_data.accel_x);
	float accel_y_filtered = LowPassFilter_Update(&accel_y_filter,
		(float)icm_raw_data.accel_y);
	float accel_z_filtered = LowPassFilter_Update(&accel_z_filter,
		(float)icm_raw_data.accel_z);
	
	// ========== 3. 读取磁力计（使用完整校准） ==========
	// 优先使用完整校准函数（硬铁+软铁），如果没有校准则使用原始数据
	QMC_StatusTypeDef qmc_status = QMC_Read_Full_Calibrated_DATA(&qmc_raw_data);
	if(qmc_status != QMC_OK) 
	{
		// 如果完整校准读取失败，尝试使用基础校准
		qmc_status = QMC_Read_Calibrated_Raw_DATA(&qmc_raw_data);
		if(qmc_status != QMC_OK) 
		{
			// 如果基础校准也失败，使用原始数据
			QMC_Read_Raw_DATA(&qmc_raw_data);
		}
	}
	
	// 磁力计滤波
	float mag_x_filtered = LowPassFilter_Update(&mag_x_filter, (float)qmc_raw_data.X);
	float mag_y_filtered = LowPassFilter_Update(&mag_y_filter, (float)qmc_raw_data.Y);
	float mag_z_filtered = LowPassFilter_Update(&mag_z_filter, (float)qmc_raw_data.Z);
	
	// ========== 4. 读取气压计 ==========
    LPS22HB_ReadRawPressure(&g_lps, &lps_value);
    
    // ========== 5. 打包滤波后的传感器数据 ==========
 
    sensor_data_t sensor_packet = 
	{
        .icm = {
			.accel_x = (int16_t)accel_x_filtered,
			.accel_y = (int16_t)accel_y_filtered,
			.accel_z = (int16_t)accel_z_filtered,
			.gyro_x = (int16_t)gyro_x_filtered,
			.gyro_y = (int16_t)gyro_y_filtered,
			.gyro_z = (int16_t)gyro_z_filtered
		},
        .qmc = {
			.X = (int16_t)mag_x_filtered,
			.Y = (int16_t)mag_y_filtered,
			.Z = (int16_t)mag_z_filtered
		},
        .Pressure = lps_value
    };
    
    // ========== 6. 发送到传感器队列 ==========
    osMessageQueuePut(sensorQueueHandle, &sensor_packet, 0, 0);
    
    // ========== 7. 周期延时 ==========
    osDelay(18); // 18ms = 55.5Hz更新率
	printf("sensor_task\r\n");
  }
  /* USER CODE END sensor_task */
}

/* USER CODE BEGIN Header_esp32_rx_task */
/**
* @brief 接收ESP32的串口数据, 比如接收传来的PID参数
* @param argument: Not used
* @retval None
* @introduce 从 queue_rx 里一字节一字节读出数据，按自定义协议拼成数据帧，校验后
根据 CMD 分发给对应的处理函数。
注意！！！由于DMA的缓冲区大小有限，本任务的osDelay()不能太低，否则会导致来不及处理
缓冲区里面最新的字节(因为会被esp32发来的数据覆盖掉)
*/
/* USER CODE END Header_esp32_rx_task */
void esp32_rx_task(void *argument)
{
  /* USER CODE BEGIN esp32_rx_task */
enum { S_WAIT_H, S_CMD, S_LEN, S_DATA, S_SUM, S_TAIL 
	}st = S_WAIT_H;//定义枚举变量，并且赋初始值

    uint8_t  cmd = 0, len = 0, sum_rx = 0;
    uint8_t  data[(MAX_FRAME_LEN > 200) ? 200 : MAX_FRAME_LEN];
    uint8_t  pos = 0;
    uint16_t sum_calc = 0;

    // 1s 心跳（可选）
    uint32_t last_beat = osKernelGetTickCount();

    for (;;)
    {
        uint8_t b;
        int got = 0;

        // 单次最多处理 N 个字节，避免饿死其他任务
        uint16_t budget = 256;

        while (budget-- && uart2_rx_getc(&b))//如果还在单次允许处理的最大字
		//节量内，并且有新的字节返回
        {
            got = 1;
		
            switch (st)
            {
            case S_WAIT_H:
                if (b == FRAME_HEADER) {
                    st = S_CMD;
                    pos = 0;
                    sum_calc = 0;
                }
                break;

            case S_CMD:
                cmd = b;
		//printf("cmd:%d\r\n",cmd);
                sum_calc = cmd;
                st = S_LEN;
                break;

            case S_LEN:
                len = b;
                sum_calc += len;

                if (len > sizeof(data)) {
                    st = S_WAIT_H;
                    pos = 0;
                    break;
                }
                pos = 0;
                if (len == 0) {
                    // 数据长度为0，直接跳转到校验和状态
                    st = S_SUM;
                } else {
                    st = S_DATA;
                }
                break;

            case S_DATA:
                data[pos++] = b;
                sum_calc += b;
                if (pos >= len) st = S_SUM;
                break;

            case S_SUM:
                sum_rx = b;
                st = S_TAIL;
                break;

            case S_TAIL:
                if (b != FRAME_TAIL) 
		{
                    st = S_WAIT_H;
                    pos = 0;
                    break;
                }
                if (((uint8_t)sum_calc) != sum_rx) {
                    st = S_WAIT_H;
                    pos = 0;
                    break;
                }
		printf("[RX_OK] cmd=%u len=%u\r\n", cmd, len);
                if (cmd < CommandList_MAX && cmd_table[cmd].func_handler) 
		{
                    cmd_table[cmd].func_handler(cmd, data, len);
			//printf("cmd:%d\r\n",cmd);
                }
                st = S_WAIT_H;
                pos = 0;
                break;

            default:
                st = S_WAIT_H;
                pos = 0;
                break;
            }
        }

        // 心跳：每秒一次（可选）
        uint32_t now = osKernelGetTickCount();
        if ((now - last_beat) >= 1000) {
            // printf("esp32_rx_task alive\r\n"); // 建议先别开，或改成计数输出
            last_beat = now;
        }

        // 没拿到数据就睡一下；拿到数据但预算用完了，也让出一下 CPU
        if (!got) {
            osDelay(1);
        } else {
            osDelay(0); // 或 taskYIELD(); 让其他同优先级任务跑一下
        }
	//HAL_UART_Transmit_DMA(&huart1,"&",1);
    }
  /* USER CODE END esp32_rx_task */
}

/* USER CODE BEGIN Header_esp32_tx_task */
/**
* @brief 把数据通过串口发送给ESP32
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_esp32_tx_task */
void esp32_tx_task(void *argument)
{
  /* USER CODE BEGIN esp32_tx_task */
	g_msg_t msg;// 定义一个全局串口消息的结构体变量
  for(;;)
  {
		// 测试接收端 esp32_rx_task 所以把下面的数据注释了
		if(osMessageQueueGet(g_txQueueHandle, &msg, 0, 0) == osOK)//从全局串口消息队列中获取一个消息
		{
			if (esp_send_from_msg(&msg) != ESP32_TX_Success)
			{
				printf("esp send error\r\n");
			}
			// 每次获取完队列都要释放内存
			free_frame_slot(&msg);
		}
		printf("esp32_tx_task\r\n");
		osDelay(3000);//这里能不能再降？目前数据流动慢，vofa延迟高
  }
  /* USER CODE END esp32_tx_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void maneuver_task(void *argument)
{
  rc_cmd_t rc = {0};
  uint32_t last_rc_tick = 0;

  for (;;)
  {
    // 1) 拉最新遥控（阻塞最多 1ms）
    rc_cmd_t tmp;
    if (osMessageQueueGet(rcCmdQueueHandle, &tmp, NULL, 1) == osOK) {
      rc = tmp;
      last_rc_tick = rc.tick;
    }

    // 2) 拉最新传感器（非阻塞：有就更新，没有就用上次）
    // 3) 遥控失联保护：200ms 没新指令，就把油门降到0（或怠速）
    uint32_t now = osKernelGetTickCount();
    if ((now - last_rc_tick) > 200) {
      rc.throttle = 0.0f;
      rc.roll = rc.pitch = rc.yaw = 0.0f;
    }

    // 4) 融合/机动逻辑（先给一个最简单版本）
    // 这里你可以用 s.icm.gyro_x/y/z、姿态角 pitch/roll/yaw 做闭环
    // 先用“指令直通”做混控，后面再叠加姿态稳定
    float T = rc.throttle;            // 0~1
    float R = rc.roll * 0.2f;         // 混控系数先小一点，避免暴力
    float P = rc.pitch * 0.2f;
    float Y = rc.yaw * 0.2f;

    // 5) X型混控（示例：M1前左 M2前右 M3后右 M4后左）
    float m1 = T + P + R + Y;
    float m2 = T + P - R - Y;
    float m3 = T - P - R + Y;
    float m4 = T - P + R - Y;

    // 限幅到 [0,1]
    m1 = (m1 < 0) ? 0 : (m1 > 1 ? 1 : m1);
    m2 = (m2 < 0) ? 0 : (m2 > 1 ? 1 : m2);
    m3 = (m3 < 0) ? 0 : (m3 > 1 ? 1 : m3);
    m4 = (m4 < 0) ? 0 : (m4 > 1 ? 1 : m4);

    // 6) 输出PWM（把TIM/通道改成你实际四电机）
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm_from_01(m1));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_from_01(m2));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_from_01(m3));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_from_01(m4));

    // 7) 控制周期：5ms = 200Hz（可按你系统调）
    osDelay(20);
  }
}
/* USER CODE END Application */
