#ifndef IMUSOLUTION_H__
#define IMUSOLUTION_H__
#include <stdint.h>   // ��������
#include <math.h>
#include "main.h"

typedef struct icm42688_raw_data_t ICM42688_Raw_Data_t;
typedef struct icm42688_acc_raw_data_t ICM42688_Acc_Raw_Data_t;
typedef struct icm42688_gyro_raw_data_t ICM42688_Gyro_Raw_Data_t;



// ========== �㷨ѡ�� ==========
typedef enum {
    IMU_ALGO_COMPLEMENTARY_PI = 0, // ʹ�� IMUupdate()��PI ���������˲���
    IMU_ALGO_COMPLEMENTARY_V2 = 1  // ʹ�� algorithm()����ĵڶ��滥���˲���
} IMUFusionAlgo;

typedef struct imusloution_init
{
	//��̬ƫ��
	int16_t static_gx;
	int16_t static_gy;
	int16_t static_gz;
	int16_t static_ax;
	int16_t static_ay;
	int16_t static_az;
	IMUFusionAlgo algo;
	float gyro_resolution;
	float accel_resolution;
}IMUSOLUTION_INIT_DATA;

typedef struct Quaternion{
	float q0;
	float q1;
	float q2;
	float q3;
}quat_st;//
typedef struct Quaternion_moni{
	float q0;
	float q1;
	float q2;
	float q3;
	float q4;
	float q5;
	float q6;
	float q7;	
	float q8;
	float q9;
	float q10;	
}quat_st_moni;//这个结构体存在是因为posture任务中还没有引入具体的姿态融合过程，所
//以现在直接用这个结构体把加速度、角度、磁力数据发出去，等后面解算好四元数之后还用上面
//的那个结构体


// ========== API ==========
void imu_fusion_init(IMUSOLUTION_INIT_DATA *imu_init_data);

void imu_fusion_reset(void);

/** ι��һ֡ԭʼ���ݣ�int16�����ڲ��Զ�ʹ�����ȫ�ֱַ����뵥λת�� */
void imu_fusion_process(const ICM42688_Acc_Raw_Data_t *accraw, const ICM42688_Gyro_Raw_Data_t *gyroraw);

/** ��ȡŷ���ǣ���λ��deg�� */
void imu_fusion_get_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

/** ��ȡ��Ԫ������λ���� */
void imu_fusion_get_quaternion(float *oq0, float *oq1, float *oq2, float *oq3);

#endif 
