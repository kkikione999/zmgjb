#ifndef __PID_H__
#define __PID_H__
//#include "system_params.h"
// -------- PID 在 bank 中的布局定义 ----------------------------------------------------
#define PID_BANK_PITCH   0
#define PID_BANK_ROLL    1
#define PID_BANK_YAW     2
// 在每个 bank 内部，AxisPID 各个项的下标
#define PID_IDX_KP_ANGLE 0
#define PID_IDX_KI_ANGLE 1
#define PID_IDX_KD_ANGLE 2
#define PID_IDX_KP_RATE  3
#define PID_IDX_KI_RATE  4
#define PID_IDX_KD_RATE  5

// -------- 你可以按需修改的容量参数 --------
#ifndef PARAM_ARRAY_SIZE
#define PARAM_ARRAY_SIZE   50
#endif

#ifndef NUM_ARRAY_BANKS
#define NUM_ARRAY_BANKS    6   // 6 组数组
#endif


//typedef   __packed struct
typedef   struct
{
    float pid[3];             // 例：PID
    float cal[3];             // 例：校准
    float bank[NUM_ARRAY_BANKS][PARAM_ARRAY_SIZE]; // 6×N 的浮点数组,Axis_PID结构体变量中的双环PID增益非常适合映射到此处
} system_params_t;
// -------- PID 在 bank 中的布局定义 END  -----------------------------------------------

//轴枚举+队列消息结构体，一次不一定跟新三个轴
typedef enum{
    PID_AXIS_ROLL  = 0,
    PID_AXIS_PITCH = 1,
    PID_AXIS_YAW   = 2,
} pid_axis_t;

// 单轴 PID 参数结构体
typedef struct {
    float kp_angle;  // 角度环 P 增益
    float ki_angle;  // 角度环 I 增益
    float kd_angle;  // 角度环 D 增益

    float kp_rate;   // 角速度环 P 增益
    float ki_rate;   // 角速度环 I 增益
    float kd_rate;   // 角速度环 D 增益
} AxisPID;

//消息结构体变量，用于接收从队列中获取的PID更新消息
typedef struct 
{
    pid_axis_t axis;//哪个轴》？
    AxisPID    pid;//对应的PID参数
} pid_update_msg_t;

// 三轴串联 PID 参数总结构体
typedef struct 
{
    AxisPID roll;   // 横滚轴
    AxisPID pitch;  // 俯仰轴
    AxisPID yaw;    // 偏航轴
} CascadedPIDParams;

void axisPID_to_bank_slot(float *slot, const AxisPID *pid);
void bank_to_axisPID(int bank_idx, AxisPID *pid);
void PID_LoadFromParams(void);
#endif