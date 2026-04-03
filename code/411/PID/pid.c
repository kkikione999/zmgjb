
#include "system_params.h"
#include "usart.h"     // AxisPID / CascadedPIDParams
#include "pid.h"

extern CascadedPIDParams g_runtime_pid;                               //真正控制环用的 PID
extern system_params_t g_system_params;
void bank_to_axisPID(int bank_idx, AxisPID *pid)
{
    float *slot = g_system_params.bank[bank_idx];

    pid->kp_angle = slot[PID_IDX_KP_ANGLE];
    pid->ki_angle = slot[PID_IDX_KI_ANGLE];
    pid->kd_angle = slot[PID_IDX_KD_ANGLE];
    pid->kp_rate  = slot[PID_IDX_KP_RATE ];
    pid->ki_rate  = slot[PID_IDX_KI_RATE ];
    pid->kd_rate  = slot[PID_IDX_KD_RATE ];
}

/* 启动时调用一次，把 Flash 中的值搬到运行时 */
void PID_LoadFromParams(void)
{
    bank_to_axisPID(PID_BANK_PITCH, &g_runtime_pid.pitch);
    bank_to_axisPID(PID_BANK_ROLL,  &g_runtime_pid.roll);
    bank_to_axisPID(PID_BANK_YAW,   &g_runtime_pid.yaw);
}

// 把一个 AxisPID 写入指定的 bank[x][0..5]
void axisPID_to_bank_slot(float *slot, const AxisPID *pid)
{
    slot[PID_IDX_KP_ANGLE] = pid->kp_angle;//注意，这个传入的slot形参本身就已经被定位在g_system_params.bank的物理地址那儿了
    slot[PID_IDX_KI_ANGLE] = pid->ki_angle;//注意，此处操作 slot[i] 本质是 (slot + i)。下面的四句话同理
    slot[PID_IDX_KD_ANGLE] = pid->kd_angle;
    slot[PID_IDX_KP_RATE ] = pid->kp_rate;
    slot[PID_IDX_KI_RATE ] = pid->ki_rate;
    slot[PID_IDX_KD_RATE ] = pid->kd_rate;
}