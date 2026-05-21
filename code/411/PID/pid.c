
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

float PID_Calculate(PID_State_t *s,
                    float kp, float ki, float kd,
                    float setpoint, float measurement,
                    float dt,
                    float out_min, float out_max,
                    float d_alpha)
{
    // 首次运行：初始化测量值，避免微分尖峰
    if (!s->initialized) {
        s->prev_measurement = measurement;
        s->prev_derivative = 0.0f;
        s->initialized = 1;
        // 仅 P 项，跳过 I 和 D
        float error = setpoint - measurement;
        float output = kp * error;
        if (output > out_max) output = out_max;
        if (output < out_min) output = out_min;
        s->output = output;
        return output;
    }

    float error = setpoint - measurement;

    // P 项
    float p_term = kp * error;

    // I 项（带 anti-windup）
    s->integral += error * dt;
    float i_term = ki * s->integral;

    // D 项（对 measurement 微分，避免 setpoint 突变尖峰）
    float raw_derivative = -(measurement - s->prev_measurement) / dt;
    // 一阶低通滤波
    float filtered_derivative = d_alpha * raw_derivative + (1.0f - d_alpha) * s->prev_derivative;
    s->prev_measurement = measurement;
    s->prev_derivative = filtered_derivative;
    float d_term = kd * filtered_derivative;

    // 计算输出
    float output = p_term + i_term + d_term;

    // Anti-windup：超限时截断积分
    if (output > out_max) {
        output = out_max;
        if (ki > 0.0f) s->integral -= error * dt;
    } else if (output < out_min) {
        output = out_min;
        if (ki > 0.0f) s->integral -= error * dt;
    }

    s->output = output;
    return output;
}

float PID_Cascaded(CascadedState_t *cs,
                   const AxisPID *params,
                   float angle_sp, float angle_meas, float rate_meas,
                   float dt,
                   float rate_out_min, float rate_out_max)
{
    // 外环（角度环）：输出作为内环的 setpoint
    // 角速度范围限制为 ±500 deg/s
    float rate_sp = PID_Calculate(&cs->angle,
                                   params->kp_angle, params->ki_angle, params->kd_angle,
                                   angle_sp, angle_meas,
                                   dt,
                                   -500.0f, 500.0f,
                                   0.5f);

    // 内环（角速度环）：输出作为电机校正量
    float output = PID_Calculate(&cs->rate,
                                  params->kp_rate, params->ki_rate, params->kd_rate,
                                  rate_sp, rate_meas,
                                  dt,
                                  rate_out_min, rate_out_max,
                                  0.5f);

    return output;
}

void PID_State_Reset(PID_State_t *s)
{
    s->integral = 0.0f;
    s->prev_measurement = 0.0f;
    s->prev_derivative = 0.0f;
    s->initialized = 0;
    s->output = 0.0f;
}

void CascadedState_Reset(CascadedState_t *cs)
{
    PID_State_Reset(&cs->angle);
    PID_State_Reset(&cs->rate);
}

void AxisCascadedState_Reset(AxisCascadedState_t *state)
{
    CascadedState_Reset(&state->roll);
    CascadedState_Reset(&state->pitch);
    CascadedState_Reset(&state->yaw);
}