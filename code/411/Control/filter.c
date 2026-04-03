#include "filter.h"
#include <stddef.h>  // 定义 NULL 宏

/**
 * @brief 初始化低通滤波器
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) return;
    
    // 限制alpha范围在[0, 1]
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    filter->alpha = alpha;
    filter->last_output = 0.0f;
}

/**
 * @brief 更新低通滤波器
 * 
 * 一阶低通滤波公式:
 * y[n] = α * x[n] + (1-α) * y[n-1]
 * 
 * 其中:
 * - x[n] 是当前输入
 * - y[n-1] 是上次输出
 * - α 是滤波系数
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input)
{
    if (filter == NULL) return input;
    
    // 一阶低通滤波
    filter->last_output = filter->alpha * input + (1.0f - filter->alpha) * filter->last_output;
    
    return filter->last_output;
}

/**
 * @brief 重置滤波器状态
 */
void LowPassFilter_Reset(LowPassFilter_t *filter)
{
    if (filter == NULL) return;
    
    filter->last_output = 0.0f;
}

/**
 * @brief 设置滤波器系数
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha)
{
    if (filter == NULL) return;
    
    // 限制alpha范围在[0, 1]
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    
    filter->alpha = alpha;
}

/**
 * @brief 获取当前滤波器系数
 */
float LowPassFilter_GetAlpha(const LowPassFilter_t *filter)
{
    if (filter == NULL) return 0.0f;
    
    return filter->alpha;
}

