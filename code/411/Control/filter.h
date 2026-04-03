#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>

/**
 * @brief 一阶低通滤波器结构体
 * 
 * 原理: y[n] = α * x[n] + (1-α) * y[n-1]
 * α越小，滤波越强，但响应越慢
 * α越大，响应越快，但滤波越弱
 * 
 * 推荐值:
 * - 陀螺仪: α = 0.3 ~ 0.5 (中等滤波)
 * - 加速度计: α = 0.1 ~ 0.2 (强滤波)
 * - 磁力计: α = 0.05 ~ 0.1 (很强滤波)
 */
typedef struct {
    float alpha;        // 滤波系数 (0-1)
    float last_output;  // 上次输出值
} LowPassFilter_t;

/**
 * @brief 初始化低通滤波器
 * @param filter 滤波器结构体指针
 * @param alpha 滤波系数 (0-1), 推荐: 陀螺仪0.4, 加速度0.15, 磁力计0.08
 */
void LowPassFilter_Init(LowPassFilter_t *filter, float alpha);

/**
 * @brief 更新低通滤波器并返回滤波后的值
 * @param filter 滤波器结构体指针
 * @param input 输入值
 * @return 滤波后的输出值
 */
float LowPassFilter_Update(LowPassFilter_t *filter, float input);

/**
 * @brief 重置滤波器状态
 * @param filter 滤波器结构体指针
 */
void LowPassFilter_Reset(LowPassFilter_t *filter);

/**
 * @brief 设置滤波器系数
 * @param filter 滤波器结构体指针
 * @param alpha 新的滤波系数 (0-1)
 */
void LowPassFilter_SetAlpha(LowPassFilter_t *filter, float alpha);

/**
 * @brief 获取当前滤波器系数
 * @param filter 滤波器结构体指针
 * @return 当前滤波系数
 */
float LowPassFilter_GetAlpha(const LowPassFilter_t *filter);

#endif // __FILTER_H__

