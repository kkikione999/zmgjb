# Change Log

## 2026-05-14 — 阶段 2：PID 计算引擎 + 级联控制闭环

### 阶段 1（电机标定）结果
- 4 电机转向正确：M1(FL)=CCW, M2(FR)=CW, M3(RR)=CCW, M4(RL)=CW，对角同向
- 最低起步 PWM ≈ 80（ARR=999），起飞需 PWM ≥ 500
- 混控方向验证通过：Roll / Pitch / Yaw 均正确

### 阶段 1 改动
- `usart.c/h`：新增 USART1 中断接收 + 文本命令解析（m1-m4, all, stop, mix, sweep）
- `dma.c`：`HAL_UART_RxCpltCallback` 新增 USART1 分支调用 `usart1_rx_byte_handler()`
- `freertos.c`：maneuver_task 新增 `g_motor_test_active` 跳过逻辑，AHRS printf 临时关闭
- `scripts/motor_test.py`：Python 交互式电机测试工具（pyserial, 自动检测串口）

### 阶段 2（PID 闭环）改动
- `PID/pid.h`：新增 `PID_State_t`、`CascadedState_t`、`AxisCascadedState_t` 结构体
- `PID/pid.c`：实现 `PID_Calculate()`（P+I+D + anti-windup + 微分低通滤波）、`PID_Cascaded()`（角度外环→角速度内环）、状态重置函数
- `Core/Inc/usart.h`：新增 `attitude_data_t` 结构体和 `volatile g_attitude` 全局声明
- `Core/Src/freertos.c`：
  - `PWM_MAX` 从 80 改为 999（ARR 满量程）
  - `posture_task`：AHRS 计算后写入 `g_attitude`（Euler 角 + gyro deg/s）
  - `maneuver_task`：从开环直通改为级联 PID 闭环（Roll/Pitch 角度+角速度双环，Yaw 单角速度环）
  - `pid_update_task`：取消注释，启动时创建
  - `MX_FREERTOS_Init()`：启动时调用 `SystemParams_Init()` + `PID_LoadFromParams()`
  - RC 超时检查改用 `HAL_GetTick()`（修复与 `osKernelGetTickCount()` 不同步 bug）
- `Core/Src/usart.c`：
  - 新增调试命令：`att`（姿态）、`pid`（参数）、`pidr`/`pida`（实时调参）、`pidrst`（重置状态）、`rc`（注入虚拟RC）、`motors`（读取PWM）
  - 修复：`rc`/`att`/`pid`/`motors` 命令不再激活 `g_motor_test_active`

### 修复的 bug
1. **时间源不一致**：`rc.tick` 用 `HAL_GetTick()` 赋值，超时检查用 `osKernelGetTickCount()`，两者有固定偏移导致无符号减法溢出，RC 超时永远触发，油门永远为 0。改为统一用 `HAL_GetTick()`
2. **电机测试模式误触发**：`parse_motor_cmd()` 对所有命令设置 `g_motor_test_active=1`，导致 `rc` 等调试命令触发后 maneuver_task 跳过 PID 输出
3. **PID 参数未加载**：`SystemParams_Init()` 和 `PID_LoadFromParams()` 未在启动时调用，PID 参数一直是默认值 {1,0,0}

### 当前 PID 参数（Flash 默认值）
| 轴 | 角度环 Kp | 角度环 Ki | 角度环 Kd | 角速度环 Kp | 角速度环 Ki | 角速度环 Kd |
|----|----------|----------|----------|------------|------------|------------|
| Roll/Pitch | 4.0 | 0 | 0 | 0.1 | 0 | 0.02 |
| Yaw | 3.0 | 0 | 0 | 0.08 | 0 | 0.015 |

### 验证结果
- PID 闭环 PWM 输出正确：T=30% 基线 ≈300，Roll/Yaw 差速方向验证通过
- USART1 调试命令全部可用：`att`、`pid`、`pidr`、`pida`、`pidrst`、`rc`、`motors`
- 电机测试命令（m1-m4, mix, stop）继续可用

## 2026-05-13 — 陀螺仪姿态解算调通

### 改动
- `ICM42688.c`：`ICM42688_init()` 末尾补上 accel/gyro resolution 赋值（±8G, ±2000dps），之前因校准函数被跳过而一直为 0
- `AHRS_Mahony.c/h`：新增 `mahony_set_sample_period(0.02f)` 修正 500Hz→50Hz 积分误差；pitch 取反匹配航空惯例
- `platformio.ini`：加 `-Wl,-u,_printf_float` 让 newlib-nano 支持 `%f`
- `freertos.c`：posture_task 加 ATT 串口输出；sensor_task 滤波 alpha 调高（gyro 0.8, accel 0.5, mag 0.3），注释掉干扰 printf
- `main.c`：启动时打印三个传感器 WHO_AM_I；加载磁力计校准参数（硬铁偏移 + 软铁矩阵）；Baro_Init；Kp_mag=2.0
- `LPS22HBTR.c/h`：同时接受 LPS22HH WHO_AM_I=0xB3
- `scripts/flash_monitor.sh`：一键编译+烧录+串口捕获
- `scripts/attitude_viewer.html`：Web Serial API 3D 姿态可视化页面

### 结果
- Roll ~0.8° ±0.03°, Pitch ~1.1° ±0.01°, Yaw 漂移 <0.02°/s
- 三轴方向验证通过：右抬=正Roll，抬头=正Pitch，水平旋转=Yaw连续变化
