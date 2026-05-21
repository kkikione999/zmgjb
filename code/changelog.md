# Change Log

## 2026-05-14 — PID 代码审查 + 200Hz 内环升级

### 代码审查发现并修复的问题

#### 问题 2: `prev_error` 命名误导（pid.h / pid.c）
- `PID_State_t.prev_error` 实际存储的是上一次 measurement，不是 error
- 改名: `prev_error` → `prev_measurement`

#### 问题 3: pid_update_task 延迟过大（freertos.c）
- `pid_update_task` 循环底部 `osDelay(1000)` 导致每秒只能处理 1 条参数更新
- 改为 `osDelay(10)`，快速调参时响应从 3s+ 降至 0.47s

#### 问题 5: g_runtime_pid 无互斥保护（freertos.c）
- `pid_update_task` 写、`maneuver_task` 读 `g_runtime_pid`（72 字节），无原子保护
- 在 `maneuver_task` 中添加 `__disable_irq()` 原子拷贝到局部变量

#### 问题 6: 首次运行微分尖峰（pid.h / pid.c）
- PID 首次调用时 `prev_measurement=0`，如果初始测量值不为 0 会产生微分尖峰
- `PID_State_t` 新增 `uint8_t initialized` 标志
- `PID_Calculate()` 首次运行时初始化 measurement 并跳过 D 项
- `PID_State_Reset()` 重置 `initialized=0`

#### 问题 1（误报）: pitch 混控符号
- 审查时怀疑 pitch 混控符号反了，但通过电机逐一测试确认 Mahony pitch 正方向（抬头为正）与混控符号一致
- **无需修改**

### 200Hz 内环升级（方案 A：整体提速）

#### 改动文件
- `Core/Src/freertos.c`

#### sensor_task 改动
- 平均采样 3→1（`ICM_GET_Average_Raw_data` 参数从 3 改为 1）
- 定时: `osDelay(18)` → `osDelayUntil` 5ms 周期 (200Hz)

#### posture_task 改动
- Mahony 采样周期: `mahony_set_sample_period(0.02f)` → `0.005f`
- 定时: `osDelay(20)` → `osDelayUntil` 5ms 周期 (200Hz)

#### maneuver_task 重写（核心改动）
- 内环角速度 PID: 50Hz → **200Hz** (`RATE_DT=0.005f`)
- 外环角度 PID: 保持 **50Hz** (`ANGLE_DT=0.02f`，每 4 个内环周期运行 1 次)
- 不再使用 `PID_Cascaded()`，改为分别调用 `PID_Calculate()` 传入不同 dt
- 外环输出的 rate setpoint 用 static 变量在调用间保持（零阶保持）
- 姿态熔断计数: `FUSE_COUNT_LIMIT` 50→200（适配 200Hz，仍为 1s）
- 电机测试模式定时: `osDelay(20)` → `osDelayUntil` 5ms
- 栈空间: 512×4 → 640×4（新增局部变量）

#### CPU 负载估算
| 任务 | 频率 | 每次耗时 | CPU 占用 |
|------|------|----------|----------|
| sensor_task | 200Hz | ~2ms | 40% |
| posture_task | 200Hz | ~0.3ms | 6% |
| maneuver_task | 200Hz | ~0.1ms | 2% |
| 其他 | — | — | ~5% |
| 合计 | | | **~53%** |

### 验证结果
- 编译通过: RAM 41.0%, Flash 12.9%
- 串口命令正常: `att`、`pid`、`pidr`、`pidrst`、`m1-m4`、`stop`
- PID 参数在线更新正常: 3 条命令 0.47s 完成
- 姿态数据正常: Euler 角和陀螺仪数据合理

## 2026-05-14 — 阶段 3：Roll 单轴 PID 调参

### 前期飞行测试结论
- 多次三轴同时调参失败（Roll/Pitch/Yaw 互相干扰，无法定位各轴问题）
- 悬停油门约 42-43%（PWM ~420/999）
- CG 偏右后，导致飞行时 Pitch 持续偏正（+13-15°），Roll 偏右（+5.8°）
- 50Hz 控制率是根本瓶颈，无法抑制 >12.5Hz 振荡
- **结论**: 改为单轴逐个调试方法

### 代码改动（飞行调试期间）
- `freertos.c`:
  - `PID_RATE_OUT_MAX` 从 150 降至 80（防止单电机修正溢出）
  - RC 超时从 200ms 改为 5000ms
  - 新增熔断保护：姿态角 >45° 持续 1s 自动归零油门
  - 新增 `#include <math.h>` 用于 fabsf
- `usart.c`: att 输出格式改为 `Euler: Roll, Pitch, Yaw  Gyro: gx, gy, gz`

### Roll 单轴调参（无人机固定在 Roll 轴方向）
#### Step 1: 方向验证 — PASS
- 方法：只开 Roll rate Kp=0.30，给 50% 油门，手动倾斜
- 结果：**方向正确**
  - 向右倾斜（右边低）→ 右侧电机(M2+M3)加速 → 推右侧上去 → 修正倾斜 ✓
  - 向左倾斜（左边低）→ 左侧电机(M1+M4)加速 → 推左侧上去 → 修正倾斜 ✓
- **不需要修改混控公式**

#### Step 2: Rate Kp 扫描（初步，角度环 Kp=2.0, RC: 50%油门 10%Roll）
| Kp_rate | avg_diff | max|diff| | 评级 |
|---------|----------|-----------|------|
| 0.10    | +1.6     | 16        | 弱   |
| 0.15    | +6.0     | 16        | 弱   |
| 0.20    | +9.6     | 20        | 弱   |
| 0.25    | +14.8    | 32        | 中   |
| 0.30    | +25.2    | 52        | 中   |
| 0.40    | +32.0    | 56        | 中   |
| 0.50    | +42.4    | 68        | 中   |
| 0.60    | +35.2    | 76        | 中   |
| 0.80    | -25.6    | 228       | 强(振荡) |

- Kp=0.80 出现大幅振荡（max|diff|=228，方向反转），说明振荡点在 0.60-0.80 之间
- **待继续**: 需要更精细扫描 0.60-0.80 区间，以及验证这些值在实际飞行中的表现

### 新增脚本
- `scripts/roll_tune.py` — Roll 单轴交互式调参工具
- `scripts/roll_dir_test.py` — Roll 方向自动验证脚本
- `scripts/roll_kp_scan.py` — Rate Kp 自动扫描脚本

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
