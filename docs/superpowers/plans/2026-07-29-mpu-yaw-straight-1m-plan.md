# MPU6050 X 轴 yaw 闭环直行 1 m 实现计划

> **面向 AI 代理的工作者：** 必须使用 `subagent-driven-development` 或 `executing-plans` 逐任务实现本计划。步骤使用复选框语法跟踪进度。

**目标：** 将 MPU6050 封装为 `Yaw_Init()` / `GetYaw()`，使用其 X 轴角度保持航向，同时以双编码器平均里程控制小车直行 100 cm 后停车。

**架构：** `yaw.c` 是主程序唯一使用 MPU 的入口；它初始化、校准并基于 `HAL_GetTick()` 积分 X 轴角速度。`main.c` 只读取 `GetYaw()`，将 yaw PID 与既有 `PID_Control_SetTarget()` 的编码器差速修正叠加，最后只调用 `Motor_SetSpeed()`。`mpu.c` 和 `i2c_soft.c` 保持为协议和寄存器访问层。

**技术栈：** STM32G491 HAL、软件 I2C（PC8=SCL、PC9=SDA）、TIM2/TIM3 编码器、TIM4 PWM、IAR EWARM。

---

## 文件结构

- 创建 `Core/Inc/yaw.h`：对主程序公开 `Yaw_Init()` 与 `GetYaw()`。
- 创建 `Core/Src/yaw.c`：维护校准后的 X 轴 yaw、时间戳和有效状态。
- 修改 `Core/Inc/mpu.h`、`Core/Src/mpu.c`：使 MPU 初始化能报告通信状态，改为 PC8/PC9，移除 F1/I2C HAL 依赖。
- 修改 `Core/Src/main.c`：初始化 yaw，完成 1 m 编码器里程和 yaw 闭环控制。
- 修改 `EWARM/ST_Car.ewp`：将 motor、PID、软件 I2C、MPU 和 yaw 源文件纳入 IAR 构建。

### 任务 1：让 MPU 底层使用正确的引脚和平台头文件

**文件：**

- 修改：`Core/Inc/mpu.h`
- 修改：`Core/Src/mpu.c:1-55`

- [ ] **步骤 1：先写出可检查的接口约束**

在 `mpu.h` 中将初始化声明定义为：

```c
#include <stdbool.h>

bool MPU6050_Init(void);
```

`true` 只表示读到 `WHO_AM_I == 0x68` 且初始化寄存器写入均成功；否则返回 `false`。

- [ ] **步骤 2：实现最小通信状态检查**

在 `mpu.c` 中移除 `#include "i2c.h"` 和 `#include "stm32f1xx_hal.h"`，保留 `main.h`/`i2c_soft.h` 取得 G4 HAL 定义；将 I2C 初始化替换为：

```c
I2C_Soft_Init(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9);
```

对 WHO_AM_I 读操作和每个关键寄存器写操作检查返回值，任何失败立刻返回 `false`；全部成功后返回 `true`。

- [ ] **步骤 3：运行静态验证**

运行：

```powershell
rg -n "stm32f1xx_hal|GPIO_PIN_11|GPIO_PIN_12|I2C_Soft_Init" Core/Src/mpu.c
```

预期：不存在 F1 头文件和 PA11/PA12；只存在 `GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9` 的软件 I2C 初始化。

- [ ] **步骤 4：提交底层修正**

```powershell
git add Core/Inc/mpu.h Core/Src/mpu.c
git commit -m "fix: configure mpu software i2c pins"
```

### 任务 2：实现 yaw 模块公共接口

**文件：**

- 创建：`Core/Inc/yaw.h`
- 创建：`Core/Src/yaw.c`

- [ ] **步骤 1：定义接口**

创建 `yaw.h`：

```c
#ifndef __YAW_H__
#define __YAW_H__

#include <stdbool.h>

bool Yaw_Init(void);
float GetYaw(void);

#endif
```

- [ ] **步骤 2：实现状态与初始化**

`yaw.c` 定义 `static float s_yaw_deg`、`static uint32_t s_last_tick` 和 `static bool s_ready`。`Yaw_Init()` 先调用 `MPU6050_Init()`；失败时清零状态并返回 `false`。成功时调用 `MPU6050_CalibrateGyroBias(300U, 2U)`，清零 `s_yaw_deg`，保存 `HAL_GetTick()`，置 `s_ready = true` 并返回 `true`。

- [ ] **步骤 3：实现 GetYaw 的时间与轴映射**

`GetYaw()` 在 `s_ready == false` 时返回 `0.0f`。否则用 `HAL_GetTick()` 计算秒级 `dt`，将 `dt <= 0` 或 `dt > 0.2f` 回退为 `0.02f`，并执行：

```c
float gyro_dps[3];
MPU6050_GetGyroData(gyro_dps);
s_yaw_deg += gyro_dps[0] * dt_s;
return s_yaw_deg;
```

X 轴是用户定义的 yaw 轴；不要调用现有 `Only_for_Roll()`，避免其在线偏置重估改变已校准的航向基准。

- [ ] **步骤 4：运行接口静态检查**

运行：

```powershell
rg -n "bool Yaw_Init|float GetYaw|gyro_dps\[0\]|HAL_GetTick" Core/Inc/yaw.h Core/Src/yaw.c
```

预期：两个接口各声明和定义一次，积分源仅为 `gyro_dps[0]`。

- [ ] **步骤 5：提交 yaw 模块**

```powershell
git add Core/Inc/yaw.h Core/Src/yaw.c
git commit -m "feat: add x-axis yaw module"
```

### 任务 3：在主程序实现 1 m 直行闭环

**文件：**

- 修改：`Core/Src/main.c`

- [ ] **步骤 1：定义控制参数与小函数**

在 `main.c` 的 USER CODE 区定义以下参数：`CONTROL_INTERVAL_MS = 20U`、`ENCODER_COUNTS_PER_REV = 780.0f`、`WHEEL_CIRCUMFERENCE_CM = 20.73f`、`STRAIGHT_SPEED = 500`、`TARGET_DISTANCE_CM = 100.0f`、`DISTANCE_TOLERANCE_CM = 1.0f`、`YAW_KP = 18.0f`、`YAW_OUTPUT_LIMIT = 200`。实现饱和函数，限制航向修正和最终左右输出至 `[-1000, 1000]`。

- [ ] **步骤 2：实现单次直行函数**

实现 `static void Car_DriveStraightCm(float target_cm)`：保存上次左右编码器计数；每 20 ms 取有符号的 `int16_t` 增量，按以下公式累加平均距离：

```c
distance_cm += ((float)(delta_left + delta_right) * 0.5f)
             * WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REV;
```

每个控制周期调用 `GetYaw()`，用 `yaw_correction = clamp(YAW_KP * yaw_deg, -YAW_OUTPUT_LIMIT, YAW_OUTPUT_LIMIT)` 计算航向差速；调用 `PID_Control_SetTarget(STRAIGHT_SPEED, 0.0f, &base_left, &base_right)`，之后仅通过：

```c
Motor_SetSpeed(base_left - yaw_correction,
               base_right + yaw_correction);
```

驱动方向若实车发现反向，保持函数结构不变，仅反转 `yaw_correction` 的符号。距离进入容差后连续 5 次确认，退出时调用 `Motor_SetSpeed(0, 0)` 和 `PID_Control_Reset()`。

- [ ] **步骤 3：连接启动流程与主循环**

在 `Motor_Init()`、`PID_Control_Init()` 后调用 `Yaw_Init()`；若失败，调用 `Motor_SetSpeed(0, 0)` 并进入 `Error_Handler()`。删除当前无限前进的目标速度逻辑，改为在完成初始化后调用一次 `Car_DriveStraightCm(TARGET_DISTANCE_CM)`，随后空转主循环。

- [ ] **步骤 4：验证电机接口边界**

运行：

```powershell
rg -n "HAL_TIM|HAL_GPIO_WritePin|Motor_SetSpeed|GetYaw|Car_DriveStraightCm" Core/Src/main.c
```

预期：`main.c` 中电机执行命令仅为 `Motor_SetSpeed(...)`；控制循环调用 `GetYaw()` 并调用一次 `Car_DriveStraightCm(100.0f)`。

- [ ] **步骤 5：提交直行闭环**

```powershell
git add Core/Src/main.c
git commit -m "feat: drive straight one meter with yaw control"
```

### 任务 4：把全部用户源文件纳入 IAR 并构建验证

**文件：**

- 修改：`EWARM/ST_Car.ewp:1082-1100`

- [ ] **步骤 1：补全 IAR 的 Core 源文件列表**

在 `User/Core` 组中追加：

```xml
<file><name>$PROJ_DIR$/../Core/Src/motor.c</name></file>
<file><name>$PROJ_DIR$/../Core/Src/pid_control.c</name></file>
<file><name>$PROJ_DIR$/../Core/Src/i2c_soft.c</name></file>
<file><name>$PROJ_DIR$/../Core/Src/mpu.c</name></file>
<file><name>$PROJ_DIR$/../Core/Src/yaw.c</name></file>
```

- [ ] **步骤 2：检查项目引用完整性**

运行：

```powershell
rg -n "Core/Src/(motor|pid_control|i2c_soft|mpu|yaw)\.c" EWARM/ST_Car.ewp
```

预期：五个文件均在工程中恰好出现一次。

- [ ] **步骤 3：使用 IAR 构建**

若 `IarBuild.exe` 已安装，运行：

```powershell
IarBuild.exe EWARM/ST_Car.ewp -build Debug
```

预期：返回码 0，且没有未定义的 `Yaw_Init`、`GetYaw`、`Motor_SetSpeed`、`MPU6050_Init` 符号。若环境未安装 IAR，记录该限制并执行步骤 2 的工程引用检查。

- [ ] **步骤 4：实车验收清单**

1. 架空轮胎后上电：`Yaw_Init()` 成功，不触发错误处理。
2. 静止 10 秒：`GetYaw()` 变化应很小；转动车身，角度应随 X 轴方向变化。
3. 低速跑 1 m：车偏离时两个轮子的 PWM 有反向差速修正。
4. 调整 `WHEEL_CIRCUMFERENCE_CM` 至实际累计里程为 100 cm；随后才调 `YAW_KP`。

- [ ] **步骤 5：提交工程集成**

```powershell
git add EWARM/ST_Car.ewp
git commit -m "build: include yaw control sources"
```

## 最终验证

- [ ] 运行 `git diff --check`，预期无输出。
- [ ] 运行 `git status --short`，确认只保留用户已有的 `README.md`、`road_1.c` 和与本功能无关的未跟踪文件；所有本功能改动均已明确纳入提交。
- [ ] 在交付说明中明确：未经实车标定，轮周长、左右编码器符号和 yaw 修正符号可能仍需微调。
