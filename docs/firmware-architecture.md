# 固件架构与安全状态机

最后更新：2026-08-01。正式滚球执行器已由 P60 直流减速电机迁移为 MS42CG + 普菲德 TB6600 光耦型 STEP/DIR/ENA 驱动器。

## 1. 控制边界

NUCLEO-G491RE 是唯一实时控制器。MaixCAM2 只输出球位置；比赛图传与此控制链独立。

正式滚球采用视觉单环 PID：

```text
MaixCAM2 真测量位置/球速
        -> 视觉 PID（输出有符号 command steps/s）
        -> 速率/加速度/换向限制
        -> TIM15 STEP + DIR + EN
        -> MS42CG 驱动器与机构
```

不存在 SA100 角度内环、步进位置环、虚拟中点或软件行程边界。TIM4 步进编码器只用于堵转检测和调试，不参与球位 PID、启动门禁或正常控制反馈。`status=2` 是视觉保持值，不是测量，不得刷新位置滤波、球速或 PID 积分。

左右轮仍各自具有独立轮速 PI；循迹 PD 只产生左右目标速度，不能直接输出左右 PWM 差值。

## 2. 调度

| 周期 | 工作 |
|---|---|
| UART DMA/IDLE 事件 | 只完成缓冲切换、帧提取和时间戳 |
| 5 ms | 编码器快照、左右轮 PI、视觉 PID 输出复用、步进速率斜坡 |
| 10 ms | 循迹采样/外环、应用状态机、安全监控 |
| 100 ms | 非阻塞遥测与 OLED 刷新 |

TIM15 以 1 MHz 计数基准输出 STEP。驱动层按命令步频动态设置 ARR/CCR；停止时先将比较值清零并停通道。换向时先停 STEP、更新 DIR、等待 `APP_STEPPER_DIRECTION_SETUP_MS`，随后才恢复脉冲。TIM15 不使用中断。

## 3. 模块职责

| 模块 | 职责 | 禁止事项 |
|---|---|---|
| `motor` | 仅管理左右轮 TB6612、TIM1_CH1/2、PB0 STBY | 不管理步进电机 |
| `stepper` | 管理 TIM15 STEP、PC4 DIR、PC5 EN、停脉冲与方向建立时间 | 不读取视觉/编码器，不实现控制器 |
| `encoder` | 三个 AB 计数器、回绕、累计计数；左右轮换算 mm/s | 不驱动输出；执行器计数不换算球位 |
| `vision_uart` | 循环 DMA、协议解析、真测量时间戳与球速 | `status=2` 不作为新测量 |
| `line_sensor` | 生产循迹掩码和归一化误差 | 不写 PWM |
| `control_loops` | 左右轮 PI、循迹外环、视觉球位 PID、步进速率限制 | 不建立步进位置/角度反馈环 |
| `safety_monitor` | 执行器堵转、丢线与故障锁存 | 不自动清故障 |
| `app` | 比赛模式、启动检查、完成/故障流程 | 不直接写 GPIO/PWM |
| `bench_debug` | 限时步频测试、视觉闭环、遥测与急停 | 台架激活时不允许底盘运行 |

旧 `sa100.*` 文件仅作为未纳入正式构建的历史文件保留；CMake 和 IAR 工程都不得重新加入它。旧 `pid_control.*`、`i2c_soft.*`、`oled.*`、`road_1.c` 同样不属于正式控制链。

## 4. 输出安全语义

- 复位、待机、故障：PC5 EN 低、TIM15 STEP 停止；PB0 低；PA10/PB1 固定低。
- 活跃平衡且 PID 输出为零：STEP 停止，PC5 可保持高，使步进驱动器提供静态保持转矩。
- 视觉测量超过 `APP_VISION_CONTROL_HOLD_MS=150 ms`：立即停 STEP，不等待加速度斜坡；清除积分时间基准。
- 关闭滚球环：立即停 STEP；关闭执行器：同时将 EN 拉低。
- 急停/故障不绕过堵转、超时或启动检查；恢复必须经待机重新启动。

实物采用 3.3 V 共阳：ENA+ 接 3V3、PC5 接 ENA-。TB6600 的 ENA 是脱机输入，PC5 低使 ENA 光耦有效并脱机，PC5 高释放脱机并允许驱动。因此固件仍以“PC5 高=逻辑使能”表示。首次通电仍必须在电机主电源断开的情况下测量这一行为；若实物与通用说明不一致，不得接机构测试。

`VERIFIED（用户台架实测，2026-08-01）`：当前 3.3 V 共阳光耦输入工作可靠；正命令使电机顺时针旋转并抬高水管。TIM4 在该方向的原始单圈变化为 `-4096 count`，编码器层集中使用 `APP_ENCODER_ACTUATOR_SIGN=-1`，因此调试遥测以正命令方向为正计数。该反相只影响编码器调试/堵转数据，不改变 STEP/DIR 方向，也不把编码器接入 PID。

## 5. 视觉 PID

只在新的 `status=1` 测量到来时计算：

```text
error = target_position - measured_position
inside = Kp*error - Kd*measured_speed - Ka*measured_acceleration + Ki*integral
requested_step_rate = control_sign * clamp(inside, rate_limit)
```

单位：位置 mm，速度 mm/s，加速度 mm/s²，PID 输出 command steps/s。积分只随真测量时间推进并条件抗饱和；保持区内清零积分并停 STEP。驱动器已在2026-08-01切换并确认到16细分、3200 pulses/rev。正式要求3的实物验证参数为 `Kp=1.2`、`Ki=2.0`、`Kd=2.0`、`Ka=0`、`rate_limit=400 steps/s`。

视觉无效/陈旧、急停或故障直接停脉冲。端点以 `100 steps/s` 有界释放；普通回中若球在积分压坡后同时满足真实位移至少2 mm和速度至少10 mm/s，则暂时进入快速卸坡捕获，退出条件为速度回落至8 mm/s。该捕获模式在要求3序列中强制关闭，序列始终服从 `400 steps/s` 上限。

## 6. 堵转检测

当实际 STEP 频率绝对值至少为 `32 steps/s`，且 TIM4 执行器编码器在连续 5 ms 快照中没有计数变化达 `750 ms`，锁存 `FAULT_ACTUATOR_STALL` 并关闭 EN。该阈值是未经实物验证的初始值；在正常运动/人为堵转数据采齐前，不得把它描述成可靠保护。

关闭此保护前必须先证明编码器随电机运动可靠变化；编码器不兼容、未供电或方向未知时只允许做静态波形与受监控的短时低速测试，不能进入滚球闭环。

## 7. 应用状态机

状态：`STANDBY`、`LEVELING`、`RUNNING`、`FINISHED`、`FAULT`。

模式：单独循迹、静态 `0 -> +50 -> -50 mm`、AB 中心保持、整圈中心保持、移动任意目标、保持当前视觉位置。旧 SA100 角度测试模式已移除。

启动前：需要循迹的模式必须检测到线；需要滚球的模式必须有年龄不超过 `APP_VISION_COAST_MS` 的真实 `status=1` 球位，并满足对应起点容差。执行器编码器不会作为中点/位置门禁。通过检查后才拉高执行器 EN 或底盘 STBY。

静态要求 3 在首次真测量进入 `+50 ±10 mm` 后切换到 `-50 mm`；最终位置需在位置/速度阈值内按真实视觉帧累计稳定 250 ms。完成后冻结时间，但继续视觉闭环保持和安全监控。

## 8. 面板与台架

- PA4/PA15 短按：待机/调平态执行相反方向的 `200 steps/s、60 ms` 受限试动；这是机构人工调平，不修改传感器零点。
- PA4 长按：启动静态要求 3。
- PA15 长按：捕获当前真实球位并启动保持。
- PC13：比赛状态下启动/停止；台架激活时为急停。

台架命令：`bench on|off`、`run <signed_steps_s> <ms>`、`zero`、`diag reset`、`ball <mm>|sequence`、`gain ball ...`、`gain accel ...`、`limit rate <steps_s>`、`status`、`stream on [ms]|off`、`stop`、`clear`。

`zero` 只清执行器编码器的调试累计值，不定义物理中点，也不影响控制。`run` 同时受100 steps/s、500 ms和最多10个命令脉冲三重限制，并继续执行堵转保护；因此危险的 `run 200 500` 会被拒绝。

## 9. CubeMX 再生成检查

再生成前保存工作树，生成后必须确认：

1. TIM1 只有 CH1/CH2 两路 20 kHz PWM，PA10 仍为低电平 GPIO；
2. TIM15_CH1 在 PB14 为 PWM 输出，预分频 169、无 TIM15 IRQ；
3. PC4/PC5 是低启动的 DIR/EN，PB1 保持低；
4. TIM4 仍为 PB6/PB7 TI12 编码器；
5. UART4 RX 循环 DMA/IDLE 未被删除；
6. SWD PA13/PA14 与 ST-Link VCP PA2/PA3 未被占用；
7. `main.c` 业务逻辑没有被堆回中断或初始化函数。

## 10. 验证状态

- `VERIFIED（静态/编译，2026-08-01）`：GNU Arm `-Wall -Wextra -Werror` 编译链接成功；CMake/IAR 均含 `stepper.c`；IOC 与手写初始化的引脚/定时器一致。
- `VERIFIED（用户台架实测，2026-08-01）`：3.3 V 共阳功能兼容、机械正方向、编码器单圈计数及方向。
- `UNVERIFIED`：光耦电流具体数值、电机相电流/电流拨码、编码器电平幅度/输出级、堵转阈值、步频/加速度与所有新 PID 参数。
- `TODO`：完成独立 STEP/DIR/EN 波形、单电机低速、编码器、空管、视觉固定点与静态滚球各阶段实测并保存数据。
