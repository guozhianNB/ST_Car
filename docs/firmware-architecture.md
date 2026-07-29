# STM32 固件架构与继续开发指南

## 入口与调度

`Core/Src/main.c` 只做 HAL/外设初始化，然后调用 `App_Init()` 和持续调用 `App_Run()`。工程不使用 RTOS，也不在运行状态调用 `HAL_Delay()`。

主循环以 `HAL_GetTick()` 做无阻塞分频：

| 周期/事件 | 工作 |
|---|---|
| 5 ms | 三编码器快照、左右轮速 PI、SA100 水管角度 PID |
| 10 ms | 八路循迹采样、循迹 PD、弯道降速、状态机与安全检查 |
| 新 `status=1` 视觉帧 | 球速估计、球位置 PD，更新目标水管角度 |
| 100 ms | LPUART1 非阻塞遥测 |

所有长度为 mm，速度为 mm/s，时间为 ms，角度为度。物理方向只允许通过 `Core/Inc/app_config.h` 的 sign 参数校正。

## 模块责任

| 模块 | 责任 | 禁止事项 |
|---|---|---|
| `motor` | 三路 TB6612 方向、PWM、制动/滑行、STBY | 不读取编码器，不运行控制器 |
| `encoder` | 三个定时器启动、回绕差分、累计计数、轮速 mm/s | 不直接驱动电机 |
| `sa100` | PWM Input 快照、占空比角度、跨零展开、超时 | 不用 P60 编码器冒充水管角度 |
| `vision_uart` | UART4 环形 DMA、换行组帧、字段/范围检查、球速低通 | `status=2` 不得刷新测量时间或微分 |
| `line_sensor` | 8 路采样、加权误差、横线判断 | 不直接输出 PWM |
| `pid` | 限幅、条件积分抗饱和的通用 PID | 不包含硬件和全局模式逻辑 |
| `control_loops` | 两轮独立 PI、循迹 PD、球位外环、角度内环、方向限位 | 不决定比赛流程 |
| `safety_monitor` | 超时、角度/计数越界、P60 堵转、丢线 | 不隐式清除故障 |
| `app` | 按键、模式、A 点门控、阶段计时、故障降级 | 不实现 HAL 外设初始化 |
| `telemetry` | 通过 ST-Link VCP 输出单行快照 | 不在控制周期阻塞等待串口 |
| `bench_debug` | 可编译移除的平衡台架命令、P60 限时脉冲、运行时标定和扩展遥测 | 不控制底盘；未显式开启时不得使能电机 |

旧的 `pid_control.*`、`i2c_soft.*`、`oled.*` 和空的 `road_1.c` 没有加入 IAR 或 CMake 工程；它们只作为早期实验遗留，不应被新代码引用。MPU6050 已改为硬件 I2C3，但当前 H 题主控制不依赖 MPU/Yaw。

## 运行模式

待机时 PA4 模式键循环选择，PC13 板载键启动：

1. `APP_MODE_LINE_ONLY`：只循迹，一圈后确认 A 横线并制动停车。
2. `APP_MODE_STATIC_BALL`：静止状态在总计 5 s 内完成 `+50 -> -50 mm`，每个目标连续稳定 250 ms。
3. `APP_MODE_MOVING_CENTER_AB`：球目标 0，循迹并以里程超过 B；`1550 mm` 是待赛道实测参数。
4. `APP_MODE_MOVING_CENTER_LAP`：球目标 0，整圈通过 A 后继续 200 mm 再停车；计时在确认通过 A 时冻结。
5. `APP_MODE_MOVING_TARGET`：球目标由 `App_SetMovingTarget()` 设置，整圈逻辑同上。
6. `APP_MODE_ANGLE_TEST`：空管依次跟踪 `0,+1,-1,+2,-2,0°`。

启动前状态机会检查：循迹模式必须看到线；摆杆模式必须已经把 `APP_BEAM_RANGE_VERIFIED` 和 `APP_SA100_CALIBRATION_VERIFIED` 置为 1、收到新鲜 SA100 且水管在水平 ±0.5°；滚球模式还必须有新鲜的真实视觉测量，并且球距该项目初始位置不超过 15 mm（静态/中心模式为 0，任意位置模式为预设目标）。检查不通过时不会拉高 TB6612 STBY。

A 点门控同时要求“已经离开起始横线”和“平均里程至少 300 mm”，再次检测到至少 6 路黑线并保持 30 ms 才确认一圈。上述阈值都在 `app_config.h`。

## 故障行为

任何故障先立即关闭底盘。只有 SA100 和 P60 工作区仍可信时，摆杆才允许在最多 1 s 内回到 0°；SA100 超时、角度越界、编码器越界或堵转时立即关闭摆杆 STBY。故障原因锁存在 `AppStatus.fault`，再次按启动键只回到待机，不会自动重启任务。

当前保护包括：

- SA100 100 ms 超时及 ±3° 软件限位；
- P60 相对中点 `[-260,+260]` 计数方向限位；
- P60 大 PWM、低计数变化持续 300 ms 的堵转判断；
- 真实视觉测量 200 ms 超时；
- 循迹丢线 500 ms；
- 静态滚球总过程 5 s 超时。

这些数值是安全的初始占位，不是已经在实物上验证的最终参数。

## 参数与首次调试

所有可调量集中在 `Core/Inc/app_config.h`。必须按以下顺序实测，否则不要进行组合闭环：

1. 三个 `MOTOR_*_SIGN`、三个 `ENCODER_*_SIGN`；
2. 最终 TI12 模式下的三个 counts/rev；
3. 两轮直径、轮距和三电机最低有效 PWM；
4. P60 安全中点、两侧计数边界和堵转阈值；
5. SA100 周期范围、厂家占空比公式、水平原始角和方向；
6. 循迹模块黑线极性、左右顺序、权重和横线阈值；
7. 左右轮 PI，再调循迹 PD；
8. 空管角度环，再调静态球位置环；
9. 最后低速组合，之后才考虑底盘加速度前馈。

初始 PID 只为框架提供有限输出，不能直接视为可比赛参数。第一次上电必须架空轮子、拆开或松开连杆，并保持两路 STBY 物理断开。

## CubeMX 与工程文件

`ST_Car.ioc` 已记录完整引脚、DMA、IRQ、定时器和 I2C/UART 配置；`EWARM/ST_Car.ewp` 已纳入当前模块与 I2C HAL 驱动。手工 HAL 初始化与 IOC 保持一致。

重新生成代码前：

1. 先提交或备份当前工作区；
2. 在 CubeMX 检查所有资源仍与 `docs/stm32-pinout.md` 一致；
3. 生成后逐项审查 `main.c`、`gpio.c`、`tim.c`、`usart.c`、`i2c.c`、MSP 和 IRQ diff；
4. 不允许让 CubeMX 恢复成旧的 TIM4 双 PWM、删除 UART4 DMA，或把电机 STBY 默认拉高；
5. 重新做 `-Wall -Wextra -Werror` 编译和完整链接验证。

仓库保留 IAR 工程，同时只提交不含本机绝对路径的顶层 `CMakeLists.txt`、`cmake/*.cmake` 和 GCC 链接脚本。`CMakePresets.json`、`CMakeUserPresets.json`、`.vscode/`、`.settings/`、`.clangd` 与 `.mxproject` 均为本机配置，由各开发者在本地维护且不得提交；尤其不得把工具链安装目录、调试器路径或串口号写入受 Git 跟踪的文件。IAR 与 CMake 两套正式构建必须保持相同的活跃源码清单；旧实验模块不得加入任一正式构建。当前本机 VS Code Debug 预设输出为 `build/debug/ST_Car.elf`，仅构建不会构成硬件验证。

## 独立平衡台架调试

`APP_ENABLE_BENCH_DEBUG=1` 时，LPUART1 RX 接受台架命令。只有明确执行 `bench on` 且正常应用仍在待机时才进入台架状态；此后正常比赛状态机暂停、底盘强制关闭、PC13 作为急停。P60 开环只允许受 `APP_BENCH_OPEN_LOOP_PWM_LIMIT` 和 `APP_BENCH_PULSE_MAX_MS` 限制的脉冲；`APP_BEAM_RANGE_VERIFIED=0` 或 SA100 标定未确认时拒绝角度/滚球闭环，闭环运行时继续执行传感器超时、角度、编码器、堵转和视觉保护。

调试完成后将 `APP_ENABLE_BENCH_DEBUG` 改成 `0`；模块编译为惰性空桩，不再启动 LPUART1 RX，也不能使能电机。完整命令和阶段门禁见 `docs/balance-bench-debug.md`。
