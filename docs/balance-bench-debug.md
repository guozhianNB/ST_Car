# 独立平衡滚球台架调试指南

本文只调试 P60、TB6612、P60 编码器、曲柄连杆、水管、SA100、MaixCAM2 和钢球。左右轮与循迹保持断开或禁用，直到静态滚球完整通过。

## 1. 调试功能的启用与移除

`Core/Inc/app_config.h` 中：

```c
#define APP_ENABLE_BENCH_DEBUG 1
```

为 `1` 时启用 ST-Link VCP 台架控制台。调试完成、把实测参数写回 `app_config.h` 后改为 `0` 并重新编译；`bench_debug.c` 会编译成不能使能电机的空桩。调试代码集中在 `Core/Inc/bench_debug.h`、`Core/Src/bench_debug.c` 和 `tools/bench_console.py`，不需要到处删除临时代码。

只有收到 `bench on` 后才进入台架状态。激活期间正常比赛状态机暂停、底盘强制关闭、PC13 板载按钮为急停。开环命令只能是最长 500 ms、绝对值不超过 250/1000 的限时脉冲；单次角度或滚球闭环最长运行 30 s 后自动停止，并继续检查 SA100 新鲜度、角度限位、P60 编码器限位、堵转和视觉超时。

## 2. 每次让电机动作前的安全条件

1. 左右轮悬空或电机断开，钢球移除；
2. 初次 P60 测试时连杆脱开；连接机构后确认曲柄远离死点；
3. 第一次只测 PA10 时，P60 STBY 在驱动侧可靠接 GND；
4. NUCLEO、TB6612、编码器、SA100、MaixCAM2 和电源可靠共地；
5. 用万用表确认 VM/VCC 极性与电压，用示波器确认未知传感器输出电平；
6. 操作者能立即断开 VM；PC13 软件急停不能替代物理断电；
7. 优先使用带限流实验电源；未取得 MG513P60 电流规格并实测前，不认为 TB6612 电流能力已经满足。

## 3. VS Code 构建、烧录和控制台

1. 用 ST-Link USB 连接 NUCLEO，打开仓库根目录。
2. `Ctrl+Shift+B` 运行 `STM32: Build Debug`。
3. `Ctrl+Shift+D`，选择 `STM32 Launch STLink GDB Server`，按 `F5` 烧录；程序停在 `main` 时再按一次 `F5` 运行。
4. 用设备管理器查 ST-Link VCP 的 COM 号，或在 PowerShell 执行：

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Name
```

5. 第一次使用控制台安装依赖：

```powershell
python -m pip install pyserial
```

6. 只打开一个串口程序：在 VS Code 的集成终端直接运行下面的命令。不要再同时打开串口助手、Arduino 串口监视器或另一个 Python 控制台；同一个 COM 口通常只能被一个程序占用。

```powershell
python tools/bench_console.py --port COM5
```

串口为 115200 8N1。程序持续运行、等待命令是正常现象，并不是卡死；退出使用 `Ctrl+C`。连接后应看到 `bench> ` 提示符，在这个提示符后直接输入：

```text
help
bench on
status
```

正常返回 `mode=idle fault=none`。退出前执行：

```text
stop
bench off
```

控制台退出时也会尝试发送这两条命令，但改线前仍须物理断开 VM。

台架模式默认不连续输出数据，`status` 只返回一帧。需要观察动态过程时输入 `stream on 200`，表示每 200 ms 输出一帧；结束观察输入 `stream off`。如果出现“无法打开 COM 口/Access denied”，说明另一个串口程序仍占用了同一个端口，关闭它后重试。

## 4. 命令表

| 命令 | 作用 | 前置条件 |
|---|---|---|
| `bench on` | 进入台架模式并关闭全部输出 | 正常应用为 standby |
| `bench off` | 关闭输出并退出台架模式 | 任意台架状态 |
| `status` | 立即输出完整快照 | 台架已开启 |
| `stream on [ms]` | 连续输出快照，默认 100 ms，可设 50～2000 ms | 台架已开启 |
| `stream off` | 停止连续输出，恢复安静命令行 | 台架已开启 |
| `stop` | 关闭两块 STBY，故障后回到 idle | 台架已开启 |
| `zero` | 人工确认安全机械中点后将 P60 编码器清零 | 输出已停止 |
| `pulse <pwm> <ms>` | P60 原始开环限时脉冲，如 `pulse 100 200`；不应用正式控制的最小 PWM 补偿 | 当前计数未触边界 |
| `angle <deg>` | 保持水管目标角 | SA100 新鲜且已标定 |
| `ball <mm>` | 保持球目标位置 | SA100 与真实视觉测量新鲜 |
| `gain angle <kp> <ki> <kd>` | 修改台架角度 PID | 建议先 stop |
| `gain ball <kp> <kd> <sign>` | 修改球位置 PD 与方向 | sign 只能为 ±1 |
| `limit pwm <value>` | 修改闭环 PWM 上限 | 不超过固件 P60 上限 |
| `limit angle <deg>` | 修改台架目标角限幅 | 不超过软件安全角 |
| `sa cal <scale> <zero> <sign>` | 设置 `raw=duty*scale`、`angle=(raw-zero)*sign` | 输出已停止 |
| `config` | 输出当前参数汇总 | 台架已开启 |

`status` 或已开启的连续遥测以 `BENCH` 开头，主要字段为：

```text
mode/fault/stream/rate/pwm/count/delta
sa/fresh/sacal/rangeok/per/high/raw/ang/aref
vision/vfresh/ball/vel/bref
akp/aki/akd/bkp/bkd/bsign/plim/alim
scale/zero/sasign
```

`count` 是 P60 累计计数，`delta` 是最近 5 ms 计数变化；`per/high` 是 SA100 原始微秒量；`ball/vel` 单位为 mm 和 mm/s。运行时 `gain`、`limit`、`sa cal` 只改 RAM，复位会恢复 `app_config.h`。

其中 `rangeok=0` 或 `sacal=0` 时，固件会拒绝 `angle` 和 `ball` 命令：前者表示 P60 安全范围还没有写回并确认，后者表示本次上电尚未执行 `sa cal`，且固件参数也尚未标为已验证。这是防止占位参数意外闭环的强制门禁。

## 5. 必须取得并写回的参数表

不要直接复制理论值。按后续阶段实测后，把结果、日期、供电电压和机械版本一并记录，再修改对应宏。

| 参数 | 获取阶段与方法 | 写回位置 | 闭环前要求 |
|---|---|---|---|
| P60 电机正方向 | 阶段 B 比较正负脉冲的真实运动方向 | `APP_MOTOR_BEAM_SIGN` | 已确认 |
| P60 编码器正方向 | 阶段 B 比较运动方向与 `count` 增减 | `APP_ENCODER_BEAM_SIGN` | 已确认 |
| P60 输出轴 counts/rev | 脱连杆、输出轴准确转一圈，正反各三次 | `APP_ENCODER_BEAM_CPR` | 三次结果可重复，确认 TI12 已计入 |
| 最低有效/最大 PWM | 阶段 B 从低到高试验，记录启动、回转和温升 | `APP_MOTOR_BEAM_MIN_PWM/MAX_PWM` | 不过流、不异常升温 |
| 机械安全计数范围 | 阶段 C 从人工中点向两侧点动并留裕量 | `APP_BEAM_ENCODER_MIN/MAX_COUNT` | 写回后令 `APP_BEAM_RANGE_VERIFIED=1` |
| 堵转阈值 | 阶段 C 受控试验记录 PWM、5 ms 计数变化和持续时间 | `APP_BEAM_STALL_*` | 能识别异常且正常换向不误报 |
| SA100 换算、零点、方向 | 阶段 D 用可靠角度基准做多点记录/拟合 | `APP_SA100_DUTY_TO_DEG`、`APP_SA100_HORIZONTAL_RAW_DEG`、`APP_SA100_ANGLE_SIGN` | 写回后令 `APP_SA100_CALIBRATION_VERIFIED=1` |
| SA100 周期和超时 | 阶段 D 观察 `per` 的实测范围及拔线行为 | `APP_SA100_PERIOD_MIN/MAX_US`、`APP_SA100_TIMEOUT_MS` | 正常无误报，拔线可靠超时 |
| 角度 PID | 阶段 E 从小角度、小 PWM 单变量调节 | `APP_BEAM_ANGLE_KP/KI/KD` | `0,+1,-1,+2,-2,0°` 通过 |
| 球位置 PD 与方向 | 阶段 G 从中心、小角度开始 | `APP_BALL_KP/KD`、`APP_BALL_CONTROL_SIGN` | 静态三目标重复通过 |
| 闭环角度上限 | 阶段 E/G 逐步放宽后的最小充分值 | `APP_BALL_ANGLE_LIMIT_DEG` | 不超过实测机械安全范围 |
| 视觉坐标与超时 | 阶段 F 五点标定、遮挡和拔线试验 | `Vision/cv.py` 与 `APP_VISION_TIMEOUT_MS` | `status=1` 才刷新真实测量 |

以下阶段必须严格按顺序执行，不能跨过 `rangeok/sacal` 门禁。

## 6. 阶段 A：只验证 PA10 PWM

电机、连杆、SA100 和编码器均可不接；驱动侧 STBY 保持 GND。

```text
bench on
pulse 100 500
```

示波器地接 NUCLEO GND，探头接 PA10，应看到约 20 kHz、0~3.3 V、约 10% 占空比。500 ms 后自动回零。`pulse -100 500` 应有同样占空比；负号只改变 PC4/PC5 方向。

`pulse` 参数采用千分制，`1000=100%`。该命令为测量最低有效 PWM 而直接使用原始值：例如 `pulse 50 200` 应输出约 5%，不会被 `APP_MOTOR_BEAM_MIN_PWM=100` 提升到 10%。正式角度/滚球闭环仍使用配置的最小 PWM 补偿。

通过条件：频率约 20 kHz、脉宽与命令比例一致、复位和脉冲结束后 PA10/PB1 为低。

## 7. 阶段 B：P60 脱连杆开环与编码器

连接 P60、第二块 TB6612 和编码器，连杆保持脱开。逐级测试：

```text
pulse 80 200
pulse 100 200
pulse 120 200
```

只有上一级没有过流、异常噪声、快速温升或复位才提高 PWM。若 250 仍不能启动，先查 STBY、方向、电机端电压、机械卡滞和限流，不直接提高调试上限。

记录正/负命令方向、最低可靠启动 PWM、空载电流和温升。断开 VM 后执行 `zero`，手动让最终输出轴准确转一圈，记录 `count`，正反各重复三次；静止时 `delta` 不应持续跳动。由此确定：

- `APP_MOTOR_BEAM_SIGN`；
- `APP_ENCODER_BEAM_SIGN`；
- `APP_ENCODER_BEAM_CPR`；
- `APP_MOTOR_BEAM_MIN_PWM/MAX_PWM`。

## 8. 阶段 C：机构中点、边界与堵转

断 VM 后连接曲柄连杆，不放球。手动把曲柄放到远离死点、传动关系良好且水管水平的位置，安装/调整 PA15 中点开关并做物理标记，然后：

```text
stop
zero
```

使用最低有效 PWM 的 100~200 ms 脉冲向两侧逐步移动，每步保存 `count`、电流和机构位置。当前 `[-260,+260]` 是保守初值；只有人工确认更大范围仍远离死点时，才修改源码扩大并重新编译。

记录两侧机械危险边界，在其内留出可重复余量后填写 `APP_BEAM_ENCODER_MIN/MAX_COUNT`，并把 `APP_BEAM_RANGE_VERIFIED` 改为 `1` 后重新构建、烧录。通过受控测试确定 `APP_BEAM_STALL_PWM/DELTA_COUNT/TIMEOUT_MS`，不要为消除误报关闭保护。重新连接控制台后，`status` 必须显示 `rangeok=1`。

## 9. 阶段 D：SA100 原始量与标定

先不接 PB14，用示波器确认 SA100 高电平兼容该引脚。确认后连接，保持电机停止，用 `status` 观察 `sa/fresh/per/high/raw/ang`。

使用可靠角度基准记录水管 `-2,-1,0,+1,+2°` 时的 period/high/duty。采用厂家公式或多点拟合：

```text
raw = duty * scale
beam_angle = (raw - zero) * sign
```

运行时试验示例：

```text
stop
sa cal 360.0 183.4 -1
status
```

执行 `sa cal` 后，`status` 应显示 `sacal=1`，本次上电才允许角度闭环。验证后把 scale、zero、sign 写回 `APP_SA100_DUTY_TO_DEG`、`APP_SA100_HORIZONTAL_RAW_DEG`、`APP_SA100_ANGLE_SIGN`，把 `APP_SA100_CALIBRATION_VERIFIED` 改为 `1`，并按实测收紧周期范围；重新构建、烧录后 `sacal` 应从启动起就是 `1`。拔线后必须看到 `fresh=0`，闭环立即进入 `sa100_timeout`。

## 10. 阶段 E：空管角度闭环

不放球，从保守值开始：

```text
stop
limit pwm 150
limit angle 0.5
gain angle 30 0 0
angle 0
```

确认 0° 不失控，再测试 `angle 0.2`、`angle -0.2`。若误差扩大，立即按 PC13/断 VM并核对 sign。方向正确后一次只改一个量：先增加 Kp 获得响应，再增加 Kd 抑制过冲，只有长期静差时才加入很小 Ki。之后逐步扩展到 ±0.5、±1、±2°。

每组记录目标角、实际角、PWM、计数、最大过冲、稳定时间、电池电压和电流。用 `config` 保存最终运行时值并写回 `app_config.h`。

## 11. 阶段 F：视觉标定

摄像头安装到最终刚性支架，水管保持水平，人工把球放在 `-100,-50,0,+50,+100 mm`。MaixCAM2 应把球心投影到两标记点定义的水管轴向量后输出毫米坐标。

暂不运行球闭环，只观察 `ball/vel/vfresh`，验证方向、重复性、端到端延迟、遮挡和丢球。换光照、分辨率、相机位姿或焦距后重新标定。静态视觉误差应明显小于比赛 10 mm 总限值。

## 12. 阶段 G：静态滚球外环

角度环稳定、视觉标定完成后才放球：

```text
stop
limit pwm 150
limit angle 0.5
gain ball 0.005 0 1
ball 0
```

轻推钢球，观察水管是否送球回 0。若越来越远立即急停；确认其他方向正确后可用 `gain ball 0.005 0 -1` 反转球控制方向。方向正确后先逐步增加 ball Kp，再增加 Kd 抑制往返，不先加积分。逐步放宽角度到 ±1、必要时 ±2°，再测试：

```text
ball 0
ball 50
ball -50
```

最后退出台架，使用正常 `APP_MODE_STATIC_BALL` 验证 `O -> +50 -> -50 mm`、总时间 ≤5 s、误差绝对值 ≤10 mm、连续稳定 250 ms。

## 13. 调试收尾

1. 执行 `config`，保存参数与实验记录；
2. 把确认值写回 `Core/Inc/app_config.h`；
3. 用正常角度测试和静态滚球模式复验；
4. 将 `APP_ENABLE_BENCH_DEBUG` 改成 `0`；
5. 完整重新编译，确认 LPUART1 不再接收命令、复位后两路 STBY 为低；
6. 保存固件提交号、硬件版本、电源、日志和参数；
7. 静态滚球多次稳定通过后，才开始左右轮和循迹调试。
