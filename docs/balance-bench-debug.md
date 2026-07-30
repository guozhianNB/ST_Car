# 独立平衡滚球台架：测量、标定与调参操作手册

本文用于在不安装到底盘、不接左右轮、不接循迹模块的情况下，单独调通以下链路：

```text
MaixCAM2 测球位置
        ↓ UART4
NUCLEO-G491RE 球位置外环
        ↓ 目标水管角度
SA100 水管角度内环
        ↓
P60 + TB6612 + 曲柄连杆
        ↓
水管倾斜并控制钢球
```

本文所有步骤均与当前 `bench_debug.c`、`app_config.h`、`Vision/cv.py` 对齐。没有写入本文的传感器、开关或结构件都不是当前台架调试的前置条件。

## 1. 当前台架需要与不需要的硬件

### 1.1 需要的硬件

- NUCLEO-G491RE；
- 一块用于 P60 的 TB6612；
- MG513P60 及其 AB 相编码器；
- 曲柄、连杆和 25 cm 开槽水管；
- 安装在水管左侧转轴处、测量水管真实角度的 SA100；
- MaixCAM2 和固定刚性支架；
- 钢球；
- 电机电源、逻辑电源、万用表；
- 强烈建议准备示波器和带限流直流电源。

### 1.2 当前阶段明确不需要的硬件

- 左右两个 P30 车轮电机；
- 红外循迹模块；
- PA4 模式按键；
- OLED；
- MPU6050；
- PA15 微动开关或任何自动寻零开关。

PA15 当前在 IOC 和固件中未分配。P60 使用增量编码器，每次上电或复位后的唯一归零流程是：人工对齐安全中点刻线，确认水管接近水平，然后执行 `zero`。不要为了本手册增加微动开关、碰块或额外支架。

### 1.3 台架使用的实际引脚

| 功能 | NUCLEO 引脚 | 当前代码行为 |
|---|---|---|
| P60 PWM | PA10 / TIM1_CH3 | 约 20 kHz，命令采用 `0～1000` 千分制 |
| P60 IN1 / IN2 | PC4 / PC5 | TB6612 方向控制 |
| P60 STBY | PB1 | 低时禁用驱动，高时允许 P60 动作 |
| P60 编码器 A / B | PB6 / PB7 / TIM4 | TI12 编码器模式，最终 counts/rev 必须以实测为准 |
| SA100 PWM | PB14 / TIM15_CH1 | 1 us 输入捕获，CH1 测周期、CH2 测高电平 |
| MaixCAM2 到 STM32 | Maix A21 TX → PC11 RX | UART4，115200 8N1 |
| 调试控制台 | ST-Link VCP，PA2/PA3 | LPUART1，115200 8N1 |
| 软件急停 | NUCLEO 板载 PC13 蓝色键 | 台架激活后按下请求急停 |

所有电源和信号必须共地。PC13 软件急停不能替代电机 VM 的物理断电。

## 2. 当前源码中的真实状态

开始前打开 `Core/Inc/app_config.h` 核对。当前源码包含以下值：

```c
#define APP_ENABLE_BENCH_DEBUG              1
#define APP_ENCODER_BEAM_CPR             3200.0f
#define APP_MOTOR_BEAM_MIN_PWM              80
#define APP_BEAM_RANGE_VERIFIED              0
#define APP_SA100_CALIBRATION_VERIFIED        0
```

`APP_ENCODER_BEAM_CPR=3200` 和 `APP_MOTOR_BEAM_MIN_PWM=80` 来自此前“阶段 A/B”调试提交，但仓库中没有包含供电、电流、重复次数的完整原始记录。硬件和编码器版本没有变化时可以把它们作为当前工作值；进入闭环前仍应按本文快速复核并补充记录。

两个 `VERIFIED` 标志目前为 `0`，因此代码会拒绝 `angle`、`ball` 和 `ball sequence` 闭环。这是正常安全门禁，不是程序故障：

- 测完 P60 机械安全范围并写回后，将 `APP_BEAM_RANGE_VERIFIED` 改成 `1`；
- 标定 SA100 并写回后，将 `APP_SA100_CALIBRATION_VERIFIED` 改成 `1`；
- 每次改源码后必须重新编译和烧录；
- 每次 MCU 复位都会清空 P60 增量计数，所以连杆已连接时，复位前必须先把机构人工放回安全中点。

## 3. 坐标、单位和正方向必须先统一

整个固件使用：

- 长度：mm；
- 球速度：mm/s；
- 时间：ms；
- 水管角度：degree；
- PWM：`0～1000`，例如 `100` 表示约 10% 占空比。

建议本台架统一采用以下物理正方向，并在全部记录中保持一致：

- 水管中心为 `x=0 mm`；
- 从水管左支点看向右端，右侧为球位置正方向；
- 右端升高定义为水管正角度；
- 完成方向标定后，正 P60 命令应使右端升高；
- 完成编码器方向标定后，正 P60 命令应使 `count` 增大。

如果实际安装相反，只修改 `app_config.h` 中集中定义的 sign，不要在 PID、视觉或电机代码中临时添加负号。

## 4. 每次上电前的强制安全检查

每次可能让 P60 动作前逐项确认：

1. 左右轮电机和循迹模块断开或不安装；
2. 阶段 A/B 时曲柄连杆必须脱开；
3. 阶段 C 以后不放钢球，直到空管角度环验收完成；
4. 曲柄、支架和水管运动范围内没有手、线束或工具；
5. NUCLEO、TB6612、编码器、SA100、MaixCAM2 和电源已经共地；
6. 万用表确认 TB6612 VM/VCC 极性和电压正确；
7. 未确认 SA100 输出电平前，不把它接入 PB14；
8. 操作者可以立即断开电机 VM；
9. 任何重新接线、改变曲柄位置或连接/拆卸连杆都必须先断 VM；
10. 出现过流、复位、异常噪声、快速升温、机构接近死点或方向不确定时立即断 VM。

## 5. 编译、烧录、打开唯一的调试控制台

### 5.1 编译和烧录

当前本机 `.vscode` 配置属于忽略跟踪的本地配置。如果本机任务存在：

1. `Ctrl+Shift+B`，运行 `STM32: Build Debug`；
2. `Ctrl+Shift+D`；
3. 选择 `STM32 Launch STLink GDB Server`；
4. 按 `F5` 烧录；
5. 程序停在 `main()` 时，再按一次 `F5` 让程序继续运行。

若 VS Code 任务不可用，可以在已配置 STM32CubeCLT PATH 的终端执行：

```powershell
cmake -S . -B build/debug -G Ninja `
  -DCMAKE_BUILD_TYPE=Debug `
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON `
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake
cmake --build build/debug --target ST_Car
```

烧录后必须让程序离开 `main()` 断点，否则串口命令和控制循环都不会运行。

### 5.2 查找 COM 口

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Name
```

选择名称包含 ST-Link Virtual COM Port 的端口，例如 `COM5`。

### 5.3 只打开一个串口程序

不要同时打开其他串口助手。第一次使用安装依赖：

```powershell
python -m pip install pyserial
```

建议从第一次测量开始保存日志：

```powershell
python tools/bench_console.py --port COM5 `
  --log docs/measurements/balance-2026-07-30.txt
```

将 COM 号和日期替换成实际值。脚本会记录发送的命令和 STM32 回复。程序持续等待输入是正常现象；退出用 `Ctrl+C`。

连接后输入：

```text
bench on
status
```

正确现象：

- 回复 `OK bench active`；
- `mode=idle`；
- `fault=none`；
- P60 和底盘 STBY 在执行具体动作前保持关闭；
- 命令行默认不连续刷数据。

需要连续观察时：

```text
stream on 100
```

停止连续输出：

```text
stream off
```

退出或改线前：

```text
stop
bench off
```

然后物理断开电机 VM。

## 6. 命令与当前代码的准确含义

| 命令 | 实际代码行为 | 是否掉电保存 |
|---|---|---|
| `bench on` | 仅在正常应用为 standby 时进入台架，关闭底盘和全部电机输出 | 否 |
| `bench off` | 关闭全部输出并退出台架 | 否 |
| `status` | 输出一帧传感器、控制器和参数快照 | 不适用 |
| `stream on [ms]` | 每 50～2000 ms 连续输出快照，省略周期时为 100 ms | 否 |
| `stream off` | 关闭连续输出 | 否 |
| `stop` | 两路 STBY 拉低、PID 清零、故障回到 idle | 否 |
| `zero` | 在人工确认安全中点后，把 TIM4 和 P60 累计计数清零 | 否，复位后必须重新确认 |
| `pulse <pwm> <ms>` | 原始 P60 开环脉冲；绝对值最大 250，最长 500 ms，不应用最低 PWM 补偿 | 否 |
| `sa cal <scale> <zero> <sign>` | 修改 RAM 中 SA100 换算并使 `sacal=1` | 否 |
| `limit pwm <value>` | 修改台架角度闭环输出上限，最大不超过 `APP_MOTOR_BEAM_MAX_PWM` | 否 |
| `limit angle <deg>` | 修改台架球外环/角度命令限幅，不能超过软件软限位 | 否 |
| `gain angle <kp> <ki> <kd>` | 修改台架角度 PID | 否 |
| `angle <deg>` | 启动角度闭环，最长运行 30 s；要求 range 和 SA100 门禁通过 | 否 |
| `gain ball <kp> <kd> <sign>` | 修改台架球位置 PD 和总方向 | 否 |
| `ball <mm>` | 持续保持单个球目标，最长运行 30 s | 否 |
| `ball sequence` | 自动执行 `0 → +50 → -50 mm`，总超时 5000 ms | 否 |
| `config` | 输出当前 RAM 参数和关键编译参数，供人工写回 `app_config.h` | 不适用 |

重要区别：

- `pulse` 使用 `Motor_SetRaw()`，例如 `pulse 50 200` 就是约 5%，不会被最低有效 PWM 提升；
- `angle` 和 `ball` 使用正式 `Motor_Set()`，非零命令低于 `APP_MOTOR_BEAM_MIN_PWM` 时会补偿到该值；
- `gain`、`limit`、`sa cal` 都只改 RAM，复位、重烧或断电后恢复源码值；
- `APP_BEAM_RANGE_VERIFIED` 只能修改源码并重烧，不能用命令临时绕过。
- 进入任何 `fault` 后故障会锁存，`pulse`、`angle`、`ball` 都会被拒绝；先断 VM、检查原因，再执行 `stop` 清除台架故障。`zero` 和 `sa cal` 也要求先回到 `idle`。

## 7. `status` 每个字段怎样读

典型快照以 `BENCH` 开头。字段含义如下：

| 字段 | 单位/含义 | 用在哪一步 |
|---|---|---|
| `t` | STM32 启动后的 ms | 日志时间轴 |
| `mode` | `idle/pulse/angle/ball/ball_sequence/fault` | 判断当前状态 |
| `fault` | 故障原因 | 每次异常定位 |
| `stream/rate` | 是否连续输出及周期 ms | 日志设置 |
| `seq/seqms` | 五秒序列阶段和已经用时 | `ball sequence` |
| `pwm` | P60 实际有符号命令，已经应用电机 sign；正式闭环还包含最低 PWM 补偿 | 电机/角度调试 |
| `count` | P60 从最近一次 `zero` 开始的累计计数，已经应用编码器 sign | 中点和安全范围 |
| `delta` | 最近一次约 5 ms 更新的计数变化 | 运动/堵转趋势；电机停后通常回到 0 |
| `sa` | SA100 是否产生过格式有效的捕获 | SA100 接线 |
| `fresh` | SA100 最后一帧是否在 100 ms 内 | 角度闭环门禁 |
| `sage` | SA100 最后一帧年龄 ms；无有效帧为 -1 | 周期/超时验证 |
| `sacal` | 本次启动是否已确认 SA 标定 | 闭环门禁 |
| `rangeok` | 编译时 `APP_BEAM_RANGE_VERIFIED` | 闭环门禁 |
| `per/high` | SA100 PWM 周期和高电平时间，单位 us | 原始标定数据 |
| `duty` | `high/per`，范围应为 0～1 | SA100 拟合 |
| `raw` | `duty*scale` 并进行跨零展开后的原始角 | SA100 拟合 |
| `ang` | `(raw-zero)*sasign`，即当前水管角度 degree | 角度反馈 |
| `aref` | 当前目标水管角度 degree | 角度/球闭环 |
| `vision` | Maix 最近一帧状态：1 真测量、2 保持、0 无效 | 视觉丢球检查 |
| `vfresh` | 最后一个 `status=1` 是否在 200 ms 内 | 球闭环门禁 |
| `vage` | 最后一个真实测量的年龄 ms；没有真实帧为 -1 | 视觉延迟/超时 |
| `vframe` | STM32 收到并成功解析的视觉帧累计数 | 帧率/通信检查 |
| `ball` | 最后一个真实球位置 mm | 视觉和球反馈 |
| `vel` | STM32 根据真实视觉帧估算并低通后的球速 mm/s | 球位置 PD |
| `bref` | 球目标 mm | 球闭环 |
| `akp/aki/akd` | 当前台架角度 PID | 调参记录 |
| `bkp/bkd/bsign` | 当前台架球位置 PD 与方向 | 调参记录 |
| `plim/alim` | 当前台架闭环 PWM/角度限幅 | 调参记录 |
| `scale/zero/sasign` | 当前 RAM 中 SA100 标定参数 | 写回源码 |

`vision=2` 只是 Maix 保持上一位置，不是新测量。它不会刷新 `vage`、`vfresh` 或球速。遮住球后即使短暂出现 `vision=2`，超过视觉超时仍应进入故障。

## 8. 阶段 A：只测 PA10、方向 GPIO 和 STBY，不接电机

### 8.1 接线状态

- 不接 P60；
- 不接编码器；
- 不接连杆、SA100、MaixCAM2；
- TB6612 驱动侧 STBY 必须可靠接 GND，暂时不要让 PB1 真正使能驱动；
- 示波器地接 NUCLEO GND。

### 8.2 操作

```text
bench on
pulse 100 500
```

测量：

1. PA10 频率；
2. PA10 低/高电平；
3. PA10 占空比；
4. PC4/PC5 电平；
5. 脉冲期间 MCU 侧 PB1 是否为高；
6. 500 ms 后 PA10、PB1 是否回到低；
7. 再执行 `pulse -100 500`，确认 PA10 占空比不变而 PC4/PC5 对调。

预期：

- PA10 约 20 kHz；
- `pulse 100` 约 10%；
- 正负脉冲只改变方向，不改变 PWM 绝对值；
- 脉冲结束后程序调用急停路径，P60 STBY 为低；
- MCU 不复位。

记录表：

| 项目 | 正脉冲 | 负脉冲 | 结论 |
|---|---:|---:|---|
| PA10 频率 Hz | | | |
| PA10 占空比 % | | | |
| PC4 电平 | | | |
| PC5 电平 | | | |
| 脉冲期间 PB1 | | | |
| 500 ms 后 PB1 | | | |

只有本阶段完全通过，才把 TB6612 STBY 从强制 GND 改接 PB1。

## 9. 阶段 B：P60 脱连杆开环、方向和编码器

### 9.1 接线和机械状态

- 断开 VM 后接 P60、TB6612、PB1、PB6/PB7 编码器；
- 连杆继续脱开，P60 输出端完全无外部负载；
- 输出轴画一条整圈计数参考线；
- 若有条件，串接电流表或观察限流电源电流。

### 9.2 快速复核最低有效原始 PWM

当前源码工作值是 80。不要直接从较高 PWM 开始，依次测试：

```text
zero
pulse 50 200
status
zero
pulse 60 200
status
zero
pulse 70 200
status
zero
pulse 80 200
status
zero
pulse 90 200
status
```

脱开连杆时，这里的 `zero` 只是让下一次 `status` 的 `count` 直接等于本次脉冲造成的计数变化；只有输出轴完全停止后才能清零。负方向把 PWM 改为相应负数，并同样在每次脉冲前清零。每个 PWM 正、负方向各做至少 5 次。每次记录：

- 是否每次都能启动；
- `count` 变化；
- 空载电流；
- 是否有异常噪声、复位或快速温升。

“最低有效 PWM”不是偶尔动一次的值，而是正反方向均能可靠启动的最小值。若正反方向不同，配置值取两者中较大的值，并留少量可靠裕量。

不要在本阶段测所谓“最大 PWM”：`pulse` 被代码限制为 250，本阶段只为确认启动区、方向和编码器。`APP_MOTOR_BEAM_MAX_PWM=700` 仍是闭环安全上限候选，必须等连杆、角度环和电流数据齐全后再决定是否收紧。

### 9.3 统一电机和编码器正方向

临时执行正脉冲并观察输出轴方向。连接机构前先约定“将来使水管右端升高”的旋转方向为正方向。

写回规则：

- 如果 `pulse +100` 的实际方向将来会使右端升高，`APP_MOTOR_BEAM_SIGN` 保持 `1`；
- 否则改为 `-1`，重烧后重新验证；
- 正命令运动时 `count` 应增大，否则把 `APP_ENCODER_BEAM_SIGN` 改为 `-1`；
- 修改 sign 后必须重烧，再用正负脉冲确认一次。

### 9.4 测量输出轴 counts/rev

保持 NUCLEO 逻辑供电和 `bench on`，物理断开电机 VM：

```text
stop
zero
status
```

缓慢转动你定义的“最终输出轴”，不是电机转子轴。准确转一整圈并对齐参考线，然后执行 `status`，记录 `count`。反方向重新做一圈。正、反方向各做三次。

| 次数 | 正向一圈 count | 反向一圈 count 绝对值 |
|---:|---:|---:|
| 1 | | |
| 2 | | |
| 3 | | |
| 平均 | | |

通过条件：

- 同方向三次结果可重复；
- 正向计数为正，反向计数为负；
- 停止转动后 `count` 不再漂移；
- 最终 TI12 计数已经包含定时器的边沿解码，不在软件中再乘 4。

若复核仍约为 3200，则保留当前 `APP_ENCODER_BEAM_CPR=3200.0f`。否则写入三次平均的实测值并记录编码器型号。

### 9.5 本阶段必须写回

```c
APP_MOTOR_BEAM_SIGN
APP_ENCODER_BEAM_SIGN
APP_ENCODER_BEAM_CPR
APP_MOTOR_BEAM_MIN_PWM
```

重烧并再次确认：正命令对应未来右端升高、`count` 增大、最低 PWM 可靠启动。

## 10. 阶段 C：连接机构，建立人工中点和软件安全范围

### 10.1 建立唯一人工安全中点

1. 物理断开 VM；
2. 连接曲柄和连杆，不放钢球；
3. 手动布置曲柄，使其远离两个死点、传力关系良好；
4. 调整水管接近水平；
5. 在曲柄/输出盘与固定支架上各画一条清晰刻线；
6. 将这两条刻线对齐的位置定义为唯一安全中点。

不要安装 PA15 开关。每次上电、复位、重烧、重新连接编码器或执行 `zero` 前，都先回到这组刻线。

逻辑上电、进入台架后：

```text
bench on
stop
zero
status
```

必须看到 `count=0`。如果机构不在刻线位置，禁止执行 `zero`。

### 10.2 逐步建立 count 与水管角度的对应表

仍然不放球。使用当前可靠最低 PWM，从短脉冲开始：

```text
pulse 80 100
status
```

连接机构后的最低启动 PWM 可能高于空载值。如果没有动作且电流正常，不要立刻判断为堵转；保持 100 ms 短脉冲，每次只增加约 10，最多不超过开环上限 250，同时观察电流、噪声和机构位置。任何电流突升、只响不转或接近干涉都要立即断 VM，而不是继续加 PWM。

每次脉冲后都等待机构完全停止，再记录：

- `count`；
- 水管外部量角工具读数；
- 曲柄与死点/干涉位置的距离；
- 电流；
- 是否能顺利反向回到中点。

向正方向逐步点动，再回中点重新 `zero`，然后向负方向重复。不要一口气执行 500 ms。建议记录：

| count | 外部水管角度 ° | 电流 | 机械状态 | 可否安全反向 |
|---:|---:|---:|---|---|
| 0 | | | 安全中点 | |
| 正向点 1 | | | | |
| 正向点 2 | | | | |
| 负向点 1 | | | | |
| 负向点 2 | | | | |

当前代码先用 `[-260,+260]` 阻止开环继续向外运动。若这个范围已经覆盖至少约 `-2°～+2°` 且远离死点，可以直接把它作为第一版软件范围。软件限位不需要逼近真实机械极限；它应明显位于危险位置之前，并保证两侧都能可靠退回中点。

如果在 ±260 内还达不到所需角度，不要直接放宽：先断 VM，检查曲柄半径、连杆安装点和中点选择，人工确认更大计数仍安全后，再小幅修改 `APP_BEAM_ENCODER_MIN/MAX_COUNT`、重烧并继续分段测量。

### 10.3 写回并打开范围门禁

将最终保守范围写入：

```c
#define APP_BEAM_ENCODER_MIN_COUNT  (...)
#define APP_BEAM_ENCODER_MAX_COUNT  (...)
#define APP_BEAM_RANGE_VERIFIED     1
```

范围可以不对称。重烧前把机构放回人工中点；重烧后重新：

```text
bench on
zero
status
```

确认 `count=0 rangeok=1`。

### 10.4 堵转参数当前怎样处理

当前堵转条件是：

```text
|pwm| >= APP_BEAM_STALL_PWM（当前 350）
且 |delta| <= APP_BEAM_STALL_DELTA_COUNT（当前 1）
持续 APP_BEAM_STALL_TIMEOUT_MS（当前 300 ms）
```

台架默认闭环 PWM 上限是 300，原始 `pulse` 最大是 250，所以前述阶段不会触发 350 的堵转判断。不要为了“验证堵转”故意卡住机构或提高输出。等角度环稳定并确实需要超过 350 时，再根据正常运动日志确定阈值，并用安全、可立即断电的受控方法验证。此前必须把这组参数标为 `UNVERIFIED`，但不影响继续进行低 PWM 角度调试。

## 11. 阶段 D：SA100 原始 PWM、零点和方向标定

### 11.1 先确认电气电平

1. 保持 PB14 与 SA100 信号断开；
2. 给 SA100 正确供电并共地；
3. 示波器测量 SA100 输出低/高电平、周期和占空比；
4. 只有确认输出与 STM32 PB14 输入兼容后才连接；
5. 若高电平超过允许范围，使用合适电平转换，不能只凭串联电阻猜测安全。

仓库仍没有可以唯一对应实物的 SA100 手册，因此供电和公式必须来自你的具体传感器资料或实测，本文不虚构固定电压/公式。

### 11.2 采集五个角度的原始数据

不放球，电机停止。将水管分别放在外部可靠量角工具测得的 `-2°、-1°、0°、+1°、+2°`。每个位置保持不动并记录至少 10 帧：

```text
stream on 100
```

记录字段：`per/high/duty/raw/sage`。完成后：

```text
stream off
```

| 外部角度 ° | period 最小/最大 us | high 平均 us | duty 平均 | raw 平均 | 抖动范围 |
|---:|---:|---:|---:|---:|---:|
| -2 | | | | | |
| -1 | | | | | |
| 0 | | | | | |
| +1 | | | | | |
| +2 | | | | | |

### 11.3 计算代码需要的三个参数

当前固件公式是：

```text
raw = duty * scale
beam_angle = (raw - zero) * sign
```

如果具体 SA100 手册确认一圈占空比对应 360°，通常 `scale=360`。此时：

```text
zero = 水管水平时的 duty平均 * 360
```

`sign` 只取 `+1` 或 `-1`，选择使“水管右端升高”时 `ang` 增大的值。没有可靠手册时，用五点数据做线性拟合，检查线性、残差和方向；不要只用水平一个点就声称比例已标定。

在 RAM 中试验，例如：

```text
stop
sa cal 360.0 183.4 -1
status
```

这里的数字只是命令格式示例，不能直接复制。`sa cal` 后等待至少一个新 SA100 周期，再查看：

- 水平时 `ang` 接近 0；
- 右端升高时 `ang` 为正；
- 五个角度点与外部量角工具一致；
- `fresh=1`、`sage` 稳定；
- `sacal=1`。

### 11.4 周期范围和拔线超时

从多帧 `per` 的最小/最大值确定合理裕量，写入 `APP_SA100_PERIOD_MIN/MAX_US`。范围必须覆盖正常抖动，又不能继续保留没有意义的超宽占位范围。

保持电机输出关闭，拔掉 SA100 信号线：

- `sage` 应持续增大；
- 约超过 `APP_SA100_TIMEOUT_MS` 后 `fresh=0`；
- 尝试 `angle 0` 必须被拒绝或正在运行的闭环进入 `sa100_timeout`。

### 11.5 写回并打开 SA100 门禁

写回：

```c
APP_SA100_DUTY_TO_DEG
APP_SA100_HORIZONTAL_RAW_DEG
APP_SA100_ANGLE_SIGN
APP_SA100_PERIOD_MIN_US
APP_SA100_PERIOD_MAX_US
APP_SA100_TIMEOUT_MS
APP_SA100_CALIBRATION_VERIFIED = 1
```

重烧前回到人工安全中点。重烧后执行：

```text
bench on
zero
status
```

必须同时看到：

```text
count=0 rangeok=1 sacal=1 fresh=1
```

## 12. 阶段 E：不放球，调通水管角度内环

### 12.1 闭环前最后检查

- 钢球移除；
- 人工中点刻线对齐并已执行 `zero`；
- `rangeok=1`；
- `sacal=1 fresh=1`；
- 水平时 `ang≈0°`；
- 正命令、正 `count`、正 `ang` 都对应右端升高；
- PC13 急停和 VM 物理断电均可立即操作。

### 12.2 第一次只允许很小输出和角度

```text
stop
limit pwm 100
limit angle 0.3
gain angle 20 0 0
stream on 50
angle 0
```

观察 1～2 s：`aref=0`、`ang`、`pwm`、`count` 不应发散。然后：

```text
angle 0.1
```

正确趋势：

- `aref` 为 +0.1；
- P60 先沿此前已验证的“正逻辑命令使右端升高”方向动作；`pwm` 字段显示的是应用 `APP_MOTOR_BEAM_SIGN` 后的实际 H 桥方向，因此当该 sign 为 `-1` 时，正角度动作的 `pwm` 可以是负值；
- `count` 增大；
- `ang` 向 +0.1 接近；
- 误差减小时 PWM 回落。

若 `ang` 远离目标或机构迅速向边界运动，立即按 PC13 并断 VM。先检查电机 sign、编码器 sign、SA100 sign，禁止用调小 Kp 或增大 Kd 掩盖正反馈。

再测试：

```text
stop
angle -0.1
```

负目标应产生完全相反且有界的趋势。

### 12.3 角度 PID 的实际调节顺序

台架角度参数在 `gain angle` 命令后立即生效，但建议每次先 `stop`：

1. `Ki=0、Kd=0`，逐步增加 Kp，使小目标能可靠响应；
2. 出现明显过冲或往返时，再逐步增加 Kd；
3. 只有 P/D 已稳定但存在可重复静差时，才加入很小 Ki；
4. 一次只改变一个参数；
5. 每组都记录目标、反馈、PWM、count、过冲和稳定时间。

当前 PID 量纲：

- Kp：PWM/degree；
- Ki：PWM/(degree·s)；
- Kd：PWM/(degree/s)，当前实现对角度误差做微分。

建议扩展顺序：

```text
±0.1° → ±0.2° → ±0.5° → ±1° → ±2°
```

每次放宽前先确认当前级别正负方向都能回到 0。调整限幅示例：

```text
stop
limit pwm 120
limit angle 0.5
gain angle <Kp> <Ki> <Kd>
angle 0.5
```

注意：正式闭环使用最低 PWM 补偿。若计算输出只有 1，只要非零，电机实际命令就会提升到 `APP_MOTOR_BEAM_MIN_PWM`。低幅来回抖动可能是最低 PWM 过大、机械回差或 Kp/Kd 不合适，不应盲目增加积分。

### 12.4 空管角度环验收表

最终依次测试：

```text
angle 0
angle 1
angle -1
angle 2
angle -2
angle 0
```

每条命令最长运行 30 s；换目标前可以先 `stop`。记录：

| 目标 ° | 最大实际角 ° | 稳态角 ° | 最大 PWM | count 范围 | 稳定时间 ms | 是否振荡 |
|---:|---:|---:|---:|---:|---:|---|
| 0 | | | | | | |
| +1 | | | | | | |
| -1 | | | | | | |
| +2 | | | | | | |
| -2 | | | | | | |
| 0 | | | | | | |

验收条件：

- 正负目标方向正确；
- 不持续振荡；
- 换向不过度冲击；
- count 始终远离软件边界；
- 断 SA100、越 count 边界或按 PC13 时输出能关闭；
- 0、±1、±2° 均可重复。

完成后执行：

```text
config
stream off
stop
```

将最终角度参数写入正式宏：

```c
APP_BEAM_ANGLE_KP/KI/KD
APP_MOTOR_BEAM_MAX_PWM
APP_BEAM_ANGLE_SOFT_LIMIT_DEG
APP_BENCH_INITIAL_ANGLE_KP/KI/KD
```

台架每次复位默认加载 `APP_BENCH_INITIAL_ANGLE_*`，正常比赛状态机加载 `APP_BEAM_ANGLE_*`。为了重烧后继续球调试，两组都应写入已验证的角度参数；仍保留保守 PWM 和角度限幅。

## 13. 阶段 F：MaixCAM2 五点视觉标定与丢球测试

### 13.1 最终固定相机后再标定

相机必须使用最终刚性支架并随水管运动。改变以下任一项都要重新标定：

- 相机位置或角度；
- 焦距；
- 分辨率；
- 水管/标记位置；
- 光照或曝光策略。

当前 `Vision/cv.py` 使用：

- 左标记像素 `LEFT_REF_X/Y` 对应 `-100 mm`；
- 右标记像素 `RIGHT_REF_X/Y` 对应 `+100 mm`；
- 球心正交投影到两标记连线，再换算为轴向毫米位置；
- UART 帧为 `$B,<x_mm>,<status>\n`；
- `status=1` 是真实检测，2 是短时保持，0 是无效。

### 13.2 更新左右参考点和 ROI

水管保持水平，运行 Maix 程序并查看画面。把左右 `-100/+100 mm` 标记中心的实际像素填入：

```python
LEFT_REF_X / LEFT_REF_Y
RIGHT_REF_X / RIGHT_REF_Y
BALL_GLOBAL_ROI
```

重新运行 Maix 程序。黄色轴线应穿过两个标记和球运动中心线，ROI 应覆盖球的有效运动区但尽量避开水管高光和背景。

### 13.3 五点静态标定

STM32 台架只观察，不开球闭环：

```text
bench on
stream on 100
```

用刻度把球依次放在 `-100、-50、0、+50、+100 mm`，每点保持至少 2 s。记录真实位置和 `ball` 的平均、最小、最大值：

| 真实位置 mm | ball 平均 mm | 最小/最大 mm | 绝对误差 mm | vision=1 比例 |
|---:|---:|---:|---:|---:|
| -100 | | | | |
| -50 | | | | |
| 0 | | | | |
| +50 | | | | |
| +100 | | | | |

检查：

- 右侧位置为正，左侧为负；
- `vframe` 持续增加；
- 正常检测时 `vision=1 vfresh=1`；
- `vage` 通常明显小于 200 ms；
- 静态误差应明显小于比赛总允许误差 10 mm，建议视觉自身尽量控制在几毫米级；
- 球静止时 `vel` 应回到接近 0，而不是长期大幅跳变。

如果方向相反，优先交换左右标记对应关系或修正视觉标定，不在 STM32 多处加负号。

### 13.4 遮挡和超时

遮住球：

1. 前两帧可能出现 `vision=2`；
2. `vframe` 仍可能增加，但 `vage` 不应因为 status=2 清零；
3. 超过 `APP_VISION_TIMEOUT_MS` 后 `vfresh=0`；
4. 正在运行的 `ball` 模式应进入 `vision_timeout` 并关闭输出。

重新出现真实球后，必须重新看到 `vision=1` 才算恢复测量。

### 13.5 动态方向和延迟

手动缓慢把球从左向右移动：

- `ball` 应增加；
- `vel` 应短时为正；
- 从右向左时 `vel` 应为负；
- 记录画面运动到 STM32 `ball` 变化的大致延迟和有效帧率。

完成后 `stream off`，不要立刻闭环；先保存视觉五点数据。

## 14. 阶段 G：脱离小车调试钢球位置外环

### 14.1 只有全部前置条件通过才放球

必须同时满足：

- 空管角度环已通过 0、±1、±2°；
- `rangeok=1`；
- `sacal=1 fresh=1`；
- 视觉五点标定和遮挡超时已通过；
- 人工中点已对齐并执行 `zero`；
- 整个装置固定在稳定桌面，不安装到运动小车；
- 可以立即按 PC13 或断 VM。

### 14.2 先用很小角度保持中心

在同一次台架会话中重新输入已经验证的角度 PID，或者确认 `config` 显示的是最终值：

```text
stop
zero
limit pwm 120
limit angle 0.3
gain angle <已验证Kp> <已验证Ki> <已验证Kd>
gain ball 0.003 0 1
stream on 50
ball 0
```

`0.003` 只是低增益起步值，不是最终参数。轻轻把球偏离中心 10～20 mm，观察是否返回 0。

正确现象：

- 球在目标右侧时，控制最终使球向左回目标；
- 球在目标左侧时，控制最终使球向右回目标；
- `aref` 始终受当前 `alim=0.3°` 限制；
- SA100 角度环能跟随 `aref`；
- P60 count 不接近软件边界。

如果球越来越远：

```text
stop
gain ball 0.003 0 -1
ball 0
```

只翻转 `bsign`，不要同时修改相机方向、SA sign 和电机 sign。重新用 10～20 mm 小扰动验证。

### 14.3 球 PD 调参顺序

外环公式与代码一致：

```text
position_error = ball_target - ball_position
angle_ref = ball_sign * (Kp*position_error - Kd*ball_speed)
```

调节步骤：

1. `Kd=0`，小步增加 Kp，直到球有明确回到目标的趋势；
2. 若球在目标两侧持续往返，再小步增加 Kd；
3. 先不加球位置积分，当前台架命令也没有球位置 Ki；
4. 每次只改 Kp 或 Kd 之一；
5. 先保持 `alim=0.3°`，稳定后才放宽到 0.5°、1°，确有必要才到 2°；
6. 每组测试结束先 `stop`，保存日志再继续。

记录：

| bkp | bkd | bsign | alim ° | 初始偏差 mm | 最大超调 mm | 稳定时间 ms | 结论 |
|---:|---:|---:|---:|---:|---:|---:|---|
| | | | | | | | |

### 14.4 单目标测试

中心稳定后依次测试：

```text
ball 0
ball 50
ball -50
```

每条命令会维持当前目标，最长 30 s。观察并记录：

- `bref`；
- `ball`；
- `vel`；
- `aref/ang`；
- `pwm/count`；
- 首次进入 ±10 mm 的时间；
- 连续稳定 250 ms 的时间；
- 是否出现视觉丢失、角度限位、编码器限位或振荡。

不要在目标切换时用手帮球。若球接近水管危险端、视觉丢失或振荡增大，立即急停。

### 14.5 直接运行五秒自动题目序列

把球人工放回 `0 mm`，误差必须在 ±15 mm 内，然后：

```text
stop
zero
stream on 50
ball sequence
```

当前代码会：

1. 检查真实视觉和 SA100 新鲜；
2. 检查球初始位置在 `0±15 mm`；
3. 立即把目标设为 `+50 mm`；
4. 当误差 ≤10 mm、球速绝对值 ≤25 mm/s，并连续保持 250 ms 后切换目标到 `-50 mm`；
5. 在 `-50 mm` 满足同样稳定条件后停止输出；
6. 若全过程超过 5000 ms，进入 `sequence_timeout`；
7. 成功时回复 `OK ball sequence complete elapsed=... ms`。

状态字段：

- `mode=ball_sequence`；
- `seq=0` 表示正在去 +50；
- `seq=1` 表示正在去 -50；
- `seq=2` 表示已完成；
- `seqms` 表示总用时。

至少重复 10 次，记录成功率、最大误差、用时、电源电压和失败原因。一次成功不能视为完成。

## 15. 把台架 RAM 参数写回正式固件

调参满意后执行：

```text
config
stream off
stop
```

至少核对并写回：

```text
APP_MOTOR_BEAM_SIGN
APP_ENCODER_BEAM_SIGN
APP_ENCODER_BEAM_CPR
APP_MOTOR_BEAM_MIN_PWM
APP_MOTOR_BEAM_MAX_PWM
APP_BEAM_ENCODER_MIN/MAX_COUNT
APP_BEAM_RANGE_VERIFIED
APP_BEAM_STALL_*
APP_SA100_*
APP_SA100_CALIBRATION_VERIFIED
APP_BEAM_ANGLE_KP/KI/KD
APP_BENCH_INITIAL_ANGLE_KP/KI/KD
APP_BALL_KP/KD
APP_BALL_CONTROL_SIGN
APP_BALL_ANGLE_LIMIT_DEG
APP_VISION_TIMEOUT_MS
```

运行时 `plim`、`alim`、角度 PID 和球 PD 不会自动保存，必须人工写回。写回后重新编译、烧录、对齐人工中点、`zero`，再完整运行角度验收和 `ball sequence`。只有重烧后的参数仍能重复通过，才说明正式固件配置正确。

## 16. 调试结束后移除台架功能

静态独立装置多次通过后：

```c
#define APP_ENABLE_BENCH_DEBUG 0
```

重新完整编译。`bench_debug.c` 会编译成不能使能电机的空桩，LPUART1 不再接收台架命令。关闭调试功能前必须确保所有最终参数已经写回 `app_config.h`，因为 RAM 调参值会全部消失。

暂时不要安装到底盘。先完成并保存以下证据：

- 阶段 A PWM 波形；
- 阶段 B 方向、最低 PWM、三次 counts/rev；
- 阶段 C 中点刻线照片和 count/角度表；
- 阶段 D SA100 五点原始数据和拔线超时；
- 阶段 E 0、±1、±2° 日志；
- 阶段 F 视觉五点误差和遮挡日志；
- 阶段 G 至少 10 次 `ball sequence` 结果。

这些全部通过后，才开始左右轮、循迹和小车运动对钢球的扰动调试。

## 17. 故障后的处理

| fault | 含义 | 处理顺序 |
|---|---|---|
| `estop` | PC13 急停 | 断 VM，检查现场，执行 `stop` 后重新开始 |
| `sa100_timeout` | SA100 超过 100 ms 无有效帧 | 查 PB14、供电、周期范围和电平 |
| `angle_limit` | 实际水管角度超过软件软限位 | 断 VM，查方向、PID、机构和 SA 标定 |
| `encoder_limit` | P60 count 超出软件范围 | 断 VM，人工回中点，检查 sign/zero/边界 |
| `stall` | 大 PWM 下编码器长时间无变化 | 查死点、干涉、编码器和电流；禁止直接放宽保护 |
| `vision_timeout` | 200 ms 内没有真实 status=1 | 查 Maix 检测、UART、遮挡和光照 |
| `sequence_timeout` | 自动 +50/-50 序列超过 5 s | 保存日志，先判断角度环、视觉还是球 PD 导致 |

故障后先保存当时最后几帧日志，再执行：

```text
stop
status
```

不能通过关闭超时、放宽限位或增大 PWM 来掩盖故障。
