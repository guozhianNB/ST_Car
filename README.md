# ST_Car — 2026 电赛 H 题固件与视觉

本仓库实现 NUCLEO-G491RE 车载平衡滚球系统：双轮差速循迹、P60 曲柄连杆水管角度控制、SA100 真实角度反馈，以及 MaixCAM2 钢球位置视觉。

开始开发前先阅读：

- [AGENT.md](AGENT.md)：题目事实、已确认方案、当前真实状态和调试约束；
- [docs/stm32-pinout.md](docs/stm32-pinout.md)：完整接线、定时器、DMA 和中断分配；
- [docs/firmware-architecture.md](docs/firmware-architecture.md)：代码分层、调度、状态机、安全行为和调试顺序。

核心入口是 `Core/Src/main.c` 与 `Core/Src/app.c`，所有暂定参数集中在 `Core/Inc/app_config.h`。视觉端位于 `Vision/cv.py`，当前串口协议为：

```text
$B,<x_mm>,<status>\n
```

其中只有 `status=1` 是真实新测量；`status=2` 是短时保持，不能用于刷新球速或视觉超时。

当前状态：软件框架、IOC、IAR 工程和 VS Code/CMake 调试工程已经完成并通过 Cortex-M4 交叉编译/链接；电机方向、编码器分辨率、SA100 标定、安全边界和 PID 仍必须在实物上按文档顺序验证。首次烧录必须架空车轮、脱开连杆并物理断开 TB6612 STBY。
