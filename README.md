# ST_Car — 接口文档

## motor (`Core/Inc/motor.h`)

```c
void Motor_Init(void);
// 初始化 PWM + 编码器，使用前必须调用一次

void Motor_SetSpeed(int speedL, int speedR);
// speedL/speedR: -1000~1000, 正数前进负数后退

int Motor_GetEncoderCount(void);   // 返回: 左右编码器平均值
int Motor_GetEncoderCountL(void);  // 返回: 左轮编码器计数值
int Motor_GetEncoderCountR(void);  // 返回: 右轮编码器计数值

double Get_SpeedL(int dt_ms);      // dt_ms: 时间间隔(ms)
double Get_SpeedR(int dt_ms);      // 返回: 归一化速度
```

## pid_control (`Core/Inc/pid_control.h`)

```c
void PID_Control_Init(void);
// 清零 PID 内部状态，启动时调用一次

void PID_Control_SetTarget(int target_speed, float track_error,
                           int *out_speedL, int *out_speedR);
// target_speed: 0~1000, track_error: 寻迹偏离(0=正中)
// out_speedL/R: [输出] 左右轮速度 -1000~1000

void PID_Control_Reset(void);
// 重置 PID 积分和上一拍误差

void PID_Control_TuneEncoderPID(float kp, float ki, float kd);
void PID_Control_TuneTrackingPID(float kp, float ki, float kd);
// 在线改参，传负数表示不修改该项
```