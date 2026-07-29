#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 电机初始化 */
void Motor_Init(void);

/* 设置电机速度，范围[-1000, 1000]，正数前进，负数后退 */
void Motor_SetSpeed(int speedL, int speedR);

/* 获取编码器计数值 */
int Motor_GetEncoderCount(void);
int Motor_GetEncoderCountL(void);
int Motor_GetEncoderCountR(void);

/* 获取左右轮速度（单位：编码器计数/毫秒归一化值） */
double Get_SpeedL(int dt_ms);
double Get_SpeedR(int dt_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
