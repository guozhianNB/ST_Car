#ifndef __PID_CONTROL_H__
#define __PID_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*===========================================================================
 * PID 控制模块
 *
 * 提供两种 PID 校正的叠加:
 *   1) 编码器直线修正（底层） — 通过左右轮编码器速度差保持直线
 *   2) 寻迹 PID（上层）      — 通过外部红外传感器误差跟踪路径
 *
 * 调用方式（在主循环中反复调用）:
 *   int speedL, speedR;
 *   PID_Control_SetTarget(期望速度, 寻迹误差, &speedL, &speedR);
 *   Motor_SetSpeed(speedL, speedR);
 *===========================================================================*/

/* 初始化 PID 控制器（恢复默认参数、清零内部状态） */
void PID_Control_Init(void);

/*
 * 设置目标并更新 PID 控制（每次主循环调用一次）
 *
 * @param target_speed  目标速度 (0 ~ 1000)
 * @param track_error   寻迹误差（红外传感器，无单位，0 = 在路径正中）
 * @param out_speedL    输出：左轮电机速度 (-1000 ~ 1000)
 * @param out_speedR    输出：右轮电机速度 (-1000 ~ 1000)
 *
 * 注：函数内部通过 HAL_GetTick() 计算时间差 dt，不依赖外部定时。
 */
void PID_Control_SetTarget(int target_speed, float track_error,
                           int *out_speedL, int *out_speedR);

/* 重置所有 PID 内部状态（积分、上一拍误差） */
void PID_Control_Reset(void);

/* 在线调参（设 0 或负数表示不修改该参数） */
void PID_Control_TuneEncoderPID(float kp, float ki, float kd);
void PID_Control_TuneTrackingPID(float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROL_H__ */
