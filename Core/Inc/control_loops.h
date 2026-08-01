#ifndef CONTROL_LOOPS_H
#define CONTROL_LOOPS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CONTROL_BALL_SEQUENCE_OFF = 0,
  CONTROL_BALL_SEQUENCE_POSITIVE_CONTROL,
  CONTROL_BALL_SEQUENCE_NEGATIVE_CONTROL
} ControlBallSequenceStage;

typedef struct {
  float left_target_mm_s;
  float right_target_mm_s;
  float left_ramped_mm_s;
  float right_ramped_mm_s;
  float ball_target_mm;
  float ball_pid_output_steps_s;
  float ball_integral_steps_s;
  float ball_target_position_steps;
  float ball_command_position_steps;
  float ball_acceleration_mm_s2;
  float ball_acceleration_steps_s;
  float line_steer_mm_s;
  int16_t left_pwm;
  int16_t right_pwm;
  int32_t stepper_target_rate_steps_s;
  int32_t stepper_rate_steps_s;
  ControlBallSequenceStage ball_sequence_stage;
  bool chassis_enabled;
  bool actuator_enabled;
  bool ball_enabled;
} ControlStatus;

void ControlLoops_Init(void);
void ControlLoops_Reset(void);
void ControlLoops_EnableChassis(bool enable);
void ControlLoops_EnableActuator(bool enable);
void ControlLoops_EnableBall(bool enable);
void ControlLoops_StartBallSequence(uint32_t now_ms);
void ControlLoops_StartBallSequenceNegative(uint32_t now_ms);
void ControlLoops_FinishBallSequenceHold(void);
void ControlLoops_SetBaseSpeed(float speed_mm_s);
void ControlLoops_SetBallTarget(float position_mm);
void ControlLoops_SetBallGains(float kp, float ki, float kd, float sign);
void ControlLoops_EnableStaticReleaseCatch(bool enable);
void ControlLoops_SetBallAccelerationGain(float ka);
void ControlLoops_SetStepperRateLimit(float limit_steps_s);
void ControlLoops_StartMovingZero(void);
void ControlLoops_SetMovingZeroGains(float kx, float kv, float ki,
                                     float kq, float ka);
void ControlLoops_SetMovingZeroLimits(float rate_steps_s,
                                      float pose_steps);
bool ControlLoops_IsMovingZeroActive(void);
void ControlLoops_ManualStepperRun(int32_t rate_steps_s, uint32_t duration_ms,
                                   uint32_t now_ms);
void ControlLoops_LineUpdate(uint32_t dt_ms);
void ControlLoops_FastUpdate(uint32_t dt_ms);
const ControlStatus *ControlLoops_GetStatus(void);

#endif /* CONTROL_LOOPS_H */
