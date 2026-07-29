#ifndef CONTROL_LOOPS_H
#define CONTROL_LOOPS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float left_target_mm_s;
  float right_target_mm_s;
  float left_ramped_mm_s;
  float right_ramped_mm_s;
  float beam_target_deg;
  float ball_target_mm;
  float line_steer_mm_s;
  int16_t left_pwm;
  int16_t right_pwm;
  int16_t beam_pwm;
  bool chassis_enabled;
  bool beam_enabled;
  bool ball_enabled;
} ControlStatus;

void ControlLoops_Init(void);
void ControlLoops_Reset(void);
void ControlLoops_EnableChassis(bool enable);
void ControlLoops_EnableBeam(bool enable);
void ControlLoops_EnableBall(bool enable);
void ControlLoops_SetBaseSpeed(float speed_mm_s);
void ControlLoops_SetBallTarget(float position_mm);
void ControlLoops_SetDirectBeamTarget(float angle_deg);
void ControlLoops_LineUpdate(uint32_t dt_ms);
void ControlLoops_FastUpdate(uint32_t dt_ms);
const ControlStatus *ControlLoops_GetStatus(void);

#endif /* CONTROL_LOOPS_H */
