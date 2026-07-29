#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float integral_limit;
  float output_limit;
  bool initialized;
} PidController;

void PID_Init(PidController *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);
void PID_Reset(PidController *pid);
float PID_Update(PidController *pid, float error, float dt_s);
float PID_UpdateWithRate(PidController *pid, float error, float measured_rate,
                         float dt_s);

#endif /* PID_H */
