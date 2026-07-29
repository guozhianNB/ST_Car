#include "pid.h"

static float Clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void PID_Init(PidController *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
  if (pid == 0) return;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  PID_Reset(pid);
}

void PID_Reset(PidController *pid)
{
  if (pid == 0) return;
  pid->integral = 0.0f;
  pid->previous_error = 0.0f;
  pid->initialized = false;
}

float PID_Update(PidController *pid, float error, float dt_s)
{
  float derivative = 0.0f;
  float candidate_integral;
  float output;

  if ((pid == 0) || (dt_s <= 0.0f)) return 0.0f;
  if (pid->initialized) derivative = (error - pid->previous_error) / dt_s;
  candidate_integral = Clamp(pid->integral + error * dt_s,
                             pid->integral_limit);
  output = pid->kp * error + pid->ki * candidate_integral + pid->kd * derivative;

  /* Conditional integration: do not wind up farther into saturation. */
  if ((output <= pid->output_limit && output >= -pid->output_limit) ||
      (output > pid->output_limit && error < 0.0f) ||
      (output < -pid->output_limit && error > 0.0f)) {
    pid->integral = candidate_integral;
  }
  pid->previous_error = error;
  pid->initialized = true;
  return Clamp(output, pid->output_limit);
}

float PID_UpdateWithRate(PidController *pid, float error, float measured_rate,
                         float dt_s)
{
  float candidate_integral;
  float output;
  if ((pid == 0) || (dt_s <= 0.0f)) return 0.0f;
  candidate_integral = Clamp(pid->integral + error * dt_s,
                             pid->integral_limit);
  output = pid->kp * error + pid->ki * candidate_integral - pid->kd * measured_rate;
  if ((output <= pid->output_limit && output >= -pid->output_limit) ||
      (output > pid->output_limit && error < 0.0f) ||
      (output < -pid->output_limit && error > 0.0f)) {
    pid->integral = candidate_integral;
  }
  pid->previous_error = error;
  pid->initialized = true;
  return Clamp(output, pid->output_limit);
}
