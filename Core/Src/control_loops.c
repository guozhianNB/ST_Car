#include "control_loops.h"
#include "app_config.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "pid.h"
#include "sa100.h"
#include "vision_uart.h"
#include <math.h>

static PidController wheel_left_pid;
static PidController wheel_right_pid;
static PidController beam_angle_pid;
static ControlStatus status;
static float requested_base_speed;
static float line_previous_error;
static bool line_initialized;

static float Clamp(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static float Ramp(float current, float target, float maximum_step)
{
  float delta = target - current;
  if (delta > maximum_step) delta = maximum_step;
  if (delta < -maximum_step) delta = -maximum_step;
  return current + delta;
}

void ControlLoops_Init(void)
{
  PID_Init(&wheel_left_pid, APP_WHEEL_KP, APP_WHEEL_KI, 0.0f,
           APP_WHEEL_PWM_LIMIT / APP_WHEEL_KI, APP_WHEEL_PWM_LIMIT);
  PID_Init(&wheel_right_pid, APP_WHEEL_KP, APP_WHEEL_KI, 0.0f,
           APP_WHEEL_PWM_LIMIT / APP_WHEEL_KI, APP_WHEEL_PWM_LIMIT);
  PID_Init(&beam_angle_pid, APP_BEAM_ANGLE_KP, APP_BEAM_ANGLE_KI,
           APP_BEAM_ANGLE_KD, 30.0f, APP_MOTOR_BEAM_MAX_PWM);
  ControlLoops_Reset();
}

void ControlLoops_Reset(void)
{
  PID_Reset(&wheel_left_pid);
  PID_Reset(&wheel_right_pid);
  PID_Reset(&beam_angle_pid);
  requested_base_speed = 0.0f;
  line_previous_error = 0.0f;
  line_initialized = false;
  status.left_target_mm_s = 0.0f;
  status.right_target_mm_s = 0.0f;
  status.left_ramped_mm_s = 0.0f;
  status.right_ramped_mm_s = 0.0f;
  status.beam_target_deg = 0.0f;
  status.ball_target_mm = 0.0f;
  status.line_steer_mm_s = 0.0f;
  status.left_pwm = 0;
  status.right_pwm = 0;
  status.beam_pwm = 0;
  status.chassis_enabled = false;
  status.beam_enabled = false;
  status.ball_enabled = false;
  Motor_EnableChassis(false);
  Motor_EnableBeam(false);
}

void ControlLoops_EnableChassis(bool enable)
{
  status.chassis_enabled = enable;
  Motor_EnableChassis(enable);
  if (!enable) {
    status.left_target_mm_s = 0.0f;
    status.right_target_mm_s = 0.0f;
    status.left_ramped_mm_s = 0.0f;
    status.right_ramped_mm_s = 0.0f;
    PID_Reset(&wheel_left_pid);
    PID_Reset(&wheel_right_pid);
    Motor_Brake(MOTOR_LEFT);
    Motor_Brake(MOTOR_RIGHT);
  }
}

void ControlLoops_EnableBeam(bool enable)
{
  status.beam_enabled = enable;
  Motor_EnableBeam(enable);
  if (!enable) {
    status.beam_pwm = 0;
    PID_Reset(&beam_angle_pid);
    Motor_Brake(MOTOR_BEAM);
  }
}

void ControlLoops_EnableBall(bool enable)
{
  status.ball_enabled = enable;
  if (!enable) status.beam_target_deg = 0.0f;
}

void ControlLoops_SetBaseSpeed(float speed_mm_s)
{
  requested_base_speed = speed_mm_s;
}

void ControlLoops_SetBallTarget(float position_mm)
{
  status.ball_target_mm = Clamp(position_mm, -APP_BALL_TARGET_LIMIT_MM,
                                APP_BALL_TARGET_LIMIT_MM);
}

void ControlLoops_SetDirectBeamTarget(float angle_deg)
{
  status.ball_enabled = false;
  status.beam_target_deg = Clamp(angle_deg, -APP_BEAM_INITIAL_CMD_LIMIT_DEG,
                                APP_BEAM_INITIAL_CMD_LIMIT_DEG);
}

void ControlLoops_LineUpdate(uint32_t dt_ms)
{
  const LineSensorSample *line = LineSensor_Get();
  float derivative = 0.0f;
  float magnitude;
  float base;
  if (!status.chassis_enabled || (dt_ms == 0U)) return;
  if (line_initialized) {
    derivative = (line->error - line_previous_error) * 1000.0f / (float)dt_ms;
  }
  line_previous_error = line->error;
  line_initialized = true;
  status.line_steer_mm_s = Clamp(APP_LINE_KP * line->error + APP_LINE_KD * derivative,
                                 -APP_LINE_STEER_LIMIT_MM_S,
                                 APP_LINE_STEER_LIMIT_MM_S);
  magnitude = fabsf(line->error);
  if (magnitude > 1.0f) magnitude = 1.0f;
  base = requested_base_speed;
  if (base > 0.0f) {
    float curve_base = APP_CHASSIS_CURVE_SPEED_MM_S;
    if (curve_base > base) curve_base = base;
    base -= (base - curve_base) * magnitude;
  }
  status.left_target_mm_s = base - status.line_steer_mm_s;
  status.right_target_mm_s = base + status.line_steer_mm_s;
}

void ControlLoops_FastUpdate(uint32_t dt_ms)
{
  const EncoderSample *left = Encoder_Get(ENCODER_LEFT);
  const EncoderSample *right = Encoder_Get(ENCODER_RIGHT);
  const EncoderSample *beam_encoder = Encoder_Get(ENCODER_BEAM);
  const Sa100Sample *angle = SA100_Get();
  VisionBallSample vision;
  float dt_s;

  if (dt_ms == 0U) return;
  dt_s = (float)dt_ms * 0.001f;
  if (status.chassis_enabled && left != 0 && right != 0) {
    float step = APP_WHEEL_ACCEL_LIMIT_MM_S2 * dt_s;
    status.left_ramped_mm_s = Ramp(status.left_ramped_mm_s,
                                   status.left_target_mm_s, step);
    status.right_ramped_mm_s = Ramp(status.right_ramped_mm_s,
                                    status.right_target_mm_s, step);
    status.left_pwm = (int16_t)PID_Update(&wheel_left_pid,
                          status.left_ramped_mm_s - left->speed_mm_s, dt_s);
    status.right_pwm = (int16_t)PID_Update(&wheel_right_pid,
                          status.right_ramped_mm_s - right->speed_mm_s, dt_s);
    Motor_Set(MOTOR_LEFT, status.left_pwm);
    Motor_Set(MOTOR_RIGHT, status.right_pwm);
  }

  if (status.ball_enabled && VisionUART_ConsumeNewFrame(&vision) && vision.valid) {
    float position_error = status.ball_target_mm - vision.position_mm;
    status.beam_target_deg = APP_BALL_CONTROL_SIGN *
      (APP_BALL_KP * position_error - APP_BALL_KD * vision.speed_mm_s);
    status.beam_target_deg = Clamp(status.beam_target_deg,
                                   -APP_BALL_ANGLE_LIMIT_DEG,
                                   APP_BALL_ANGLE_LIMIT_DEG);
  }

  if (status.beam_enabled && (beam_encoder != 0) && angle->valid) {
    int16_t command = (int16_t)PID_Update(&beam_angle_pid,
      status.beam_target_deg - angle->beam_angle_deg, dt_s);
    if ((beam_encoder->total_count <= APP_BEAM_ENCODER_MIN_COUNT && command < 0) ||
        (beam_encoder->total_count >= APP_BEAM_ENCODER_MAX_COUNT && command > 0)) {
      command = 0;
      PID_Reset(&beam_angle_pid);
    }
    status.beam_pwm = command;
    Motor_Set(MOTOR_BEAM, command);
  } else if (status.beam_enabled) {
    status.beam_pwm = 0;
    Motor_Brake(MOTOR_BEAM);
  }
}

const ControlStatus *ControlLoops_GetStatus(void)
{
  return &status;
}
