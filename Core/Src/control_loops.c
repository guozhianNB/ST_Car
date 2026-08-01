#include "control_loops.h"
#include "app_config.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "pid.h"
#include "stepper.h"
#include "vision_uart.h"
#include "stm32g4xx_hal.h"
#include <math.h>

static PidController wheel_left_pid;
static PidController wheel_right_pid;
static ControlStatus status;
static float requested_base_speed;
static float line_previous_error;
static bool line_initialized;
static float ball_kp;
static float ball_ki;
static float ball_kd;
static float ball_ka;
static float ball_sign;
static float stepper_rate_limit;
static float ball_integral_error_s;
static float ball_requested_rate_steps_s;
static float ball_target_position_steps;
static float ball_command_position_steps;
static float ball_pose_bias_steps;
static float stepper_ramped_rate_steps_s;
static uint32_t ball_last_measurement_ms;
static uint32_t ball_integral_stationary_since_ms;
static float ball_static_release_reference_mm;
static bool ball_static_release_reference_valid;
static bool ball_static_release_catch_active;
static bool ball_static_release_catch_enabled;
static float ball_previous_speed_mm_s;
static uint32_t ball_accel_reference_ms;
static float ball_acceleration_mm_s2;
static bool ball_endpoint_release_active;
static bool ball_endpoint_release_latched;
static float ball_endpoint_command_direction;
static uint8_t ball_endpoint_release_confirm_frames;
static bool moving_zero_enabled;
static float zero_kx;
static float zero_kv;
static float zero_ki;
static float zero_kq;
static float zero_ka;
static float zero_rate_limit;
static float zero_pose_limit;
static float zero_x_mm;
static float zero_v_mm_s;
static float zero_a_mm_s2;
static float zero_integral_mm_s;
static float zero_pose_steps;
static float zero_previous_measured_speed_mm_s;
static uint32_t zero_last_measurement_ms;
static int32_t manual_stepper_rate_steps_s;
static uint32_t manual_stepper_until_ms;

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

static void ResetBallPid(void)
{
  ball_integral_error_s = 0.0f;
  ball_requested_rate_steps_s = 0.0f;
  ball_target_position_steps = ball_command_position_steps;
  stepper_ramped_rate_steps_s = 0.0f;
  ball_last_measurement_ms = 0U;
  ball_integral_stationary_since_ms = 0U;
  ball_static_release_reference_mm = 0.0f;
  ball_static_release_reference_valid = false;
  ball_static_release_catch_active = false;
  ball_previous_speed_mm_s = 0.0f;
  ball_accel_reference_ms = 0U;
  ball_acceleration_mm_s2 = 0.0f;
  ball_endpoint_release_active = false;
  ball_endpoint_release_latched = false;
  ball_endpoint_command_direction = 0.0f;
  ball_endpoint_release_confirm_frames = 0U;
  status.ball_pid_output_steps_s = 0.0f;
  status.ball_integral_steps_s = 0.0f;
  status.ball_target_position_steps = ball_target_position_steps;
  status.ball_command_position_steps = ball_command_position_steps;
  status.ball_acceleration_mm_s2 = 0.0f;
  status.ball_acceleration_steps_s = 0.0f;
  status.stepper_target_rate_steps_s = 0;
  status.stepper_rate_steps_s = 0;
}

static void ResetMovingZero(void)
{
  moving_zero_enabled = false;
  zero_x_mm = 0.0f;
  zero_v_mm_s = 0.0f;
  zero_a_mm_s2 = 0.0f;
  zero_integral_mm_s = 0.0f;
  zero_pose_steps = 0.0f;
  zero_previous_measured_speed_mm_s = 0.0f;
  zero_last_measurement_ms = 0U;
}

static float MovingZeroUpdate(const VisionBallSample *measurement,
                              bool new_measurement, uint32_t now_ms,
                              float dt_s, bool *valid)
{
  float predicted_x;
  float unsaturated_rate;
  float rate;

  if (new_measurement) {
    if (zero_last_measurement_ms == 0U) {
      zero_v_mm_s = measurement->speed_mm_s;
      zero_a_mm_s2 = 0.0f;
    } else {
      uint32_t frame_dt_ms = measurement->timestamp_ms -
        zero_last_measurement_ms;
      if ((frame_dt_ms >= 10U) && (frame_dt_ms <= 100U)) {
        float frame_dt_s = (float)frame_dt_ms * 0.001f;
        float raw_accel = (measurement->speed_mm_s -
          zero_previous_measured_speed_mm_s) / frame_dt_s;
        raw_accel = Clamp(raw_accel, -2000.0f, 2000.0f);
        zero_a_mm_s2 += APP_ZERO_ACCEL_FILTER_ALPHA *
          (raw_accel - zero_a_mm_s2);
      } else {
        zero_a_mm_s2 = 0.0f;
      }
      zero_v_mm_s += APP_ZERO_VELOCITY_FILTER_ALPHA *
        (measurement->speed_mm_s - zero_v_mm_s);
    }
    zero_x_mm = measurement->position_mm;
    zero_previous_measured_speed_mm_s = measurement->speed_mm_s;
    zero_last_measurement_ms = measurement->timestamp_ms;
  }

  if ((zero_last_measurement_ms == 0U) ||
      ((now_ms - zero_last_measurement_ms) > APP_ZERO_VISION_HOLD_MS)) {
    *valid = false;
    zero_integral_mm_s = 0.0f;
    return 0.0f;
  }

  predicted_x = zero_x_mm + zero_v_mm_s *
    ((float)(now_ms - zero_last_measurement_ms) * 0.001f);
  zero_integral_mm_s += predicted_x * dt_s;
  zero_integral_mm_s *= Clamp(1.0f - APP_ZERO_INTEGRAL_LEAK_PER_S * dt_s,
                              0.0f, 1.0f);
  zero_integral_mm_s = Clamp(zero_integral_mm_s,
    -APP_ZERO_INTEGRAL_LIMIT_MM_S, APP_ZERO_INTEGRAL_LIMIT_MM_S);

  unsaturated_rate = zero_kx * predicted_x + zero_kv * zero_v_mm_s +
    zero_ki * zero_integral_mm_s - zero_kq * zero_pose_steps +
    zero_ka * zero_a_mm_s2;
  rate = Clamp(unsaturated_rate, -zero_rate_limit, zero_rate_limit);

  /* Pose limiting is independent of the encoder end guards.  It bounds the
   * amount of tube attitude this new controller may accumulate around its
   * activation pose, so a stuck ball cannot wind the mechanism indefinitely. */
  if (((zero_pose_steps >= zero_pose_limit) && (rate > 0.0f)) ||
      ((zero_pose_steps <= -zero_pose_limit) && (rate < 0.0f))) {
    rate = 0.0f;
  }
  status.ball_pid_output_steps_s = rate;
  status.ball_integral_steps_s = zero_ki * zero_integral_mm_s;
  status.ball_target_position_steps = 0.0f;
  status.ball_command_position_steps = zero_pose_steps;
  status.ball_acceleration_mm_s2 = zero_a_mm_s2;
  status.ball_acceleration_steps_s = zero_ka * zero_a_mm_s2;
  *valid = true;
  return rate;
}

static float BallVisualPid(const VisionBallSample *vision)
{
  float control_target_mm = status.ball_target_mm;
  float derivative_gain = ball_kd;
  float error;
  float dt_s = 0.0f;
  float pd;
  float relative_pd_steps;
  float target_position_steps;
  bool endpoint_handoff = false;
  uint32_t measurement_ms = vision->timestamp_ms;

  /* Requirement 3 changes stage on the first real frame inside +50 +/-10 mm.
   * The positive leg therefore must cross +40 mm rather than settle at +50.
   * A temporary target lead preserves drive through that boundary.  During
   * the negative leg the same lead provides fast reversal, then drops out at
   * a fixed approach point so the ordinary visual PD performs the braking and
   * final hold.  The published/judged targets remain exactly +/-50 mm. */
  if (status.ball_sequence_stage ==
      CONTROL_BALL_SEQUENCE_POSITIVE_CONTROL) {
    control_target_mm += APP_BALL_SEQUENCE_POSITIVE_LEAD_MM;
    derivative_gain *= APP_BALL_SEQUENCE_CHASE_KD_SCALE;
  } else if ((status.ball_sequence_stage ==
              CONTROL_BALL_SEQUENCE_NEGATIVE_CONTROL) &&
             (vision->position_mm >
              APP_BALL_SEQUENCE_NEGATIVE_BRAKE_MM)) {
    control_target_mm -= APP_BALL_SEQUENCE_NEGATIVE_LEAD_MM;
    derivative_gain *= APP_BALL_SEQUENCE_CHASE_KD_SCALE;
  }
  error = control_target_mm - vision->position_mm;

  if (ball_last_measurement_ms != 0U) {
    dt_s = (float)(measurement_ms - ball_last_measurement_ms) * 0.001f;
  }
  if (ball_accel_reference_ms == 0U) {
    ball_previous_speed_mm_s = vision->speed_mm_s;
    ball_accel_reference_ms = measurement_ms;
  } else {
    uint32_t accel_dt_ms = measurement_ms - ball_accel_reference_ms;
    if (accel_dt_ms >= APP_BALL_ACCEL_SAMPLE_MIN_MS) {
      if (accel_dt_ms <= APP_BALL_ACCEL_SAMPLE_MAX_MS) {
        float accel_dt_s = (float)accel_dt_ms * 0.001f;
        float raw_acceleration = (vision->speed_mm_s -
          ball_previous_speed_mm_s) / accel_dt_s;
        raw_acceleration = Clamp(raw_acceleration,
          -APP_BALL_ACCEL_LIMIT_MM_S2, APP_BALL_ACCEL_LIMIT_MM_S2);
        ball_acceleration_mm_s2 += APP_BALL_ACCEL_FILTER_ALPHA *
          (raw_acceleration - ball_acceleration_mm_s2);
      } else {
        ball_acceleration_mm_s2 = 0.0f;
      }
      ball_previous_speed_mm_s = vision->speed_mm_s;
      ball_accel_reference_ms = measurement_ms;
    }
  }
  status.ball_acceleration_mm_s2 = ball_acceleration_mm_s2;
  status.ball_acceleration_steps_s = ball_sign * -ball_ka *
    ball_acceleration_mm_s2;

  /*
   * An endpoint needs a deliberate release phase.  Position error alone
   * cannot reveal how far the tube is tilted while the ball is held by the
   * end contact.  Drive to the bounded relative pose limit, but leave this
   * phase at the first visually confirmed inward motion so normal derivative
   * braking starts before the ball acquires excessive speed.
   */
  if (ball_endpoint_release_latched &&
      (fabsf(vision->position_mm) <= APP_BALL_ENDPOINT_EXIT_ABS_MM) &&
      (fabsf(vision->speed_mm_s) <=
       APP_BALL_ENDPOINT_CATCH_EXIT_SPEED_MM_S)) {
    ball_endpoint_release_latched = false;
  }
  if (ball_endpoint_release_latched &&
      (fabsf(vision->position_mm) >= APP_BALL_ENDPOINT_ENTER_ABS_MM) &&
      (fabsf(vision->speed_mm_s) <=
       APP_BALL_ENDPOINT_CATCH_EXIT_SPEED_MM_S)) {
    ball_endpoint_release_latched = false;
  }
  if (!ball_endpoint_release_active && !ball_endpoint_release_latched &&
      (fabsf(vision->speed_mm_s) <=
       APP_BALL_ENDPOINT_CATCH_EXIT_SPEED_MM_S) &&
      (((vision->position_mm >= APP_BALL_ENDPOINT_ENTER_ABS_MM) &&
        (error < -APP_BALL_PID_HOLD_ERROR_MM)) ||
       ((vision->position_mm <= -APP_BALL_ENDPOINT_ENTER_ABS_MM) &&
        (error > APP_BALL_PID_HOLD_ERROR_MM)))) {
    ball_endpoint_release_active = true;
    ball_endpoint_command_direction =
      (ball_sign * error >= 0.0f) ? 1.0f : -1.0f;
    ball_integral_error_s = 0.0f;
    ball_integral_stationary_since_ms = 0U;
    ball_endpoint_release_confirm_frames = 0U;
  }
  if (ball_endpoint_release_active) {
    float inward_speed_mm_s = -ball_endpoint_command_direction *
      vision->speed_mm_s;
    /* Velocity is the earliest reliable proof that the ball has left the
       contact.  Waiting for net travel from the entry point is wrong when
       residual outward motion first carries the ball farther into the end:
       it keeps adding tube angle after the ball has already reversed.  PD
       continues to command the same inward tilt until braking is actually
       needed, so handing off on confirmed inward speed is safe and faster. */
    if (inward_speed_mm_s >= APP_BALL_ENDPOINT_RELEASE_SPEED_MM_S) {
      if (ball_endpoint_release_confirm_frames <
          APP_BALL_ENDPOINT_RELEASE_CONFIRM_FRAMES) {
        ball_endpoint_release_confirm_frames++;
      }
    } else if (inward_speed_mm_s <
               (0.5f * APP_BALL_ENDPOINT_RELEASE_SPEED_MM_S)) {
      ball_endpoint_release_confirm_frames = 0U;
    }
    if (ball_endpoint_release_confirm_frames >=
        APP_BALL_ENDPOINT_RELEASE_CONFIRM_FRAMES) {
      ball_endpoint_release_active = false;
      ball_endpoint_release_latched = true;
      endpoint_handoff = true;
      ball_endpoint_release_confirm_frames = 0U;
      ball_integral_error_s = 0.0f;
      ball_integral_stationary_since_ms = 0U;
    } else {
      ball_last_measurement_ms = measurement_ms;
      ball_integral_stationary_since_ms = 0U;
      status.ball_integral_steps_s = 0.0f;
      return ball_pose_bias_steps + ball_endpoint_command_direction *
        APP_BALL_ENDPOINT_POSITION_LIMIT_STEPS;
    }
  }

  if ((fabsf(error) <= APP_BALL_PID_HOLD_ERROR_MM) &&
      (fabsf(vision->speed_mm_s) <= APP_BALL_PID_HOLD_SPEED_MM_S) &&
      ((ball_ka <= 0.0f) ||
       (fabsf(ball_acceleration_mm_s2) <= APP_BALL_PID_HOLD_ACCEL_MM_S2))) {
    ball_integral_error_s = 0.0f;
    ball_integral_stationary_since_ms = 0U;
    ball_static_release_reference_valid = false;
    ball_static_release_catch_active = false;
    ball_pose_bias_steps = ball_command_position_steps;
    ball_last_measurement_ms = measurement_ms;
    status.ball_integral_steps_s = 0.0f;
    return ball_command_position_steps;
  }

  pd = ball_kp * error - derivative_gain * vision->speed_mm_s -
       ball_ka * ball_acceleration_mm_s2;
  relative_pd_steps = Clamp(ball_sign * pd,
                            -APP_BALL_PID_POSITION_LIMIT_STEPS,
                            APP_BALL_PID_POSITION_LIMIT_STEPS);
  /* Make the endpoint-release -> visual-PD transition bumpless.  The tube's
   * level pose is not known a priori, so retain the actual commanded pose and
   * infer the bias that makes the first PD target exactly continuous. */
  if (endpoint_handoff) {
    ball_pose_bias_steps = ball_command_position_steps - relative_pd_steps;
  }
  /* Integral action learns the unknown physical level pose directly.  It is
   * deliberately active only while the ball is nearly stationary; once the
   * ball moves, the learned bias freezes and PD alone performs braking.  This
   * avoids both integral re-launch and the deadlock where a relative PD pose
   * limit prevented a static ball from ever reaching the true level pose. */
  if (fabsf(vision->speed_mm_s) <=
      APP_BALL_INTEGRAL_ACTIVE_SPEED_MM_S) {
    if (ball_integral_stationary_since_ms == 0U) {
      ball_integral_stationary_since_ms = measurement_ms;
      if ((status.ball_sequence_stage == CONTROL_BALL_SEQUENCE_OFF) &&
          !ball_static_release_reference_valid) {
        ball_static_release_reference_mm = vision->position_mm;
        ball_static_release_reference_valid = true;
      }
    } else if ((ball_last_measurement_ms != 0U) &&
               ((measurement_ms - ball_integral_stationary_since_ms) >=
                APP_BALL_INTEGRAL_ARM_TIME_MS)) {
      dt_s = Clamp(dt_s, 0.005f, 0.100f);
      if (ball_ki > 0.0f) {
        float integral_limit =
          APP_BALL_INTEGRAL_POSITION_LIMIT_STEPS / ball_ki;
        float candidate = Clamp(ball_integral_error_s + error * dt_s,
                                -integral_limit, integral_limit);
        float bias_delta = ball_sign * ball_ki *
          (candidate - ball_integral_error_s);
        float maximum_bias_delta =
          APP_BALL_INTEGRAL_BIAS_RATE_STEPS_S * dt_s;
        bias_delta = Clamp(bias_delta,
                           -maximum_bias_delta, maximum_bias_delta);
        ball_pose_bias_steps += bias_delta;
        ball_integral_error_s += bias_delta / (ball_sign * ball_ki);
      }
    }
  } else {
    ball_integral_stationary_since_ms = 0U;
    if (fabsf(vision->speed_mm_s) >=
        APP_BALL_INTEGRAL_RESET_SPEED_MM_S) {
      if (status.ball_sequence_stage != CONTROL_BALL_SEQUENCE_OFF) {
        ball_integral_error_s = 0.0f;
      } else if (ball_static_release_catch_enabled &&
                 ball_static_release_reference_valid &&
                 (fabsf(vision->position_mm -
                         ball_static_release_reference_mm) >=
                  APP_BALL_STATIC_RELEASE_TRAVEL_MM)) {
        ball_integral_error_s = 0.0f;
        ball_static_release_reference_valid = false;
        ball_static_release_catch_active = true;
      }
    }
  }
  if (ball_static_release_catch_active &&
      (fabsf(vision->speed_mm_s) <= APP_BALL_PID_HOLD_SPEED_MM_S)) {
    ball_static_release_catch_active = false;
  }
  ball_last_measurement_ms = measurement_ms;
  status.ball_integral_steps_s = ball_sign * ball_ki * ball_integral_error_s;
  target_position_steps = ball_pose_bias_steps + relative_pd_steps;
  return target_position_steps;
}

void ControlLoops_Init(void)
{
  PID_Init(&wheel_left_pid, APP_WHEEL_KP, APP_WHEEL_KI, 0.0f,
           APP_WHEEL_PWM_LIMIT / APP_WHEEL_KI, APP_WHEEL_PWM_LIMIT);
  PID_Init(&wheel_right_pid, APP_WHEEL_KP, APP_WHEEL_KI, 0.0f,
           APP_WHEEL_PWM_LIMIT / APP_WHEEL_KI, APP_WHEEL_PWM_LIMIT);
  ball_kp = APP_BALL_KP;
  ball_ki = APP_BALL_KI;
  ball_kd = APP_BALL_KD;
  ball_ka = APP_BALL_KA;
  ball_sign = APP_BALL_CONTROL_SIGN;
  stepper_rate_limit = APP_BALL_RATE_LIMIT_STEPS_S;
  zero_kx = APP_ZERO_KX_STEPS_S_PER_MM;
  zero_kv = APP_ZERO_KV_STEPS_PER_MM;
  zero_ki = APP_ZERO_KI_STEPS_S_PER_MM_S;
  zero_kq = APP_ZERO_KQ_PER_S;
  zero_ka = APP_ZERO_KA_STEPS_PER_MM_S2;
  zero_rate_limit = APP_ZERO_RATE_LIMIT_STEPS_S;
  zero_pose_limit = APP_ZERO_POSE_LIMIT_STEPS;
  ControlLoops_Reset();
}

void ControlLoops_Reset(void)
{
  PID_Reset(&wheel_left_pid);
  PID_Reset(&wheel_right_pid);
  requested_base_speed = 0.0f;
  line_previous_error = 0.0f;
  line_initialized = false;
  manual_stepper_rate_steps_s = 0;
  manual_stepper_until_ms = 0U;
  status.left_target_mm_s = 0.0f;
  status.right_target_mm_s = 0.0f;
  status.left_ramped_mm_s = 0.0f;
  status.right_ramped_mm_s = 0.0f;
  status.ball_target_mm = 0.0f;
  status.line_steer_mm_s = 0.0f;
  status.left_pwm = 0;
  status.right_pwm = 0;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_OFF;
  status.chassis_enabled = false;
  status.actuator_enabled = false;
  status.ball_enabled = false;
  ball_command_position_steps = 0.0f;
  ball_pose_bias_steps = 0.0f;
  ball_static_release_catch_enabled = true;
  ResetBallPid();
  ResetMovingZero();
  Motor_EnableChassis(false);
  Stepper_Enable(false);
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

void ControlLoops_EnableActuator(bool enable)
{
  if (status.actuator_enabled == enable) return;
  status.actuator_enabled = enable;
  Stepper_Enable(enable);
  if (!enable) {
    manual_stepper_rate_steps_s = 0;
    manual_stepper_until_ms = 0U;
    ResetBallPid();
    ResetMovingZero();
  }
}

void ControlLoops_EnableBall(bool enable)
{
  status.ball_enabled = enable;
  if (!enable) {
    status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_OFF;
    ResetBallPid();
    ResetMovingZero();
    Stepper_Stop();
  }
}

void ControlLoops_StartBallSequence(uint32_t now_ms)
{
  (void)now_ms;
  /* Requirement 3 always starts with the ball held at O.  The actuator has no
   * absolute angle reference, so the pose that is actually holding O is the
   * only repeatable level-pose observation.  Re-anchor the learned bias here;
   * otherwise an integral bias left by the preceding recovery run makes two
   * identical sequence starts command different tube angles. */
  ball_pose_bias_steps = ball_command_position_steps;
  ResetBallPid();
  status.ball_target_mm = APP_BALL_SEQUENCE_GOAL_MM;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_POSITIVE_CONTROL;
}

void ControlLoops_StartBallSequenceNegative(uint32_t now_ms)
{
  (void)now_ms;
  ResetBallPid();
  status.ball_target_mm = -APP_BALL_SEQUENCE_GOAL_MM;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_NEGATIVE_CONTROL;
}

void ControlLoops_FinishBallSequenceHold(void)
{
  status.ball_target_mm = -APP_BALL_SEQUENCE_GOAL_MM;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_OFF;
}

void ControlLoops_SetBaseSpeed(float speed_mm_s)
{
  requested_base_speed = speed_mm_s;
}

void ControlLoops_SetBallTarget(float position_mm)
{
  status.ball_target_mm = Clamp(position_mm, -APP_BALL_TARGET_LIMIT_MM,
                                APP_BALL_TARGET_LIMIT_MM);
  if (!status.ball_enabled) {
    ball_pose_bias_steps = ball_command_position_steps;
  }
  ResetBallPid();
}

void ControlLoops_SetBallGains(float kp, float ki, float kd, float sign)
{
  if ((kp < 0.0f) || (ki < 0.0f) || (kd < 0.0f)) return;
  ball_kp = kp;
  ball_ki = ki;
  ball_kd = kd;
  ball_sign = (sign < 0.0f) ? -1.0f : 1.0f;
  ResetBallPid();
}

void ControlLoops_EnableStaticReleaseCatch(bool enable)
{
  ball_static_release_catch_enabled = enable;
  if (!enable) ball_static_release_catch_active = false;
}

void ControlLoops_SetBallAccelerationGain(float ka)
{
  if (ka < 0.0f) return;
  ball_ka = ka;
  ResetBallPid();
}

void ControlLoops_SetStepperRateLimit(float limit_steps_s)
{
  stepper_rate_limit = Clamp(limit_steps_s, 0.0f,
                             (float)APP_STEPPER_MAX_RATE_STEPS_S);
}

void ControlLoops_StartMovingZero(void)
{
  ResetBallPid();
  ResetMovingZero();
  moving_zero_enabled = true;
  status.ball_target_mm = 0.0f;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_OFF;
  status.ball_enabled = true;
}

void ControlLoops_SetMovingZeroGains(float kx, float kv, float ki,
                                     float kq, float ka)
{
  if ((kx < 0.0f) || (kv < 0.0f) || (ki < 0.0f) ||
      (kq < 0.0f) || !isfinite(ka)) return;
  zero_kx = kx;
  zero_kv = kv;
  zero_ki = ki;
  zero_kq = kq;
  zero_ka = ka;
  zero_integral_mm_s = 0.0f;
}

void ControlLoops_SetMovingZeroLimits(float rate_steps_s, float pose_steps)
{
  zero_rate_limit = Clamp(rate_steps_s, 0.0f,
                          (float)APP_STEPPER_MAX_RATE_STEPS_S);
  zero_pose_limit = Clamp(pose_steps, 0.0f,
                          APP_BALL_PID_POSITION_LIMIT_STEPS);
}

bool ControlLoops_IsMovingZeroActive(void)
{
  return moving_zero_enabled;
}

void ControlLoops_ManualStepperRun(int32_t rate_steps_s, uint32_t duration_ms,
                                   uint32_t now_ms)
{
  if (duration_ms > APP_BENCH_RUN_MAX_MS) duration_ms = APP_BENCH_RUN_MAX_MS;
  if (rate_steps_s > APP_STEPPER_MAX_RATE_STEPS_S) {
    rate_steps_s = APP_STEPPER_MAX_RATE_STEPS_S;
  }
  if (rate_steps_s < -APP_STEPPER_MAX_RATE_STEPS_S) {
    rate_steps_s = -APP_STEPPER_MAX_RATE_STEPS_S;
  }
  status.ball_enabled = false;
  status.ball_sequence_stage = CONTROL_BALL_SEQUENCE_OFF;
  ResetBallPid();
  ResetMovingZero();
  manual_stepper_rate_steps_s = rate_steps_s;
  manual_stepper_until_ms = now_ms + duration_ms;
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
  status.line_steer_mm_s = Clamp(APP_LINE_KP * line->error +
                                 APP_LINE_KD * derivative,
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
  uint32_t now_ms = HAL_GetTick();
  float dt_s;
  float target_rate = 0.0f;
  bool force_immediate_stop = false;

  if (dt_ms == 0U) return;
  dt_s = (float)dt_ms * 0.001f;
  if (status.chassis_enabled && (left != 0) && (right != 0)) {
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

  {
    const StepperStatus *stepper = Stepper_GetStatus();
    ball_command_position_steps +=
      (float)stepper->applied_rate_steps_s * dt_s;
    if (moving_zero_enabled) {
      zero_pose_steps += (float)stepper->applied_rate_steps_s * dt_s;
      zero_pose_steps = Clamp(zero_pose_steps,
        -zero_pose_limit - 5.0f, zero_pose_limit + 5.0f);
      status.ball_command_position_steps = zero_pose_steps;
    } else {
      status.ball_command_position_steps = ball_command_position_steps;
    }
  }

  if (status.ball_enabled && moving_zero_enabled) {
    VisionBallSample vision = {0};
    bool new_measurement = VisionUART_ConsumeNewFrame(&vision) && vision.valid;
    bool valid = false;
    target_rate = MovingZeroUpdate(&vision, new_measurement, now_ms, dt_s,
                                   &valid);
    if (!valid) {
      target_rate = 0.0f;
      force_immediate_stop = true;
    }
  } else if (status.ball_enabled) {
    VisionBallSample vision = {0};
    if (VisionUART_ConsumeNewFrame(&vision) && vision.valid) {
      ball_target_position_steps = BallVisualPid(&vision);
      status.ball_target_position_steps = ball_target_position_steps;
    }
    {
      int32_t vision_age_ms;
      float position_error_steps;
      (void)VisionUART_GetSnapshot(&vision);
      vision_age_ms = (int32_t)(now_ms - vision.timestamp_ms);
      if (!vision.valid ||
          ((vision_age_ms >= 0) &&
           ((uint32_t)vision_age_ms > APP_VISION_CONTROL_HOLD_MS))) {
        ball_requested_rate_steps_s = 0.0f;
        ball_integral_error_s = 0.0f;
        ball_integral_stationary_since_ms = 0U;
        ball_last_measurement_ms = 0U;
        ball_target_position_steps = ball_command_position_steps;
        status.ball_target_position_steps = ball_target_position_steps;
        target_rate = 0.0f;
        force_immediate_stop = true;
      } else {
        float active_rate_limit = stepper_rate_limit;
        if (ball_endpoint_release_active) {
          active_rate_limit = APP_BALL_ENDPOINT_RATE_LIMIT_STEPS_S;
        } else if (ball_static_release_catch_active) {
          active_rate_limit = APP_BALL_ENDPOINT_CATCH_RATE_STEPS_S;
        }
        position_error_steps = ball_target_position_steps -
          ball_command_position_steps;
        if (fabsf(position_error_steps) <=
            APP_BALL_POSITION_ERROR_STOP_STEPS) {
          ball_requested_rate_steps_s = 0.0f;
        } else {
          ball_requested_rate_steps_s = Clamp(
            APP_BALL_POSITION_RATE_KP * position_error_steps,
            -active_rate_limit, active_rate_limit);
        }
        status.ball_pid_output_steps_s = ball_requested_rate_steps_s;
        target_rate = ball_requested_rate_steps_s;
      }
    }
  } else if ((manual_stepper_until_ms != 0U) &&
             ((int32_t)(now_ms - manual_stepper_until_ms) < 0)) {
    target_rate = (float)manual_stepper_rate_steps_s;
  } else {
    if (manual_stepper_until_ms != 0U) force_immediate_stop = true;
    manual_stepper_until_ms = 0U;
    manual_stepper_rate_steps_s = 0;
    target_rate = 0.0f;
  }

  if (!status.actuator_enabled) {
    target_rate = 0.0f;
    force_immediate_stop = true;
  }
  {
    const EncoderSample *actuator = Encoder_Get(ENCODER_ACTUATOR);
    if (((actuator->total_count <= APP_ACTUATOR_RIGHT_GUARD_COUNT) &&
         (target_rate < 0.0f)) ||
        ((actuator->total_count >= APP_ACTUATOR_LEFT_GUARD_COUNT) &&
         (target_rate > 0.0f))) {
      target_rate = 0.0f;
      force_immediate_stop = true;
    }
  }
  status.stepper_target_rate_steps_s = (int32_t)lroundf(target_rate);
  if (force_immediate_stop) {
    stepper_ramped_rate_steps_s = 0.0f;
    status.stepper_rate_steps_s = 0;
    Stepper_Stop();
  } else {
    stepper_ramped_rate_steps_s = Ramp(stepper_ramped_rate_steps_s,
      target_rate, APP_STEPPER_ACCEL_STEPS_S2 * dt_s);
    status.stepper_rate_steps_s = (int32_t)lroundf(stepper_ramped_rate_steps_s);
    Stepper_SetRate(status.stepper_rate_steps_s, now_ms);
  }
}

const ControlStatus *ControlLoops_GetStatus(void)
{
  return &status;
}
