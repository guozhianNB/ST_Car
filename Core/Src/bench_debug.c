#include "bench_debug.h"
#include "app.h"
#include "app_config.h"
#include "control_loops.h"
#include "encoder.h"
#include "motor.h"
#include "oled_display.h"
#include "stepper.h"
#include "usart.h"
#include "vision_uart.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if APP_ENABLE_BENCH_DEBUG

#define BENCH_RX_RING_SIZE 128U
#define BENCH_LINE_SIZE     96U
#define BENCH_TX_SIZE      512U

static BenchDebugStatus bench;
static volatile uint8_t rx_byte;
static volatile uint8_t rx_ring[BENCH_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_restart_requested;
static volatile bool emergency_stop_requested;
static char command_line[BENCH_LINE_SIZE];
static uint16_t command_length;
static char tx_active[BENCH_TX_SIZE];
static char tx_queued[BENCH_TX_SIZE];
static uint16_t tx_queued_length;
static volatile bool tx_inflight;
static bool tx_queued_valid;
static uint32_t fast_tick_ms;
static uint32_t telemetry_tick_ms;
static uint32_t run_end_ms;
static uint32_t closed_loop_end_ms;
static uint32_t stall_since_ms;
static uint32_t stall_last_drive_ms;
static uint32_t sequence_start_ms;
static uint32_t sequence_settle_start_ms;
static uint32_t sequence_settle_last_measurement_ms;
static uint32_t sequence_settle_accumulated_ms;
static bool sequence_positive_reached;
static int32_t step_command_impulse_step_ms;
static uint32_t step_positive_drive_ms;
static uint32_t step_negative_drive_ms;

static void ResetActuatorDiagnostics(void)
{
  step_command_impulse_step_ms = 0;
  step_positive_drive_ms = 0U;
  step_negative_drive_ms = 0U;
}

static void UpdateActuatorDiagnostics(uint32_t dt_ms)
{
  int32_t applied = Stepper_GetStatus()->applied_rate_steps_s;
  int64_t next_impulse = (int64_t)step_command_impulse_step_ms +
    (int64_t)applied * (int64_t)dt_ms;
  if (next_impulse > INT32_MAX) next_impulse = INT32_MAX;
  if (next_impulse < INT32_MIN) next_impulse = INT32_MIN;
  step_command_impulse_step_ms = (int32_t)next_impulse;
  if (applied > 0) {
    step_positive_drive_ms += dt_ms;
  } else if (applied < 0) {
    step_negative_drive_ms += dt_ms;
  }
}

static const char *FaultName(BenchFault fault)
{
  static const char *const names[] = {
    "none", "estop", "stall", "vision_timeout", "sequence_timeout"
  };
  return ((unsigned)fault < (sizeof(names) / sizeof(names[0])))
    ? names[fault] : "unknown";
}

static void StartReceive(void)
{
  if (HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&rx_byte, 1U) != HAL_OK) {
    rx_restart_requested = true;
  }
}

static void StartQueuedTransmit(void)
{
  if (tx_inflight || !tx_queued_valid) return;
  memcpy(tx_active, tx_queued, tx_queued_length);
  tx_inflight = true;
  tx_queued_valid = false;
  if (HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)tx_active,
                           tx_queued_length) != HAL_OK) {
    tx_inflight = false;
  }
}

static void QueueMessage(const char *format, ...)
{
  va_list args;
  int length;
  va_start(args, format);
  length = vsnprintf(tx_queued, sizeof(tx_queued), format, args);
  va_end(args);
  if (length <= 0) return;
  if (length >= (int)sizeof(tx_queued)) length = (int)sizeof(tx_queued) - 1;
  tx_queued_length = (uint16_t)length;
  tx_queued_valid = true;
  StartQueuedTransmit();
}

static void StopOutputs(void)
{
  ControlLoops_EnableBall(false);
  ControlLoops_EnableActuator(false);
  Motor_EmergencyStop();
  run_end_ms = 0U;
  closed_loop_end_ms = 0U;
  stall_since_ms = 0U;
  stall_last_drive_ms = 0U;
}

static void EnterFault(BenchFault fault)
{
  if (bench.mode == BENCH_MODE_FAULT) return;
  StopOutputs();
  bench.mode = BENCH_MODE_FAULT;
  bench.fault = fault;
  QueueMessage("FAULT %s\r\n", FaultName(fault));
}

static bool ParseFloat(const char *text, float *value)
{
  char *end = 0;
  float parsed = strtof(text, &end);
  if ((end == text) || (*end != '\0') || !isfinite(parsed)) return false;
  *value = parsed;
  return true;
}

static bool ParseLong(const char *text, long *value)
{
  char *end = 0;
  long parsed = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0')) return false;
  *value = parsed;
  return true;
}

static void PrintStatus(uint32_t now_ms)
{
  const EncoderSample *encoder = Encoder_Get(ENCODER_ACTUATOR);
  const ControlStatus *control = ControlLoops_GetStatus();
  const StepperStatus *stepper = Stepper_GetStatus();
  VisionBallSample ball = {0};
  bool current_ball = VisionUART_GetCurrentMeasurement(
    &ball, now_ms, APP_VISION_COAST_MS);
  uint32_t measurement_age = (ball.timestamp_ms == 0U)
    ? 0xFFFFFFFFU : now_ms - ball.timestamp_ms;
  uint32_t packet_age = (ball.packet_timestamp_ms == 0U)
    ? 0xFFFFFFFFU : now_ms - ball.packet_timestamp_ms;
  (void)VisionUART_GetSnapshot(&ball);
  QueueMessage(
    "STATUS mode=%u fault=%s enc=%ld denc=%ld sreq=%ld sapplied=%ld crate=%ld "
    "uimp=%ld upos=%lu uneg=%lu ball=%.2f "
    "vball=%.2f aball=%.1f bref=%.2f pid=%.1f pref=%.1f ppos=%.1f "
    "apid=%.1f integ=%.1f "
    "vision=%u current=%u "
    "vframe=%lu vs=%u vage=%lu vpage=%lu vdt=%lu vmax=%lu "
    "vacc=%lu vrej=%lu seq=%u elapsed=%lu oled=%02X\r\n",
    (unsigned)bench.mode, FaultName(bench.fault),
    (long)encoder->total_count, (long)encoder->delta_count,
    (long)stepper->requested_rate_steps_s,
    (long)stepper->applied_rate_steps_s,
    (long)control->stepper_rate_steps_s,
    (long)step_command_impulse_step_ms,
    (unsigned long)step_positive_drive_ms,
    (unsigned long)step_negative_drive_ms,
    ball.position_mm, ball.speed_mm_s,
    control->ball_acceleration_mm_s2, control->ball_target_mm,
    control->ball_pid_output_steps_s,
    control->ball_target_position_steps,
    control->ball_command_position_steps,
    control->ball_acceleration_steps_s,
    control->ball_integral_steps_s, VisionUART_IsFresh(now_ms) ? 1U : 0U,
    current_ball ? 1U : 0U, (unsigned long)ball.frame_number,
    (unsigned)ball.status, (unsigned long)measurement_age,
    (unsigned long)packet_age,
    (unsigned long)ball.measurement_interval_ms,
    (unsigned long)ball.maximum_measurement_interval_ms,
    (unsigned long)ball.accepted_measurements,
    (unsigned long)ball.rejected_measurements,
    (unsigned)bench.sequence_stage,
    (unsigned long)bench.sequence_elapsed_ms,
    (unsigned)OledDisplay_GetAddress());
}

static bool CanStartBall(uint32_t now_ms, VisionBallSample *ball)
{
  if (!VisionUART_GetCurrentMeasurement(ball, now_ms,
                                         APP_VISION_COAST_MS)) {
    QueueMessage("ERR no fresh real vision measurement\r\n");
    return false;
  }
  return true;
}

static void StartBall(float target_mm, uint32_t now_ms, bool sequence)
{
  StopOutputs();
  ControlLoops_SetBallTarget(target_mm);
  ControlLoops_EnableActuator(true);
  ControlLoops_EnableBall(true);
  if (sequence) ControlLoops_StartBallSequence(now_ms);
  bench.ball_target_mm = target_mm;
  bench.fault = BENCH_FAULT_NONE;
  bench.mode = sequence ? BENCH_MODE_BALL_SEQUENCE : BENCH_MODE_BALL;
  bench.sequence_stage = sequence ? 1U : 0U;
  bench.sequence_elapsed_ms = 0U;
  sequence_start_ms = now_ms;
  sequence_settle_start_ms = 0U;
  sequence_settle_last_measurement_ms = 0U;
  sequence_settle_accumulated_ms = 0U;
  sequence_positive_reached = false;
  closed_loop_end_ms = now_ms + APP_BENCH_CLOSED_LOOP_MAX_MS;
}

static void ProcessCommand(char *line, uint32_t now_ms)
{
  char *token[8];
  unsigned count = 0U;
  char *part = strtok(line, " \t");
  float a, b, c, d;
  long integer_a, integer_b;
  while ((part != 0) && (count < (sizeof(token) / sizeof(token[0])))) {
    token[count++] = part;
    part = strtok(0, " \t");
  }
  if (count == 0U) return;

  if ((count == 2U) && !strcmp(token[0], "bench") &&
      !strcmp(token[1], "on")) {
    if (App_GetStatus()->state != APP_STATE_STANDBY) {
      QueueMessage("ERR normal app must be in standby\r\n");
      return;
    }
    StopOutputs();
    ResetActuatorDiagnostics();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK stepper visual-only bench active; command pose preserved\r\n");
  } else if ((count == 2U) && !strcmp(token[0], "bench") &&
             !strcmp(token[1], "off")) {
    StopOutputs();
    bench.mode = BENCH_MODE_OFF;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK bench off\r\n");
  } else if ((count == 2U) && !strcmp(token[0], "app") &&
             !strcmp(token[1], "req3")) {
    StopOutputs();
    bench.mode = BENCH_MODE_OFF;
    bench.fault = BENCH_FAULT_NONE;
    App_RequestRequirement3();
    QueueMessage("OK normal app requirement 3 requested\r\n");
  } else if (!strcmp(token[0], "status")) {
    PrintStatus(now_ms);
  } else if (!strcmp(token[0], "stop")) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK stopped\r\n");
  } else if (!strcmp(token[0], "clear")) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK fault cleared\r\n");
  } else if (!strcmp(token[0], "zero")) {
    if (bench.mode != BENCH_MODE_IDLE) {
      QueueMessage("ERR stop before zero\r\n");
      return;
    }
    Encoder_Reset(ENCODER_ACTUATOR);
    ResetActuatorDiagnostics();
    QueueMessage("OK actuator encoder debug count zeroed\r\n");
  } else if ((count == 2U) && !strcmp(token[0], "diag") &&
             !strcmp(token[1], "reset")) {
    ResetActuatorDiagnostics();
    QueueMessage("OK actuator diagnostics reset\r\n");
  } else if ((count == 3U) && !strcmp(token[0], "run") &&
             ParseLong(token[1], &integer_a) &&
             ParseLong(token[2], &integer_b)) {
    uint64_t requested_pulse_milli;
    if ((integer_a > APP_BENCH_OPEN_LOOP_RATE_LIMIT_STEPS_S) ||
        (integer_a < -APP_BENCH_OPEN_LOOP_RATE_LIMIT_STEPS_S) ||
        (integer_b <= 0) ||
        ((uint32_t)integer_b > APP_BENCH_RUN_MAX_MS)) {
      QueueMessage("ERR run limit +/- %d steps/s, 1..%lu ms, <=%lu pulses\r\n",
                   APP_BENCH_OPEN_LOOP_RATE_LIMIT_STEPS_S,
                   (unsigned long)APP_BENCH_RUN_MAX_MS,
                   (unsigned long)APP_BENCH_RUN_MAX_COMMAND_PULSES);
      return;
    }
    requested_pulse_milli =
      (uint64_t)((integer_a < 0) ? -integer_a : integer_a) *
      (uint64_t)integer_b;
    if (requested_pulse_milli >
        ((uint64_t)APP_BENCH_RUN_MAX_COMMAND_PULSES * 1000ULL)) {
      QueueMessage("ERR run pulse budget <=%lu; rate*ms must be <=%lu\r\n",
                   (unsigned long)APP_BENCH_RUN_MAX_COMMAND_PULSES,
                   (unsigned long)(APP_BENCH_RUN_MAX_COMMAND_PULSES * 1000U));
      return;
    }
    StopOutputs();
    bench.mode = BENCH_MODE_RUN;
    bench.fault = BENCH_FAULT_NONE;
    bench.run_rate_steps_s = (int32_t)integer_a;
    run_end_ms = now_ms + (uint32_t)integer_b;
    ControlLoops_EnableActuator(true);
    ControlLoops_ManualStepperRun(bench.run_rate_steps_s,
                                  (uint32_t)integer_b, now_ms);
    QueueMessage("OK run rate=%ld steps/s ms=%ld\r\n", integer_a, integer_b);
  } else if ((count == 2U) && !strcmp(token[0], "ball") &&
             !strcmp(token[1], "sequence")) {
    VisionBallSample ball = {0};
    if (!CanStartBall(now_ms, &ball)) return;
    if (fabsf(ball.position_mm) > APP_BALL_START_TOLERANCE_MM) {
      QueueMessage("ERR ball must start at O within +/-%.1f mm\r\n",
                   APP_BALL_START_TOLERANCE_MM);
      return;
    }
    StartBall(APP_BALL_SEQUENCE_GOAL_MM, now_ms, true);
    QueueMessage("OK visual step-rate sequence 0 -> +50 -> -50 started\r\n");
  } else if ((count == 2U) && !strcmp(token[0], "ball") &&
             ParseFloat(token[1], &a)) {
    VisionBallSample ball = {0};
    if (!CanStartBall(now_ms, &ball)) return;
    if (fabsf(a) > APP_BALL_TARGET_LIMIT_MM) {
      QueueMessage("ERR target limit +/-%.1f mm\r\n",
                   APP_BALL_TARGET_LIMIT_MM);
      return;
    }
    StartBall(a, now_ms, false);
    QueueMessage("OK visual step-rate ball target=%.2f mm\r\n", a);
  } else if ((count == 2U) && !strcmp(token[0], "zeroctl") &&
             !strcmp(token[1], "start")) {
    VisionBallSample ball = {0};
    if (!CanStartBall(now_ms, &ball)) return;
    StopOutputs();
    ControlLoops_EnableActuator(true);
    ControlLoops_StartMovingZero();
    bench.ball_target_mm = 0.0f;
    bench.fault = BENCH_FAULT_NONE;
    bench.mode = BENCH_MODE_BALL;
    closed_loop_end_ms = now_ms + APP_BENCH_CLOSED_LOOP_MAX_MS;
    QueueMessage("OK independent moving-zero controller started\r\n");
  } else if ((count == 7U) && !strcmp(token[0], "gain") &&
             !strcmp(token[1], "zero") &&
             ParseFloat(token[2], &a) && ParseFloat(token[3], &b) &&
             ParseFloat(token[4], &c) && ParseFloat(token[5], &d)) {
    float ka;
    if (!ParseFloat(token[6], &ka) || (a < 0.0f) || (b < 0.0f) ||
        (c < 0.0f) || (d < 0.0f)) {
      QueueMessage("ERR zero gains require kx kv ki kq >=0 and finite ka\r\n");
      return;
    }
    ControlLoops_SetMovingZeroGains(a, b, c, d, ka);
    QueueMessage("OK zero gains kx=%.3f kv=%.3f ki=%.3f kq=%.3f ka=%.3f\r\n",
                 a, b, c, d, ka);
  } else if ((count == 4U) && !strcmp(token[0], "limit") &&
             !strcmp(token[1], "zero") && ParseFloat(token[2], &a) &&
             ParseFloat(token[3], &b) && (a >= 0.0f) && (b >= 0.0f) &&
             (a <= APP_STEPPER_MAX_RATE_STEPS_S) &&
             (b <= APP_BALL_PID_POSITION_LIMIT_STEPS)) {
    ControlLoops_SetMovingZeroLimits(a, b);
    QueueMessage("OK zero limits rate=%.0f pose=%.0f\r\n", a, b);
  } else if ((count == 6U) && !strcmp(token[0], "gain") &&
             !strcmp(token[1], "ball") &&
             ParseFloat(token[2], &a) && ParseFloat(token[3], &b) &&
             ParseFloat(token[4], &c) && ParseFloat(token[5], &d) &&
             (a >= 0.0f) && (b >= 0.0f) && (c >= 0.0f) &&
             ((d == 1.0f) || (d == -1.0f))) {
    bench.ball_kp = a;
    bench.ball_ki = b;
    bench.ball_kd = c;
    bench.ball_sign = d;
    ControlLoops_SetBallGains(a, b, c, d);
    QueueMessage("OK ball gains kp=%.3f ki=%.3f kd=%.3f sign=%.0f\r\n",
                 a, b, c, d);
  } else if ((count == 3U) && !strcmp(token[0], "gain") &&
             !strcmp(token[1], "accel") && ParseFloat(token[2], &a) &&
             (a >= 0.0f)) {
    ControlLoops_SetBallAccelerationGain(a);
    QueueMessage("OK ball acceleration gain ka=%.3f\r\n", a);
  } else if ((count == 3U) && !strcmp(token[0], "limit") &&
             !strcmp(token[1], "rate") && ParseFloat(token[2], &a) &&
             (a >= 0.0f) && (a <= APP_STEPPER_MAX_RATE_STEPS_S)) {
    bench.rate_limit_steps_s = a;
    ControlLoops_SetStepperRateLimit(a);
    QueueMessage("OK ball rate limit=%.0f steps/s\r\n", a);
  } else if ((count == 2U) && !strcmp(token[0], "catch") &&
             (!strcmp(token[1], "on") || !strcmp(token[1], "off"))) {
    bool enable = !strcmp(token[1], "on");
    ControlLoops_EnableStaticReleaseCatch(enable);
    QueueMessage("OK static release catch %s\r\n",
                 enable ? "on" : "off");
  } else if ((count >= 2U) && !strcmp(token[0], "stream") &&
             !strcmp(token[1], "off")) {
    bench.stream_enabled = false;
    QueueMessage("OK stream off\r\n");
  } else if ((count >= 2U) && !strcmp(token[0], "stream") &&
             !strcmp(token[1], "on")) {
    long period = APP_BENCH_TELEMETRY_MS;
    if ((count == 3U) && (!ParseLong(token[2], &period) ||
                          (period < 20) || (period > 2000))) {
      QueueMessage("ERR stream period 20..2000 ms\r\n");
      return;
    }
    bench.stream_period_ms = (uint32_t)period;
    bench.stream_enabled = true;
    telemetry_tick_ms = now_ms;
    QueueMessage("OK stream on %ld ms\r\n", period);
  } else if (!strcmp(token[0], "help")) {
    QueueMessage("bench on|off; app req3; zero; diag reset; run <steps/s> <ms>; ball <mm>|sequence; zeroctl start; "
                 "gain ball <kp> <ki> <kd> <sign>; gain accel <ka>; "
                 "gain zero <kx> <kv> <ki> <kq> <ka>; limit zero <rate> <pose>; limit rate <steps/s>; catch on|off; "
                 "status; stream on [ms]|off; stop; clear\r\n");
  } else {
    QueueMessage("ERR unknown command; use help\r\n");
  }
}

static void ServiceRx(uint32_t now_ms)
{
  while (rx_tail != rx_head) {
    char ch = (char)rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % BENCH_RX_RING_SIZE);
    if ((ch == '\r') || (ch == '\n')) {
      if (command_length != 0U) {
        command_line[command_length] = '\0';
        ProcessCommand(command_line, now_ms);
        command_length = 0U;
      }
    } else if (command_length < (BENCH_LINE_SIZE - 1U)) {
      command_line[command_length++] = ch;
    } else {
      command_length = 0U;
      QueueMessage("ERR command too long\r\n");
    }
  }
}

static void CheckSafety(uint32_t now_ms, bool need_vision)
{
  const EncoderSample *encoder = Encoder_Get(ENCODER_ACTUATOR);
  const StepperStatus *stepper = Stepper_GetStatus();
  if (need_vision && !VisionUART_IsFresh(now_ms)) {
    EnterFault(BENCH_FAULT_VISION_TIMEOUT);
    return;
  }
  if ((abs(stepper->applied_rate_steps_s) >=
       APP_STEPPER_STALL_MIN_RATE_STEPS_S) &&
             (labs((long)encoder->delta_count) <=
              APP_STEPPER_STALL_DELTA_COUNT)) {
    stall_last_drive_ms = now_ms;
    if (stall_since_ms == 0U) stall_since_ms = now_ms;
    if ((now_ms - stall_since_ms) >= APP_STEPPER_STALL_TIMEOUT_MS) {
      EnterFault(BENCH_FAULT_STALL);
    }
  } else if (labs((long)encoder->delta_count) >
             APP_STEPPER_STALL_DELTA_COUNT) {
    stall_since_ms = 0U;
  } else if ((stall_last_drive_ms != 0U) &&
             ((now_ms - stall_last_drive_ms) >
              APP_STEPPER_STALL_RESET_MS)) {
    stall_since_ms = 0U;
  }
}

static void UpdateSequence(uint32_t now_ms)
{
  VisionBallSample ball = {0};
  bool current = VisionUART_GetCurrentMeasurement(
    &ball, now_ms, APP_VISION_COAST_MS);
  bench.sequence_elapsed_ms = now_ms - sequence_start_ms;
  if (bench.sequence_elapsed_ms > APP_STATIC_STAGE_TIMEOUT_MS) {
    EnterFault(BENCH_FAULT_SEQUENCE_TIMEOUT);
    return;
  }
  if (!sequence_positive_reached) {
    if (!current ||
        (fabsf(ball.position_mm - APP_BALL_SEQUENCE_GOAL_MM) >
         APP_BALL_SETTLE_ERROR_MM)) return;
    sequence_positive_reached = true;
    bench.sequence_stage = 2U;
    bench.ball_target_mm = -APP_BALL_SEQUENCE_GOAL_MM;
    ControlLoops_StartBallSequenceNegative(now_ms);
    return;
  }
  if (!current ||
      (fabsf(ball.position_mm + APP_BALL_SEQUENCE_GOAL_MM) >
       APP_BALL_SETTLE_ERROR_MM) ||
      (fabsf(ball.speed_mm_s) > APP_BALL_SETTLE_SPEED_MM_S)) {
    sequence_settle_start_ms = 0U;
    sequence_settle_last_measurement_ms = 0U;
    sequence_settle_accumulated_ms = 0U;
    return;
  }
  if (ball.timestamp_ms != sequence_settle_last_measurement_ms) {
    if (sequence_settle_last_measurement_ms == 0U) {
      sequence_settle_start_ms = ball.timestamp_ms;
    } else {
      uint32_t interval_ms =
        ball.timestamp_ms - sequence_settle_last_measurement_ms;
      if (interval_ms > APP_VISION_CONTROL_HOLD_MS) {
        sequence_settle_start_ms = ball.timestamp_ms;
        sequence_settle_accumulated_ms = 0U;
      } else {
        sequence_settle_accumulated_ms += interval_ms;
      }
    }
    sequence_settle_last_measurement_ms = ball.timestamp_ms;
  }
  if (sequence_settle_accumulated_ms < APP_BALL_SETTLE_TIME_MS) return;
  bench.sequence_stage = 3U;
  bench.mode = BENCH_MODE_BALL;
  bench.ball_target_mm = -APP_BALL_SEQUENCE_GOAL_MM;
  ControlLoops_FinishBallSequenceHold();
  closed_loop_end_ms = now_ms + APP_BENCH_CLOSED_LOOP_MAX_MS;
  QueueMessage("OK ball sequence complete elapsed=%lu ms; holding -50 mm\r\n",
               (unsigned long)bench.sequence_elapsed_ms);
}

void BenchDebug_Init(uint32_t now_ms)
{
  memset(&bench, 0, sizeof(bench));
  bench.mode = BENCH_MODE_OFF;
  bench.ball_kp = APP_BALL_KP;
  bench.ball_ki = APP_BALL_KI;
  bench.ball_kd = APP_BALL_KD;
  bench.ball_sign = APP_BALL_CONTROL_SIGN;
  bench.rate_limit_steps_s = APP_BALL_RATE_LIMIT_STEPS_S;
  bench.stream_period_ms = APP_BENCH_TELEMETRY_MS;
  ResetActuatorDiagnostics();
  fast_tick_ms = now_ms;
  telemetry_tick_ms = now_ms;
  StartReceive();
}

bool BenchDebug_Run(uint32_t now_ms)
{
  if (rx_restart_requested &&
      (hlpuart1.RxState == HAL_UART_STATE_READY)) {
    rx_restart_requested = false;
    StartReceive();
  }
  StartQueuedTransmit();
  ServiceRx(now_ms);
  if (bench.mode == BENCH_MODE_OFF) return false;

  if (emergency_stop_requested) {
    emergency_stop_requested = false;
    EnterFault(BENCH_FAULT_ESTOP);
  }
  if ((now_ms - fast_tick_ms) >= APP_CONTROL_FAST_MS) {
    uint32_t elapsed = now_ms - fast_tick_ms;
    fast_tick_ms = now_ms;
    Encoder_Update(elapsed);
    UpdateActuatorDiagnostics(elapsed);
    if ((bench.mode == BENCH_MODE_BALL) ||
        (bench.mode == BENCH_MODE_BALL_SEQUENCE) ||
        (bench.mode == BENCH_MODE_RUN)) {
      ControlLoops_FastUpdate(elapsed);
    }
  }
  if ((bench.mode == BENCH_MODE_RUN) &&
      ((int32_t)(now_ms - run_end_ms) >= 0)) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    QueueMessage("OK run complete\r\n");
  }
  if ((bench.mode == BENCH_MODE_BALL) ||
      (bench.mode == BENCH_MODE_BALL_SEQUENCE) ||
      (bench.mode == BENCH_MODE_RUN)) {
    CheckSafety(now_ms, bench.mode != BENCH_MODE_RUN);
  }
  if (bench.mode == BENCH_MODE_BALL_SEQUENCE) UpdateSequence(now_ms);
  if ((bench.mode == BENCH_MODE_BALL) && (closed_loop_end_ms != 0U) &&
      ((int32_t)(now_ms - closed_loop_end_ms) >= 0)) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    QueueMessage("OK ball hold auto-stop\r\n");
  }
  if (bench.stream_enabled &&
      ((now_ms - telemetry_tick_ms) >= bench.stream_period_ms)) {
    telemetry_tick_ms = now_ms;
    PrintStatus(now_ms);
  }
  return true;
}

bool BenchDebug_IsActive(void)
{
  return bench.mode != BENCH_MODE_OFF;
}

void BenchDebug_RequestEmergencyStop(void)
{
  emergency_stop_requested = true;
}

void BenchDebug_UartRxCpltCallback(void)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % BENCH_RX_RING_SIZE);
  if (next != rx_tail) {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  StartReceive();
}

void BenchDebug_UartTxCpltCallback(void)
{
  tx_inflight = false;
  StartQueuedTransmit();
}

void BenchDebug_UartErrorCallback(void)
{
  rx_restart_requested = true;
  tx_inflight = false;
}

const BenchDebugStatus *BenchDebug_GetStatus(void)
{
  return &bench;
}

#else

void BenchDebug_Init(uint32_t now_ms) { (void)now_ms; }
bool BenchDebug_Run(uint32_t now_ms) { (void)now_ms; return false; }
bool BenchDebug_IsActive(void) { return false; }
void BenchDebug_RequestEmergencyStop(void) {}
void BenchDebug_UartRxCpltCallback(void) {}
void BenchDebug_UartTxCpltCallback(void) {}
void BenchDebug_UartErrorCallback(void) {}
const BenchDebugStatus *BenchDebug_GetStatus(void)
{
  static const BenchDebugStatus disabled = {0};
  return &disabled;
}

#endif
