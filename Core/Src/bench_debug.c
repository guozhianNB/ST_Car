#include "bench_debug.h"
#include "app.h"
#include "app_config.h"
#include "control_loops.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "sa100.h"
#include "usart.h"
#include "vision_uart.h"
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if APP_ENABLE_BENCH_DEBUG

#define BENCH_RX_RING_SIZE 128U
#define BENCH_LINE_SIZE 96U
#define BENCH_TX_SIZE 512U

static BenchDebugStatus bench;
static PidController angle_pid;
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
static uint32_t pulse_end_ms;
static uint32_t closed_loop_end_ms;
static uint32_t last_angle_sample_ms;
static uint32_t stall_since_ms;

static float Clamp(float value, float low, float high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static const char *ModeName(BenchMode mode)
{
  static const char *const names[] = {
    "off", "idle", "pulse", "angle", "ball", "fault"
  };
  return ((unsigned)mode < (sizeof(names) / sizeof(names[0]))) ? names[mode] : "?";
}

static const char *FaultName(BenchFault fault)
{
  static const char *const names[] = {
    "none", "estop", "sa100_timeout", "angle_limit", "encoder_limit",
    "stall", "vision_timeout"
  };
  return ((unsigned)fault < (sizeof(names) / sizeof(names[0]))) ? names[fault] : "?";
}

static void QueueMessage(const char *format, ...)
{
  va_list args;
  int length;
  if (tx_queued_valid) return;
  va_start(args, format);
  length = vsnprintf(tx_queued, sizeof(tx_queued), format, args);
  va_end(args);
  if (length <= 0) return;
  if (length >= (int)sizeof(tx_queued)) length = (int)sizeof(tx_queued) - 1;
  tx_queued_length = (uint16_t)length;
  tx_queued_valid = true;
}

static void ServiceTx(void)
{
  if (!tx_queued_valid || tx_inflight ||
      (hlpuart1.gState != HAL_UART_STATE_READY)) return;
  memcpy(tx_active, tx_queued, tx_queued_length);
  if (HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)tx_active,
                          tx_queued_length) == HAL_OK) {
    tx_queued_valid = false;
    tx_inflight = true;
  }
}

static void StartReceive(void)
{
  if (HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&rx_byte, 1U) != HAL_OK) {
    rx_restart_requested = true;
  }
}

static void StopOutputs(void)
{
  Motor_EmergencyStop();
  ControlLoops_Reset();
  PID_Reset(&angle_pid);
  bench.pulse_pwm = 0;
  pulse_end_ms = 0U;
  closed_loop_end_ms = 0U;
  last_angle_sample_ms = 0U;
  stall_since_ms = 0U;
}

static void EnterFault(BenchFault fault)
{
  StopOutputs();
  bench.mode = BENCH_MODE_FAULT;
  bench.fault = fault;
  QueueMessage("ERR bench fault=%s; use 'stop' then inspect hardware\r\n",
               FaultName(fault));
}

static bool ParseFloatValue(const char *text, float *value)
{
  char *end;
  float parsed;
  if ((text == 0) || (value == 0)) return false;
  parsed = strtof(text, &end);
  if ((end == text) || (*end != '\0') || !isfinite(parsed)) return false;
  *value = parsed;
  return true;
}

static bool ParseLongValue(const char *text, long *value)
{
  char *end;
  long parsed;
  if ((text == 0) || (value == 0)) return false;
  parsed = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0')) return false;
  *value = parsed;
  return true;
}

static unsigned Tokenize(char *line, char **tokens, unsigned maximum)
{
  unsigned count = 0U;
  char *token = strtok(line, " \t");
  while ((token != 0) && (count < maximum)) {
    char *p;
    for (p = token; *p != '\0'; ++p) *p = (char)tolower((unsigned char)*p);
    tokens[count++] = token;
    token = strtok(0, " \t");
  }
  return count;
}

static void QueueStatus(uint32_t now_ms)
{
  const EncoderSample *encoder = Encoder_Get(ENCODER_BEAM);
  Sa100Sample angle = {0};
  VisionBallSample ball = {0};
  float scale;
  float zero;
  float sign;
  (void)SA100_GetSnapshot(&angle);
  (void)VisionUART_GetSnapshot(&ball);
  SA100_GetCalibration(&scale, &zero, &sign);
  QueueMessage(
    "BENCH t=%lu mode=%s fault=%s pwm=%d count=%lld delta=%ld "
    "sa=%u fresh=%u sacal=%u rangeok=%u per=%lu high=%lu raw=%.3f ang=%.3f aref=%.3f "
    "vision=%u vfresh=%u ball=%.2f vel=%.2f bref=%.2f "
    "akp=%.3f aki=%.3f akd=%.3f bkp=%.5f bkd=%.5f bsign=%.0f "
    "plim=%.0f alim=%.2f scale=%.3f zero=%.3f sasign=%.0f\r\n",
    (unsigned long)now_ms, ModeName(bench.mode), FaultName(bench.fault),
    Motor_GetCommand(MOTOR_BEAM), (long long)encoder->total_count,
    (long)encoder->delta_count, angle.valid ? 1U : 0U,
    SA100_IsFresh(now_ms) ? 1U : 0U,
    bench.sa_calibration_confirmed ? 1U : 0U,
    APP_BEAM_RANGE_VERIFIED ? 1U : 0U, (unsigned long)angle.period_us,
    (unsigned long)angle.high_us, angle.raw_angle_deg, angle.beam_angle_deg,
    bench.angle_target_deg, ball.status, VisionUART_IsFresh(now_ms) ? 1U : 0U,
    ball.position_mm, ball.speed_mm_s, bench.ball_target_mm,
    bench.angle_kp, bench.angle_ki, bench.angle_kd, bench.ball_kp,
    bench.ball_kd, bench.ball_sign, bench.pwm_limit, bench.angle_limit_deg,
    scale, zero, sign);
}

static bool EnsureActive(void)
{
  if (bench.mode != BENCH_MODE_OFF) return true;
  QueueMessage("ERR run 'bench on' first\r\n");
  return false;
}

static void ReinitializeAnglePid(void)
{
  PID_Init(&angle_pid, bench.angle_kp, bench.angle_ki, bench.angle_kd,
           30.0f, bench.pwm_limit);
  last_angle_sample_ms = 0U;
}

static void HandleCommand(char *line, uint32_t now_ms)
{
  char *token[6];
  unsigned count = Tokenize(line, token, 6U);
  float a;
  float b;
  float c;
  long integer_a;
  long integer_b;

  if (count == 0U) return;
  if (strcmp(token[0], "help") == 0) {
    QueueMessage("OK commands: bench on|off; status; stop; zero; pulse <signed_pwm> <ms>; "
                 "angle <deg>; ball <mm>; gain angle <kp> <ki> <kd>; "
                 "gain ball <kp> <kd> <sign>; limit pwm <value>; "
                 "limit angle <deg>; sa cal <scale> <zero> <sign>; config\r\n");
    return;
  }
  if ((count == 2U) && (strcmp(token[0], "bench") == 0) &&
      (strcmp(token[1], "on") == 0)) {
    if (App_GetStatus()->state != APP_STATE_STANDBY) {
      QueueMessage("ERR normal app must be in standby before bench on\r\n");
      return;
    }
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    fast_tick_ms = now_ms;
    telemetry_tick_ms = now_ms;
    QueueMessage("OK bench active; chassis disabled; PC13 is emergency stop\r\n");
    return;
  }
  if ((count == 2U) && (strcmp(token[0], "bench") == 0) &&
      (strcmp(token[1], "off") == 0)) {
    StopOutputs();
    bench.mode = BENCH_MODE_OFF;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK bench disabled\r\n");
    return;
  }
  if (!EnsureActive()) return;
  if (strcmp(token[0], "status") == 0) {
    QueueStatus(now_ms);
  } else if (strcmp(token[0], "stop") == 0) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK outputs stopped\r\n");
  } else if (strcmp(token[0], "zero") == 0) {
    if ((bench.mode != BENCH_MODE_IDLE) && (bench.mode != BENCH_MODE_FAULT)) {
      QueueMessage("ERR stop outputs before zero\r\n");
      return;
    }
    StopOutputs();
    Encoder_Reset(ENCODER_BEAM);
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    QueueMessage("OK beam encoder zeroed at manually confirmed safe center\r\n");
  } else if ((count == 3U) && (strcmp(token[0], "pulse") == 0) &&
             ParseLongValue(token[1], &integer_a) &&
             ParseLongValue(token[2], &integer_b)) {
    if ((integer_a == 0L) ||
        (labs(integer_a) > APP_BENCH_OPEN_LOOP_PWM_LIMIT) ||
        (integer_b <= 0L) ||
        ((unsigned long)integer_b > APP_BENCH_PULSE_MAX_MS)) {
      QueueMessage("ERR pulse limits: pwm=-%d..%d nonzero, ms=1..%lu\r\n",
                   APP_BENCH_OPEN_LOOP_PWM_LIMIT,
                   APP_BENCH_OPEN_LOOP_PWM_LIMIT,
                   (unsigned long)APP_BENCH_PULSE_MAX_MS);
      return;
    }
    StopOutputs();
    bench.mode = BENCH_MODE_PULSE;
    bench.fault = BENCH_FAULT_NONE;
    bench.pulse_pwm = (int16_t)integer_a;
    pulse_end_ms = now_ms + (uint32_t)integer_b;
    Motor_EnableBeam(true);
    Motor_Set(MOTOR_BEAM, bench.pulse_pwm);
    QueueMessage("OK pulse pwm=%d ms=%ld; PC13 stops immediately\r\n",
                 bench.pulse_pwm, integer_b);
  } else if ((count == 2U) && (strcmp(token[0], "angle") == 0) &&
             ParseFloatValue(token[1], &a)) {
    if (!APP_BEAM_RANGE_VERIFIED) {
      QueueMessage("ERR set verified P60 limits and APP_BEAM_RANGE_VERIFIED=1 first\r\n");
      return;
    }
    if (!bench.sa_calibration_confirmed) {
      QueueMessage("ERR run and verify 'sa cal', or set APP_SA100_CALIBRATION_VERIFIED=1\r\n");
      return;
    }
    if (fabsf(a) > bench.angle_limit_deg) {
      QueueMessage("ERR angle exceeds current limit %.2f deg\r\n",
                   bench.angle_limit_deg);
      return;
    }
    if (!SA100_IsFresh(now_ms)) {
      QueueMessage("ERR SA100 is not fresh; use status and calibrate first\r\n");
      return;
    }
    StopOutputs();
    ReinitializeAnglePid();
    bench.angle_target_deg = a;
    bench.mode = BENCH_MODE_ANGLE;
    bench.fault = BENCH_FAULT_NONE;
    closed_loop_end_ms = now_ms + APP_BENCH_CLOSED_LOOP_MAX_MS;
    Motor_EnableBeam(true);
    QueueMessage("OK angle loop target=%.3f deg; auto-stop=%lu ms\r\n", a,
                 (unsigned long)APP_BENCH_CLOSED_LOOP_MAX_MS);
  } else if ((count == 2U) && (strcmp(token[0], "ball") == 0) &&
             ParseFloatValue(token[1], &a)) {
    if (!APP_BEAM_RANGE_VERIFIED || !bench.sa_calibration_confirmed) {
      QueueMessage("ERR verified P60 range and SA100 calibration are required\r\n");
      return;
    }
    if (fabsf(a) > APP_BALL_TARGET_LIMIT_MM) {
      QueueMessage("ERR ball target limit is +/-%.1f mm\r\n",
                   APP_BALL_TARGET_LIMIT_MM);
      return;
    }
    if (!SA100_IsFresh(now_ms) || !VisionUART_IsFresh(now_ms)) {
      QueueMessage("ERR fresh SA100 and real vision measurement are required\r\n");
      return;
    }
    StopOutputs();
    ReinitializeAnglePid();
    bench.ball_target_mm = a;
    bench.angle_target_deg = 0.0f;
    bench.mode = BENCH_MODE_BALL;
    bench.fault = BENCH_FAULT_NONE;
    closed_loop_end_ms = now_ms + APP_BENCH_CLOSED_LOOP_MAX_MS;
    Motor_EnableBeam(true);
    QueueMessage("OK ball loop target=%.2f mm; auto-stop=%lu ms\r\n", a,
                 (unsigned long)APP_BENCH_CLOSED_LOOP_MAX_MS);
  } else if ((count == 5U) && (strcmp(token[0], "gain") == 0) &&
             (strcmp(token[1], "angle") == 0) &&
             ParseFloatValue(token[2], &a) && ParseFloatValue(token[3], &b) &&
             ParseFloatValue(token[4], &c) && (a >= 0.0f) && (b >= 0.0f) &&
             (c >= 0.0f)) {
    bench.angle_kp = a;
    bench.angle_ki = b;
    bench.angle_kd = c;
    ReinitializeAnglePid();
    QueueMessage("OK angle gains kp=%.3f ki=%.3f kd=%.3f\r\n", a, b, c);
  } else if ((count == 5U) && (strcmp(token[0], "gain") == 0) &&
             (strcmp(token[1], "ball") == 0) &&
             ParseFloatValue(token[2], &a) && ParseFloatValue(token[3], &b) &&
             ParseFloatValue(token[4], &c) && (a >= 0.0f) && (b >= 0.0f) &&
             ((c == 1.0f) || (c == -1.0f))) {
    bench.ball_kp = a;
    bench.ball_kd = b;
    bench.ball_sign = c;
    QueueMessage("OK ball gains kp=%.5f kd=%.5f sign=%.0f\r\n", a, b, c);
  } else if ((count == 3U) && (strcmp(token[0], "limit") == 0) &&
             (strcmp(token[1], "pwm") == 0) && ParseFloatValue(token[2], &a) &&
             (a >= 0.0f) && (a <= (float)APP_MOTOR_BEAM_MAX_PWM)) {
    bench.pwm_limit = a;
    ReinitializeAnglePid();
    QueueMessage("OK closed-loop pwm limit=%.0f\r\n", a);
  } else if ((count == 3U) && (strcmp(token[0], "limit") == 0) &&
             (strcmp(token[1], "angle") == 0) && ParseFloatValue(token[2], &a) &&
             (a > 0.0f) && (a <= APP_BEAM_ANGLE_SOFT_LIMIT_DEG)) {
    bench.angle_limit_deg = a;
    QueueMessage("OK command angle limit=%.2f deg\r\n", a);
  } else if ((count == 5U) && (strcmp(token[0], "sa") == 0) &&
             (strcmp(token[1], "cal") == 0) &&
             ParseFloatValue(token[2], &a) && ParseFloatValue(token[3], &b) &&
             ParseFloatValue(token[4], &c) && (a > 0.0f) &&
             ((c == 1.0f) || (c == -1.0f))) {
    if ((bench.mode != BENCH_MODE_IDLE) && (bench.mode != BENCH_MODE_FAULT)) {
      QueueMessage("ERR stop outputs before changing SA100 calibration\r\n");
      return;
    }
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    bench.fault = BENCH_FAULT_NONE;
    SA100_SetCalibration(a, b, c);
    bench.sa_calibration_confirmed = true;
    QueueMessage("OK SA100 calibration scale=%.3f zero=%.3f sign=%.0f; wait for new frame\r\n",
                 a, b, c);
  } else if (strcmp(token[0], "config") == 0) {
    float scale;
    float zero;
    float sign;
    SA100_GetCalibration(&scale, &zero, &sign);
    QueueMessage(
      "CONFIG BEAM_CPR=%.1f MOTOR_SIGN=%d ENCODER_SIGN=%.0f MIN_PWM=%d "
      "ENC_MIN=%ld ENC_MAX=%ld STALL_PWM=%d STALL_DELTA=%ld STALL_MS=%lu "
      "SA_SCALE=%.3f SA_ZERO=%.3f SA_SIGN=%.0f ANG_KP=%.3f ANG_KI=%.3f "
      "ANG_KD=%.3f BALL_KP=%.5f BALL_KD=%.5f BALL_SIGN=%.0f "
      "PWM_LIMIT=%.0f ANGLE_LIMIT=%.2f RANGE_OK=%u SA_CAL_OK=%u\r\n",
      APP_ENCODER_BEAM_CPR, APP_MOTOR_BEAM_SIGN, APP_ENCODER_BEAM_SIGN,
      APP_MOTOR_BEAM_MIN_PWM, (long)APP_BEAM_ENCODER_MIN_COUNT,
      (long)APP_BEAM_ENCODER_MAX_COUNT, APP_BEAM_STALL_PWM,
      (long)APP_BEAM_STALL_DELTA_COUNT,
      (unsigned long)APP_BEAM_STALL_TIMEOUT_MS, scale, zero, sign,
      bench.angle_kp, bench.angle_ki, bench.angle_kd, bench.ball_kp,
      bench.ball_kd, bench.ball_sign, bench.pwm_limit,
      bench.angle_limit_deg, APP_BEAM_RANGE_VERIFIED ? 1U : 0U,
      bench.sa_calibration_confirmed ? 1U : 0U);
  } else {
    QueueMessage("ERR invalid command; type help\r\n");
  }
}

static void ServiceRx(uint32_t now_ms)
{
  if (rx_restart_requested) {
    rx_restart_requested = false;
    (void)HAL_UART_AbortReceive(&hlpuart1);
    StartReceive();
  }
  while (rx_tail != rx_head) {
    uint8_t byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % BENCH_RX_RING_SIZE);
    if ((byte == '\r') || (byte == '\n')) {
      if (command_length != 0U) {
        command_line[command_length] = '\0';
        HandleCommand(command_line, now_ms);
        command_length = 0U;
      }
    } else if (command_length < (BENCH_LINE_SIZE - 1U)) {
      command_line[command_length++] = (char)byte;
    } else {
      command_length = 0U;
      QueueMessage("ERR command too long\r\n");
    }
  }
}

static void CheckClosedLoopSafety(uint32_t now_ms, bool need_vision)
{
  const EncoderSample *encoder = Encoder_Get(ENCODER_BEAM);
  Sa100Sample angle = {0};
  (void)SA100_GetSnapshot(&angle);
  if (!SA100_IsFresh(now_ms)) {
    EnterFault(BENCH_FAULT_SA100_TIMEOUT);
  } else if (fabsf(angle.beam_angle_deg) > APP_BEAM_ANGLE_SOFT_LIMIT_DEG) {
    EnterFault(BENCH_FAULT_ANGLE_LIMIT);
  } else if ((encoder->total_count < APP_BEAM_ENCODER_MIN_COUNT) ||
             (encoder->total_count > APP_BEAM_ENCODER_MAX_COUNT)) {
    EnterFault(BENCH_FAULT_ENCODER_LIMIT);
  } else if (need_vision && !VisionUART_IsFresh(now_ms)) {
    EnterFault(BENCH_FAULT_VISION_TIMEOUT);
  } else if ((abs(Motor_GetCommand(MOTOR_BEAM)) >= APP_BEAM_STALL_PWM) &&
             (labs((long)encoder->delta_count) <= APP_BEAM_STALL_DELTA_COUNT)) {
    if (stall_since_ms == 0U) stall_since_ms = now_ms;
    if ((now_ms - stall_since_ms) >= APP_BEAM_STALL_TIMEOUT_MS) {
      EnterFault(BENCH_FAULT_STALL);
    }
  } else {
    stall_since_ms = 0U;
  }
}

static void ClosedLoopUpdate(uint32_t now_ms, uint32_t elapsed_ms)
{
  const EncoderSample *encoder = Encoder_Get(ENCODER_BEAM);
  Sa100Sample angle = {0};
  VisionBallSample vision;
  int16_t command;
  float dt_s;

  if (bench.mode == BENCH_MODE_BALL &&
      VisionUART_ConsumeNewFrame(&vision) && vision.valid) {
    float error = bench.ball_target_mm - vision.position_mm;
    bench.angle_target_deg = bench.ball_sign *
      (bench.ball_kp * error - bench.ball_kd * vision.speed_mm_s);
    bench.angle_target_deg = Clamp(bench.angle_target_deg,
                                   -bench.angle_limit_deg,
                                   bench.angle_limit_deg);
  }

  if (!SA100_GetSnapshot(&angle) || !SA100_IsFresh(now_ms)) return;
  if (angle.timestamp_ms == last_angle_sample_ms) return;
  dt_s = (float)elapsed_ms * 0.001f;
  if (last_angle_sample_ms != 0U) {
    dt_s = (float)(angle.timestamp_ms - last_angle_sample_ms) * 0.001f;
  }
  last_angle_sample_ms = angle.timestamp_ms;
  command = (int16_t)PID_Update(&angle_pid,
    bench.angle_target_deg - angle.beam_angle_deg, dt_s);
  if ((encoder->total_count <= APP_BEAM_ENCODER_MIN_COUNT && command < 0) ||
      (encoder->total_count >= APP_BEAM_ENCODER_MAX_COUNT && command > 0)) {
    command = 0;
    PID_Reset(&angle_pid);
  }
  Motor_Set(MOTOR_BEAM, command);
}

void BenchDebug_Init(uint32_t now_ms)
{
  memset(&bench, 0, sizeof(bench));
  bench.mode = BENCH_MODE_OFF;
  bench.angle_kp = APP_BENCH_INITIAL_ANGLE_KP;
  bench.angle_ki = APP_BENCH_INITIAL_ANGLE_KI;
  bench.angle_kd = APP_BENCH_INITIAL_ANGLE_KD;
  bench.ball_kp = APP_BENCH_INITIAL_BALL_KP;
  bench.ball_kd = APP_BENCH_INITIAL_BALL_KD;
  bench.ball_sign = APP_BALL_CONTROL_SIGN;
  bench.pwm_limit = APP_BENCH_CLOSED_LOOP_PWM_LIMIT;
  bench.angle_limit_deg = APP_BENCH_ANGLE_LIMIT_DEG;
  bench.sa_calibration_confirmed = APP_SA100_CALIBRATION_VERIFIED != 0;
  ReinitializeAnglePid();
  rx_head = 0U;
  rx_tail = 0U;
  command_length = 0U;
  tx_queued_valid = false;
  tx_inflight = false;
  rx_restart_requested = false;
  emergency_stop_requested = false;
  fast_tick_ms = now_ms;
  telemetry_tick_ms = now_ms;
  StartReceive();
}

bool BenchDebug_Run(uint32_t now_ms)
{
  ServiceRx(now_ms);
  if (emergency_stop_requested) {
    emergency_stop_requested = false;
    if (bench.mode != BENCH_MODE_OFF) EnterFault(BENCH_FAULT_ESTOP);
  }

  if (bench.mode == BENCH_MODE_PULSE) {
    const EncoderSample *encoder = Encoder_Get(ENCODER_BEAM);
    if ((int32_t)(now_ms - pulse_end_ms) >= 0) {
      StopOutputs();
      bench.mode = BENCH_MODE_IDLE;
      QueueMessage("OK pulse complete; outputs stopped\r\n");
    } else if ((encoder->total_count <= APP_BEAM_ENCODER_MIN_COUNT &&
                bench.pulse_pwm < 0) ||
               (encoder->total_count >= APP_BEAM_ENCODER_MAX_COUNT &&
                bench.pulse_pwm > 0)) {
      EnterFault(BENCH_FAULT_ENCODER_LIMIT);
    }
  }

  if (((bench.mode == BENCH_MODE_ANGLE) || (bench.mode == BENCH_MODE_BALL)) &&
      ((int32_t)(now_ms - closed_loop_end_ms) >= 0)) {
    StopOutputs();
    bench.mode = BENCH_MODE_IDLE;
    QueueMessage("OK closed-loop time limit reached; outputs stopped\r\n");
  }

  if ((bench.mode != BENCH_MODE_OFF) &&
      ((now_ms - fast_tick_ms) >= APP_CONTROL_FAST_MS)) {
    uint32_t elapsed = now_ms - fast_tick_ms;
    fast_tick_ms = now_ms;
    Encoder_Update(elapsed);
    if ((bench.mode == BENCH_MODE_ANGLE) || (bench.mode == BENCH_MODE_BALL)) {
      CheckClosedLoopSafety(now_ms, bench.mode == BENCH_MODE_BALL);
      if ((bench.mode == BENCH_MODE_ANGLE) || (bench.mode == BENCH_MODE_BALL)) {
        ClosedLoopUpdate(now_ms, elapsed);
      }
    }
  }

  if ((bench.mode != BENCH_MODE_OFF) &&
      ((now_ms - telemetry_tick_ms) >= APP_BENCH_TELEMETRY_MS)) {
    telemetry_tick_ms = now_ms;
    QueueStatus(now_ms);
  }
  ServiceTx();
  return bench.mode != BENCH_MODE_OFF;
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
}

void BenchDebug_UartErrorCallback(void)
{
  rx_restart_requested = true;
}

const BenchDebugStatus *BenchDebug_GetStatus(void)
{
  return &bench;
}

#else

static const BenchDebugStatus disabled_status = {0};

void BenchDebug_Init(uint32_t now_ms) { (void)now_ms; }
bool BenchDebug_Run(uint32_t now_ms) { (void)now_ms; return false; }
bool BenchDebug_IsActive(void) { return false; }
void BenchDebug_RequestEmergencyStop(void) {}
void BenchDebug_UartRxCpltCallback(void) {}
void BenchDebug_UartTxCpltCallback(void) {}
void BenchDebug_UartErrorCallback(void) {}
const BenchDebugStatus *BenchDebug_GetStatus(void) { return &disabled_status; }

#endif
