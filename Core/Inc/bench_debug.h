#ifndef BENCH_DEBUG_H
#define BENCH_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  BENCH_MODE_OFF = 0,
  BENCH_MODE_IDLE,
  BENCH_MODE_PULSE,
  BENCH_MODE_ANGLE,
  BENCH_MODE_BALL,
  BENCH_MODE_BALL_SEQUENCE,
  BENCH_MODE_FAULT
} BenchMode;

typedef enum {
  BENCH_FAULT_NONE = 0,
  BENCH_FAULT_ESTOP,
  BENCH_FAULT_SA100_TIMEOUT,
  BENCH_FAULT_ANGLE_LIMIT,
  BENCH_FAULT_ENCODER_LIMIT,
  BENCH_FAULT_STALL,
  BENCH_FAULT_VISION_TIMEOUT,
  BENCH_FAULT_SEQUENCE_TIMEOUT
} BenchFault;

typedef struct {
  BenchMode mode;
  BenchFault fault;
  int16_t pulse_pwm;
  float angle_target_deg;
  float ball_target_mm;
  float angle_kp;
  float angle_ki;
  float angle_kd;
  float ball_kp;
  float ball_kd;
  float ball_sign;
  float pwm_limit;
  float angle_limit_deg;
  bool sa_calibration_confirmed;
  bool stream_enabled;
  uint32_t stream_period_ms;
  uint8_t sequence_stage;
  uint32_t sequence_elapsed_ms;
} BenchDebugStatus;

void BenchDebug_Init(uint32_t now_ms);
bool BenchDebug_Run(uint32_t now_ms);
bool BenchDebug_IsActive(void);
void BenchDebug_RequestEmergencyStop(void);
void BenchDebug_UartRxCpltCallback(void);
void BenchDebug_UartTxCpltCallback(void);
void BenchDebug_UartErrorCallback(void);
const BenchDebugStatus *BenchDebug_GetStatus(void);

#endif /* BENCH_DEBUG_H */
