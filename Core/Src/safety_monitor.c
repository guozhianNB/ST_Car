#include "safety_monitor.h"
#include "app_config.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "sa100.h"
#include "vision_uart.h"
#include <math.h>
#include <stdlib.h>

static uint32_t begin_ms;
static uint32_t line_lost_since_ms;
static uint32_t stall_since_ms;

void SafetyMonitor_Begin(uint32_t now_ms)
{
  begin_ms = now_ms;
  line_lost_since_ms = 0;
  stall_since_ms = 0;
}

FaultCode SafetyMonitor_Update(uint32_t now_ms, bool need_line,
                               bool need_vision, bool need_beam)
{
  const EncoderSample *beam_encoder = Encoder_Get(ENCODER_BEAM);
  const Sa100Sample *angle = SA100_Get();
  const LineSensorSample *line = LineSensor_Get();

  if (need_beam) {
    if ((now_ms - begin_ms) > APP_VISION_STARTUP_GRACE_MS &&
        !SA100_IsFresh(now_ms)) return FAULT_SA100_TIMEOUT;
    if (angle->valid && fabsf(angle->beam_angle_deg) > APP_BEAM_ANGLE_SOFT_LIMIT_DEG)
      return FAULT_BEAM_ANGLE_LIMIT;
    if ((beam_encoder->total_count < APP_BEAM_ENCODER_MIN_COUNT) ||
        (beam_encoder->total_count > APP_BEAM_ENCODER_MAX_COUNT))
      return FAULT_BEAM_ENCODER_LIMIT;
    if ((abs(Motor_GetCommand(MOTOR_BEAM)) >= APP_BEAM_STALL_PWM) &&
        (abs(beam_encoder->delta_count) <= APP_BEAM_STALL_DELTA_COUNT)) {
      if (stall_since_ms == 0U) stall_since_ms = now_ms;
      if ((now_ms - stall_since_ms) >= APP_BEAM_STALL_TIMEOUT_MS)
        return FAULT_BEAM_STALL;
    } else {
      stall_since_ms = 0;
    }
  }

  if (need_vision && ((now_ms - begin_ms) > APP_VISION_STARTUP_GRACE_MS) &&
      !VisionUART_IsFresh(now_ms)) return FAULT_VISION_TIMEOUT;

  if (need_line && ((now_ms - begin_ms) > APP_LINE_STARTUP_GRACE_MS)) {
    if (!line->line_found) {
      if (line_lost_since_ms == 0U) line_lost_since_ms = now_ms;
      if ((now_ms - line_lost_since_ms) >= APP_LINE_LOST_TIMEOUT_MS)
        return FAULT_LINE_LOST;
    } else {
      line_lost_since_ms = 0;
    }
  }
  return FAULT_NONE;
}

const char *SafetyMonitor_FaultName(FaultCode fault)
{
  static const char *const names[] = {
    "none", "sa100_timeout", "beam_angle_limit", "beam_encoder_limit",
    "beam_stall", "vision_timeout", "line_lost", "stage_timeout",
    "startup_check"
  };
  return ((unsigned)fault < (sizeof(names) / sizeof(names[0]))) ? names[fault] : "unknown";
}
