#include "safety_monitor.h"
#include "app_config.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "stepper.h"
#include "vision_uart.h"
#include <stdlib.h>

static uint32_t begin_ms;
static uint32_t line_lost_since_ms;
static uint32_t stall_since_ms;
static uint32_t stall_last_drive_ms;

void SafetyMonitor_Begin(uint32_t now_ms)
{
  begin_ms = now_ms;
  line_lost_since_ms = 0;
  stall_since_ms = 0;
  stall_last_drive_ms = 0;
}

FaultCode SafetyMonitor_Update(uint32_t now_ms, bool need_line,
                               bool need_vision, bool need_actuator)
{
  const EncoderSample *actuator_encoder = Encoder_Get(ENCODER_ACTUATOR);
  const StepperStatus *stepper = Stepper_GetStatus();
  const LineSensorSample *line = LineSensor_Get();

  if (need_actuator) {
    if ((abs(stepper->applied_rate_steps_s) >=
         APP_STEPPER_STALL_MIN_RATE_STEPS_S) &&
        (abs(actuator_encoder->delta_count) <=
         APP_STEPPER_STALL_DELTA_COUNT)) {
      stall_last_drive_ms = now_ms;
      if (stall_since_ms == 0U) stall_since_ms = now_ms;
      if ((now_ms - stall_since_ms) >= APP_STEPPER_STALL_TIMEOUT_MS)
        return FAULT_ACTUATOR_STALL;
    } else if (abs(actuator_encoder->delta_count) >
               APP_STEPPER_STALL_DELTA_COUNT) {
      stall_since_ms = 0;
    } else if ((stall_last_drive_ms != 0U) &&
               ((now_ms - stall_last_drive_ms) >
                APP_STEPPER_STALL_RESET_MS)) {
      stall_since_ms = 0;
    }
  }

  if (need_vision && !VisionUART_IsFresh(now_ms)) {
    return FAULT_VISION_TIMEOUT;
  }

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
    "none", "actuator_stall", "vision_timeout", "line_lost",
    "stage_timeout", "startup_check"
  };
  return ((unsigned)fault < (sizeof(names) / sizeof(names[0]))) ? names[fault] : "unknown";
}
