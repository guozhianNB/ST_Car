#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  FAULT_NONE = 0,
  FAULT_SA100_TIMEOUT,
  FAULT_BEAM_ANGLE_LIMIT,
  FAULT_BEAM_ENCODER_LIMIT,
  FAULT_BEAM_STALL,
  FAULT_VISION_TIMEOUT,
  FAULT_LINE_LOST,
  FAULT_STAGE_TIMEOUT,
  FAULT_STARTUP_CHECK
} FaultCode;

void SafetyMonitor_Begin(uint32_t now_ms);
FaultCode SafetyMonitor_Update(uint32_t now_ms, bool need_line,
                               bool need_vision, bool need_beam);
const char *SafetyMonitor_FaultName(FaultCode fault);

#endif /* SAFETY_MONITOR_H */
