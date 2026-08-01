#ifndef APP_H
#define APP_H

#include "safety_monitor.h"
#include <stdint.h>

typedef enum {
  APP_MODE_LINE_ONLY = 0,
  APP_MODE_STATIC_BALL,
  APP_MODE_MOVING_CENTER_AB,
  APP_MODE_MOVING_CENTER_LAP,
  APP_MODE_MOVING_TARGET,
  APP_MODE_HOLD_CURRENT,
  APP_MODE_COUNT
} AppMode;

typedef enum {
  APP_STATE_STANDBY = 0,
  APP_STATE_CENTERING,
  APP_STATE_RUNNING,
  APP_STATE_FINISHED,
  APP_STATE_FAULT
} AppRunState;

typedef struct {
  AppMode selected_mode;
  AppRunState state;
  FaultCode fault;
  uint32_t run_time_ms;
  float commanded_ball_target_mm;
  uint8_t stage;
} AppStatus;

void App_Init(void);
void App_Run(void);
/* Diagnostic injection uses the same request path as every physical button. */
void App_RequestRequirement3(void);
void App_SelectMode(AppMode mode);
void App_SetMovingTarget(float target_mm);
const AppStatus *App_GetStatus(void);

#endif /* APP_H */
