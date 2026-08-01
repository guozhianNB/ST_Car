#ifndef STEPPER_H
#define STEPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool enabled;
  int32_t requested_rate_steps_s;
  int32_t applied_rate_steps_s;
} StepperStatus;

void Stepper_Init(void);
void Stepper_Enable(bool enable);
void Stepper_SetRate(int32_t signed_rate_steps_s, uint32_t now_ms);
void Stepper_Stop(void);
const StepperStatus *Stepper_GetStatus(void);

#endif /* STEPPER_H */
