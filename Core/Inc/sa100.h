#ifndef SA100_H
#define SA100_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float duty_cycle;
  float raw_angle_deg;
  float beam_angle_deg;
  uint32_t period_us;
  uint32_t high_us;
  uint32_t timestamp_ms;
  bool valid;
} Sa100Sample;

void SA100_Init(void);
const Sa100Sample *SA100_Get(void);
bool SA100_GetSnapshot(Sa100Sample *copy);
bool SA100_IsFresh(uint32_t now_ms);
void SA100_SetCalibration(float duty_to_deg, float horizontal_raw_deg,
                          float angle_sign);
void SA100_GetCalibration(float *duty_to_deg, float *horizontal_raw_deg,
                          float *angle_sign);
void SA100_CaptureCallback(void);

#endif /* SA100_H */
