#include "sa100.h"
#include "app_config.h"
#include "main.h"
#include "tim.h"

static volatile Sa100Sample sample;
static float previous_raw_deg;
static float continuous_raw_deg;
static bool angle_initialized;

void SA100_Init(void)
{
  sample.valid = false;
  sample.timestamp_ms = 0;
  angle_initialized = false;
  if (HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

const Sa100Sample *SA100_Get(void)
{
  return (const Sa100Sample *)&sample;
}

bool SA100_IsFresh(uint32_t now_ms)
{
  return sample.valid && ((now_ms - sample.timestamp_ms) <= APP_SA100_TIMEOUT_MS);
}

void SA100_CaptureCallback(void)
{
  uint32_t period = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
  uint32_t high = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
  float raw;
  float delta;

  if ((period < APP_SA100_PERIOD_MIN_US) ||
      (period > APP_SA100_PERIOD_MAX_US) || (high > period)) {
    sample.valid = false;
    return;
  }
  raw = ((float)high / (float)period) * APP_SA100_DUTY_TO_DEG;
  if (!angle_initialized) {
    continuous_raw_deg = raw;
    angle_initialized = true;
  } else {
    delta = raw - previous_raw_deg;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    continuous_raw_deg += delta;
  }
  previous_raw_deg = raw;
  sample.raw_angle_deg = continuous_raw_deg;
  sample.beam_angle_deg = (continuous_raw_deg - APP_SA100_HORIZONTAL_RAW_DEG) *
                          APP_SA100_ANGLE_SIGN;
  sample.period_us = period;
  sample.high_us = high;
  sample.timestamp_ms = HAL_GetTick();
  sample.valid = true;
}
