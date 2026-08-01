#include "sa100.h"
#include "app_config.h"
#include "main.h"
#include "tim.h"

static volatile Sa100Sample sample;
static float previous_raw_deg;
static float continuous_raw_deg;
static float filtered_raw_deg;
static bool angle_initialized;
static uint8_t stabilization_count;
static volatile float calibration_duty_to_deg = APP_SA100_DUTY_TO_DEG;
static volatile float calibration_horizontal_raw_deg = APP_SA100_HORIZONTAL_RAW_DEG;
static volatile float calibration_angle_sign = APP_SA100_ANGLE_SIGN;

void SA100_Init(void)
{
  sample.valid = false;
  sample.timestamp_ms = 0;
  angle_initialized = false;
  stabilization_count = 0U;
  if (HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
}

const Sa100Sample *SA100_Get(void)
{
  return (const Sa100Sample *)&sample;
}

bool SA100_GetSnapshot(Sa100Sample *copy)
{
  uint32_t primask;
  if (copy == 0) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *copy = sample;
  if (primask == 0U) __enable_irq();
  return copy->valid;
}

bool SA100_IsFresh(uint32_t now_ms)
{
  return sample.valid && ((now_ms - sample.timestamp_ms) <= APP_SA100_TIMEOUT_MS);
}

void SA100_SetCalibration(float duty_to_deg, float horizontal_raw_deg,
                          float angle_sign)
{
  uint32_t primask;
  if ((duty_to_deg <= 0.0f) ||
      ((angle_sign != 1.0f) && (angle_sign != -1.0f))) return;
  primask = __get_PRIMASK();
  __disable_irq();
  calibration_duty_to_deg = duty_to_deg;
  calibration_horizontal_raw_deg = horizontal_raw_deg;
  calibration_angle_sign = angle_sign;
  angle_initialized = false;
  stabilization_count = 0U;
  sample.valid = false;
  if (primask == 0U) __enable_irq();
}

void SA100_GetCalibration(float *duty_to_deg, float *horizontal_raw_deg,
                          float *angle_sign)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (duty_to_deg != 0) *duty_to_deg = calibration_duty_to_deg;
  if (horizontal_raw_deg != 0) *horizontal_raw_deg = calibration_horizontal_raw_deg;
  if (angle_sign != 0) *angle_sign = calibration_angle_sign;
  if (primask == 0U) __enable_irq();
}

float SA100_AdjustHorizontalRaw(float delta_deg, float maximum_offset_deg)
{
  uint32_t primask;
  float low = APP_SA100_HORIZONTAL_RAW_DEG - maximum_offset_deg;
  float high = APP_SA100_HORIZONTAL_RAW_DEG + maximum_offset_deg;
  float next;
  if (maximum_offset_deg < 0.0f) return calibration_horizontal_raw_deg;
  primask = __get_PRIMASK();
  __disable_irq();
  next = calibration_horizontal_raw_deg + delta_deg;
  if (next < low) next = low;
  if (next > high) next = high;
  calibration_horizontal_raw_deg = next;
  sample.beam_angle_deg =
    (filtered_raw_deg - calibration_horizontal_raw_deg) *
    calibration_angle_sign;
  if (primask == 0U) __enable_irq();
  return next;
}

void SA100_CaptureCallback(void)
{
  uint32_t period_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
  uint32_t high_ticks = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
  const uint32_t ticks_per_us = APP_SA100_CAPTURE_TICKS_PER_US;
  float raw;
  float delta;

  if ((period_ticks < (APP_SA100_PERIOD_MIN_US * ticks_per_us)) ||
      (period_ticks > (APP_SA100_PERIOD_MAX_US * ticks_per_us)) ||
      (high_ticks > period_ticks)) {
    sample.valid = false;
    return;
  }
  sample.duty_cycle = (float)high_ticks / (float)period_ticks;
  raw = sample.duty_cycle * calibration_duty_to_deg;
  if (!angle_initialized) {
    if (stabilization_count < 2U) {
      stabilization_count++;
      previous_raw_deg = raw;
      sample.valid = false;
      return;
    }
    continuous_raw_deg = raw;
    filtered_raw_deg = raw;
    angle_initialized = true;
  } else {
    delta = raw - previous_raw_deg;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    continuous_raw_deg += delta;
    filtered_raw_deg += APP_SA100_ANGLE_FILTER_ALPHA *
                        (continuous_raw_deg - filtered_raw_deg);
  }
  previous_raw_deg = raw;
  sample.raw_angle_deg = continuous_raw_deg;
  sample.beam_angle_deg = (filtered_raw_deg - calibration_horizontal_raw_deg) *
                          calibration_angle_sign;
  sample.period_us = (period_ticks + (ticks_per_us / 2U)) / ticks_per_us;
  sample.high_us = (high_ticks + (ticks_per_us / 2U)) / ticks_per_us;
  sample.timestamp_ms = HAL_GetTick();
  sample.valid = true;
}
