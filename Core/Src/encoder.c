#include "encoder.h"
#include "app_config.h"
#include "tim.h"
#include <math.h>

typedef struct {
  TIM_HandleTypeDef *timer;
  uint32_t previous;
  float counts_per_rev;
  float sign;
  uint8_t bits;
} EncoderHardware;

static EncoderHardware encoder_hw[ENCODER_COUNT] = {
  {&htim2, 0, APP_ENCODER_LEFT_CPR, APP_ENCODER_LEFT_SIGN, 32},
  {&htim3, 0, APP_ENCODER_RIGHT_CPR, APP_ENCODER_RIGHT_SIGN, 16},
  {&htim4, 0, APP_ENCODER_ACTUATOR_CPR, APP_ENCODER_ACTUATOR_SIGN, 16}
};
static EncoderSample encoder_sample[ENCODER_COUNT];

void Encoder_Init(void)
{
  unsigned i;
  for (i = 0; i < ENCODER_COUNT; ++i) {
    if (HAL_TIM_Encoder_Start(encoder_hw[i].timer, TIM_CHANNEL_ALL) != HAL_OK) {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(encoder_hw[i].timer, 0);
    encoder_hw[i].previous = 0;
    encoder_sample[i].delta_count = 0;
    encoder_sample[i].total_count = 0;
    encoder_sample[i].speed_mm_s = 0.0f;
  }
}

void Encoder_Update(uint32_t dt_ms)
{
  unsigned i;
  const float circumference_mm = APP_WHEEL_DIAMETER_MM * 3.14159265359f;
  if (dt_ms == 0U) return;
  for (i = 0; i < ENCODER_COUNT; ++i) {
    uint32_t current = __HAL_TIM_GET_COUNTER(encoder_hw[i].timer);
    int32_t delta = (encoder_hw[i].bits == 16U)
                    ? (int32_t)(int16_t)((uint16_t)current - (uint16_t)encoder_hw[i].previous)
                    : (int32_t)(current - encoder_hw[i].previous);
    delta = (int32_t)((float)delta * encoder_hw[i].sign);
    encoder_hw[i].previous = current;
    encoder_sample[i].delta_count = delta;
    encoder_sample[i].total_count += delta;
    if (i == ENCODER_ACTUATOR) {
      encoder_sample[i].speed_mm_s = 0.0f;
    } else {
      encoder_sample[i].speed_mm_s = ((float)delta * circumference_mm * 1000.0f) /
                                     (encoder_hw[i].counts_per_rev * (float)dt_ms);
    }
  }
}

const EncoderSample *Encoder_Get(EncoderId id)
{
  return ((unsigned)id < ENCODER_COUNT) ? &encoder_sample[id] : 0;
}

void Encoder_Reset(EncoderId id)
{
  if ((unsigned)id >= ENCODER_COUNT) return;
  __HAL_TIM_SET_COUNTER(encoder_hw[id].timer, 0);
  encoder_hw[id].previous = 0;
  encoder_sample[id].delta_count = 0;
  encoder_sample[id].total_count = 0;
  encoder_sample[id].speed_mm_s = 0.0f;
}
