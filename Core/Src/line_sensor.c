#include "line_sensor.h"
#include "app_config.h"
#include "main.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  float weight;
} LineInput;

static const LineInput inputs[LINE_SENSOR_COUNT] = {
  {LINE_0_GPIO_Port, LINE_0_Pin, -3.5f},
  {LINE_1_GPIO_Port, LINE_1_Pin, -2.5f},
  {LINE_2_GPIO_Port, LINE_2_Pin, -1.5f},
  {LINE_3_GPIO_Port, LINE_3_Pin, -0.5f},
  {LINE_4_GPIO_Port, LINE_4_Pin,  0.5f},
  {LINE_5_GPIO_Port, LINE_5_Pin,  1.5f},
  {LINE_6_GPIO_Port, LINE_6_Pin,  2.5f},
  {LINE_7_GPIO_Port, LINE_7_Pin,  3.5f}
};
static LineSensorSample sample;
static float last_error;

void LineSensor_Update(void)
{
  unsigned i;
  float weighted_sum = 0.0f;
  uint8_t count = 0;
  uint8_t mask = 0;
  for (i = 0; i < LINE_SENSOR_COUNT; ++i) {
    if (HAL_GPIO_ReadPin(inputs[i].port, inputs[i].pin) == APP_LINE_ACTIVE_LEVEL) {
      mask |= (uint8_t)(1U << i);
      weighted_sum += inputs[i].weight;
      ++count;
    }
  }
  sample.active_mask = mask;
  sample.active_count = count;
  sample.line_found = (count != 0U);
  sample.cross_line = (count >= APP_LINE_CROSS_MIN_ACTIVE);
  if (count != 0U) {
    sample.error = weighted_sum / ((float)count * 3.5f);
    last_error = sample.error;
  } else {
    sample.error = last_error;
  }
  sample.timestamp_ms = HAL_GetTick();
}

const LineSensorSample *LineSensor_Get(void)
{
  return &sample;
}
