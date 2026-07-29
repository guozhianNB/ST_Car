#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 8U

typedef struct {
  uint8_t active_mask;
  uint8_t active_count;
  float error;
  bool line_found;
  bool cross_line;
  uint32_t timestamp_ms;
} LineSensorSample;

void LineSensor_Update(void);
const LineSensorSample *LineSensor_Get(void);

#endif /* LINE_SENSOR_H */
