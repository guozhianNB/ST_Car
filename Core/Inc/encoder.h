#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
  ENCODER_LEFT = 0,
  ENCODER_RIGHT,
  ENCODER_BEAM,
  ENCODER_COUNT
} EncoderId;

typedef struct {
  int32_t delta_count;
  int64_t total_count;
  float speed_mm_s;
} EncoderSample;

void Encoder_Init(void);
void Encoder_Update(uint32_t dt_ms);
const EncoderSample *Encoder_Get(EncoderId id);
void Encoder_Reset(EncoderId id);

#endif /* ENCODER_H */
