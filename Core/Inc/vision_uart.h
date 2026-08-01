#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float position_mm;
  float speed_mm_s;
  uint32_t frame_number;
  uint32_t timestamp_ms;
  uint32_t packet_timestamp_ms;
  uint32_t measurement_interval_ms;
  uint32_t maximum_measurement_interval_ms;
  uint32_t accepted_measurements;
  uint32_t rejected_measurements;
  uint8_t status;
  bool valid;
  bool new_frame;
} VisionBallSample;

void VisionUART_Init(void);
void VisionUART_Service(void);
const VisionBallSample *VisionUART_Get(void);
bool VisionUART_GetSnapshot(VisionBallSample *copy);
/* Returns the most recent genuine status=1 measurement while it is no older
 * than max_age_ms.  Interleaved status=0/2 packets never refresh its data or
 * timestamp and therefore cannot masquerade as a new measurement. */
bool VisionUART_GetCurrentMeasurement(VisionBallSample *copy, uint32_t now_ms,
                                      uint32_t max_age_ms);
bool VisionUART_IsFresh(uint32_t now_ms);
bool VisionUART_ConsumeNewFrame(VisionBallSample *copy);
void VisionUART_RxEventCallback(uint16_t dma_position);
void VisionUART_ErrorCallback(void);

#endif /* VISION_UART_H */
