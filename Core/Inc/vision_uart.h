#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float position_mm;
  float speed_mm_s;
  uint32_t frame_number;
  uint32_t timestamp_ms;
  uint8_t status;
  bool valid;
  bool new_frame;
} VisionBallSample;

void VisionUART_Init(void);
void VisionUART_Service(void);
const VisionBallSample *VisionUART_Get(void);
bool VisionUART_GetSnapshot(VisionBallSample *copy);
bool VisionUART_IsFresh(uint32_t now_ms);
bool VisionUART_ConsumeNewFrame(VisionBallSample *copy);
void VisionUART_RxEventCallback(uint16_t dma_position);
void VisionUART_ErrorCallback(void);

#endif /* VISION_UART_H */
