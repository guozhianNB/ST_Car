#include "vision_uart.h"
#include "app_config.h"
#include "main.h"
#include "usart.h"
#include <ctype.h>
#include <string.h>

#define VISION_DMA_BUFFER_SIZE 128U
#define VISION_LINE_BUFFER_SIZE 64U

static uint8_t dma_buffer[VISION_DMA_BUFFER_SIZE];
static uint16_t dma_previous;
static char line_buffer[VISION_LINE_BUFFER_SIZE];
static uint8_t line_length;
static volatile VisionBallSample sample;
static float previous_position_mm;
static uint32_t previous_timestamp_ms;
static volatile bool restart_requested;

static bool ParseFloat(const char **cursor, float *value)
{
  float result = 0.0f;
  float fraction = 0.1f;
  int sign = 1;
  bool has_digit = false;
  const char *p = *cursor;
  if (*p == '-') { sign = -1; ++p; }
  else if (*p == '+') { ++p; }
  while (isdigit((unsigned char)*p)) {
    result = result * 10.0f + (float)(*p++ - '0');
    has_digit = true;
  }
  if (*p == '.') {
    ++p;
    while (isdigit((unsigned char)*p)) {
      result += (float)(*p++ - '0') * fraction;
      fraction *= 0.1f;
      has_digit = true;
    }
  }
  if (!has_digit) return false;
  *value = result * (float)sign;
  *cursor = p;
  return true;
}

static bool ParseUnsigned(const char **cursor, uint32_t *value)
{
  uint32_t result = 0;
  bool has_digit = false;
  const char *p = *cursor;
  while (isdigit((unsigned char)*p)) {
    result = result * 10U + (uint32_t)(*p++ - '0');
    has_digit = true;
  }
  if (!has_digit) return false;
  *value = result;
  *cursor = p;
  return true;
}

static void ParseLine(char *line)
{
  const char *p = line;
  float position;
  uint32_t status;
  uint32_t now;
  float speed = 0.0f;
  bool format_ok = false;

  if ((p[0] == '$') && (p[1] == 'B') && (p[2] == ',')) {
    p += 3;
    format_ok = ParseFloat(&p, &position) && (*p++ == ',') &&
                ParseUnsigned(&p, &status);
  } else if (strncmp(p, "BALL,", 5) == 0) {
    p += 5;
    format_ok = ParseFloat(&p, &position) && (*p++ == ',') &&
                ParseUnsigned(&p, &status);
  }
  if (!format_ok || (status > 2U) ||
      (position < -APP_BALL_POSITION_LIMIT_MM) ||
      (position > APP_BALL_POSITION_LIMIT_MM)) return;

  sample.status = (uint8_t)status;
  sample.frame_number++;
  if (status == 1U) {
    now = HAL_GetTick();
    if ((previous_timestamp_ms != 0U) && (now != previous_timestamp_ms)) {
      float raw_speed = (position - previous_position_mm) * 1000.0f /
                        (float)(now - previous_timestamp_ms);
      speed = sample.speed_mm_s + APP_BALL_SPEED_FILTER_ALPHA *
              (raw_speed - sample.speed_mm_s);
    }
    previous_position_mm = position;
    previous_timestamp_ms = now;
    sample.position_mm = position;
    sample.speed_mm_s = speed;
    sample.timestamp_ms = now;
    sample.valid = true;
    sample.new_frame = true;
  } else {
    /* status=2 is a held estimate and status=0 is invalid: neither is a
       measurement, so they must not replace a pending measurement or refresh
       velocity/the timeout. Freshness expires from the last status=1 frame. */
  }
}

static void FeedByte(uint8_t byte)
{
  if ((byte == '\n') || (byte == '\r')) {
    if (line_length != 0U) {
      line_buffer[line_length] = '\0';
      ParseLine(line_buffer);
      line_length = 0;
    }
  } else if (line_length < (VISION_LINE_BUFFER_SIZE - 1U)) {
    line_buffer[line_length++] = (char)byte;
  } else {
    line_length = 0;
  }
}

void VisionUART_Init(void)
{
  memset((void *)&sample, 0, sizeof(sample));
  dma_previous = 0;
  line_length = 0;
  restart_requested = false;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, dma_buffer,
                                   VISION_DMA_BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

void VisionUART_Service(void)
{
  if (!restart_requested) return;
  restart_requested = false;
  (void)HAL_UART_AbortReceive(&huart4);
  dma_previous = 0;
  line_length = 0;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart4, dma_buffer,
                                   VISION_DMA_BUFFER_SIZE) != HAL_OK) {
    restart_requested = true;
    return;
  }
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
}

const VisionBallSample *VisionUART_Get(void)
{
  return (const VisionBallSample *)&sample;
}

bool VisionUART_GetSnapshot(VisionBallSample *copy)
{
  uint32_t primask;
  if (copy == 0) return false;
  primask = __get_PRIMASK();
  __disable_irq();
  *copy = sample;
  if (primask == 0U) __enable_irq();
  return copy->valid;
}

bool VisionUART_IsFresh(uint32_t now_ms)
{
  return (sample.timestamp_ms != 0U) &&
         ((now_ms - sample.timestamp_ms) <= APP_VISION_TIMEOUT_MS);
}

bool VisionUART_ConsumeNewFrame(VisionBallSample *copy)
{
  bool available;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  available = sample.new_frame;
  if (available) {
    if (copy != 0) *copy = sample;
    sample.new_frame = false;
  }
  if (primask == 0U) __enable_irq();
  return available;
}

void VisionUART_RxEventCallback(uint16_t dma_position)
{
  uint16_t end = dma_position;
  if (end > VISION_DMA_BUFFER_SIZE) return;
  if (end < dma_previous) {
    while (dma_previous < VISION_DMA_BUFFER_SIZE) {
      FeedByte(dma_buffer[dma_previous++]);
    }
    dma_previous = 0;
  }
  while (dma_previous < end) {
    FeedByte(dma_buffer[dma_previous]);
    ++dma_previous;
  }
  if (dma_previous == VISION_DMA_BUFFER_SIZE) dma_previous = 0;
}

void VisionUART_ErrorCallback(void)
{
  restart_requested = true;
}
