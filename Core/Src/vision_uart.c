#include "vision_uart.h"
#include "app_config.h"
#include "main.h"
#include "usart.h"
#include <ctype.h>
#include <math.h>
#include <string.h>

#define VISION_DMA_BUFFER_SIZE 128U
#define VISION_LINE_BUFFER_SIZE 64U

static uint8_t dma_buffer[VISION_DMA_BUFFER_SIZE];
static uint16_t dma_previous;
static char line_buffer[VISION_LINE_BUFFER_SIZE];
static uint8_t line_length;
static volatile VisionBallSample sample;
static volatile VisionBallSample pending_measurement;
static volatile bool measurement_pending;
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
  float reported_speed = 0.0f;
  uint32_t status;
  uint32_t now;
  float speed = 0.0f;
  bool format_ok = false;
  bool has_reported_speed = false;

  if ((p[0] == '$') && (p[1] == 'B') && (p[2] == ',')) {
    p += 3;
    format_ok = ParseFloat(&p, &position) && (*p++ == ',') &&
                ParseUnsigned(&p, &status);
  } else if (strncmp(p, "BALL,", 5) == 0) {
    p += 5;
    format_ok = ParseFloat(&p, &position) && (*p++ == ',') &&
                ParseUnsigned(&p, &status);
  }
  if (format_ok && (*p == ',')) {
    ++p;
    has_reported_speed = ParseFloat(&p, &reported_speed);
  }
  if (!format_ok || (status > 2U) ||
      (position < -APP_BALL_POSITION_LIMIT_MM) ||
      (position > APP_BALL_POSITION_LIMIT_MM) ||
      (has_reported_speed &&
       (fabsf(reported_speed) > APP_VISION_MAX_SPEED_MM_S))) return;

  now = HAL_GetTick();
  sample.packet_timestamp_ms = now;
  sample.status = (uint8_t)status;
  sample.frame_number++;
  if (status == 1U) {
    if ((previous_timestamp_ms != 0U) && (now != previous_timestamp_ms)) {
      uint32_t elapsed_ms = now - previous_timestamp_ms;
      sample.measurement_interval_ms = elapsed_ms;
      if (elapsed_ms > sample.maximum_measurement_interval_ms) {
        sample.maximum_measurement_interval_ms = elapsed_ms;
      }
      float raw_speed = (position - previous_position_mm) * 1000.0f /
                        (float)elapsed_ms;
      if (!has_reported_speed &&
          (elapsed_ms <= APP_VISION_SPEED_RESET_MS) &&
          (fabsf(raw_speed) > APP_VISION_MAX_SPEED_MM_S)) {
        /* A high-confidence reflection can still jump across the tube.  Such
           a frame is not a physical ball measurement and must not refresh the
           position, velocity or freshness timestamp. */
        sample.status = 0U;
        sample.rejected_measurements++;
        return;
      }
      if (has_reported_speed) {
        speed = reported_speed;
      } else if (elapsed_ms <= APP_VISION_SPEED_RESET_MS) {
        speed = sample.speed_mm_s + APP_BALL_SPEED_FILTER_ALPHA *
                (raw_speed - sample.speed_mm_s);
      }
    } else if (has_reported_speed) {
      speed = reported_speed;
    }
    previous_position_mm = position;
    previous_timestamp_ms = now;
    sample.position_mm = position;
    sample.speed_mm_s = speed;
    sample.timestamp_ms = now;
    sample.valid = true;
    sample.new_frame = true;
    sample.accepted_measurements++;
    pending_measurement.position_mm = sample.position_mm;
    pending_measurement.speed_mm_s = sample.speed_mm_s;
    pending_measurement.frame_number = sample.frame_number;
    pending_measurement.timestamp_ms = sample.timestamp_ms;
    pending_measurement.status = 1U;
    pending_measurement.valid = true;
    pending_measurement.new_frame = true;
    measurement_pending = true;
  } else {
    /* status=2 is a held estimate and status=0 is invalid: neither is a
       measurement, so they update only the latest protocol status.  The
       separately snapshotted pending status=1 measurement, position, velocity
       and timestamp remain coherent until the control loop consumes them. */
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
  memset((void *)&pending_measurement, 0, sizeof(pending_measurement));
  measurement_pending = false;
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

bool VisionUART_GetCurrentMeasurement(VisionBallSample *copy, uint32_t now_ms,
                                      uint32_t max_age_ms)
{
  int32_t age_ms;
  if (!VisionUART_GetSnapshot(copy)) return false;
  /* Start and scoring gates require the latest protocol packet itself to be a
     genuine status=1 measurement.  The controller consumes the separately
     snapshotted accepted frame and may coast only for its short hold window. */
  age_ms = (int32_t)(now_ms - copy->timestamp_ms);
  return (copy->status == 1U) && (copy->timestamp_ms != 0U) &&
         ((age_ms < 0) || ((uint32_t)age_ms <= max_age_ms));
}

bool VisionUART_IsFresh(uint32_t now_ms)
{
  uint32_t timestamp_ms = sample.timestamp_ms;
  int32_t age_ms = (int32_t)(now_ms - timestamp_ms);
  /* The UART ISR can publish a measurement after the caller sampled now_ms.
     A small negative signed age is therefore fresh, not a 32-bit timeout. */
  return (timestamp_ms != 0U) &&
         ((age_ms < 0) ||
          ((uint32_t)age_ms <= APP_VISION_TIMEOUT_MS));
}

bool VisionUART_ConsumeNewFrame(VisionBallSample *copy)
{
  bool available;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  available = measurement_pending;
  if (available) {
    if (copy != 0) *copy = pending_measurement;
    measurement_pending = false;
    pending_measurement.new_frame = false;
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
