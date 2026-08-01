#include "oled_display.h"
#include "app.h"
#include "app_config.h"
#include "button.h"
#include "control_loops.h"
#include "encoder.h"
#include "i2c.h"
#include "vision_uart.h"
#include <stdio.h>
#include <string.h>

#define OLED_WIDTH       128U
#define OLED_PAGES         8U
#define OLED_REFRESH_MS  100U
#define OLED_REPROBE_MS 1000U

typedef enum {
  OLED_TX_IDLE = 0,
  OLED_TX_WAIT_COMMAND,
  OLED_TX_WAIT_DATA
} OledTxState;

static uint8_t framebuffer[OLED_PAGES][OLED_WIDTH];
static uint8_t command_buffer[4];
static uint8_t data_buffer[OLED_WIDTH + 1U];
static volatile bool transfer_complete;
static volatile bool transfer_error;
static OledTxState tx_state;
static uint8_t current_page;
static uint8_t oled_address_8bit;
static bool oled_present;
static uint32_t next_refresh_ms;
static uint32_t next_probe_ms;

static const uint8_t *Glyph(char ch)
{
  static const uint8_t blank[5] = {0, 0, 0, 0, 0};
  static const uint8_t unknown[5] = {0x02, 0x01, 0x51, 0x09, 0x06};
  static const uint8_t digits[10][5] = {
    {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}
  };
  static const uint8_t letters[26][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}
  };
  static const uint8_t plus[5] = {0x08,0x08,0x3E,0x08,0x08};
  static const uint8_t minus[5] = {0x08,0x08,0x08,0x08,0x08};
  static const uint8_t dot[5] = {0x00,0x60,0x60,0x00,0x00};
  static const uint8_t colon[5] = {0x00,0x36,0x36,0x00,0x00};
  if ((ch >= '0') && (ch <= '9')) return digits[(unsigned)(ch - '0')];
  if ((ch >= 'A') && (ch <= 'Z')) return letters[(unsigned)(ch - 'A')];
  if ((ch >= 'a') && (ch <= 'z')) return letters[(unsigned)(ch - 'a')];
  if (ch == ' ') return blank;
  if (ch == '+') return plus;
  if (ch == '-') return minus;
  if (ch == '.') return dot;
  if (ch == ':') return colon;
  return unknown;
}

static void PutString(uint8_t page, const char *text)
{
  uint8_t x = 0U;
  while ((*text != '\0') && ((x + 5U) < OLED_WIDTH)) {
    const uint8_t *glyph = Glyph(*text++);
    memcpy(&framebuffer[page][x], glyph, 5U);
    x += 6U;
  }
}

static const char *StateName(AppRunState state)
{
  static const char *const names[] = {"STBY", "LEVEL", "RUN", "DONE", "FAULT"};
  return ((unsigned)state < 5U) ? names[state] : "?";
}

static const char *ModeName(AppMode mode)
{
  static const char *const names[] = {"LINE", "REQ3", "AB", "LAP", "MOVE", "HOLD"};
  return ((unsigned)mode < APP_MODE_COUNT) ? names[mode] : "?";
}

static const char *FaultName(FaultCode fault)
{
  static const char *const names[] = {
    "NONE", "STALL", "VISION", "LINE", "STAGE", "START"
  };
  return ((unsigned)fault < (sizeof(names) / sizeof(names[0])))
         ? names[fault] : "?";
}

static void BuildFrame(uint32_t now_ms)
{
  const AppStatus *application = App_GetStatus();
  const ControlStatus *control = ControlLoops_GetStatus();
  const EncoderSample *actuator_encoder = Encoder_Get(ENCODER_ACTUATOR);
  VisionBallSample ball = {0};
  char line[24];
  memset(framebuffer, 0, sizeof(framebuffer));
  (void)VisionUART_GetSnapshot(&ball);
  bool current_ball = VisionUART_GetCurrentMeasurement(
    &ball, now_ms, APP_VISION_COAST_MS);

  (void)snprintf(line, sizeof(line), "%s U:%u D:%u", StateName(application->state),
                 Button_IsRawPressed(BUTTON_LEVEL_UP) ? 1U : 0U,
                 Button_IsRawPressed(BUTTON_LEVEL_DOWN) ? 1U : 0U);
  PutString(0U, line);
  (void)snprintf(line, sizeof(line), "MODE:%s SEQ:%u", ModeName(application->selected_mode),
                 (unsigned)control->ball_sequence_stage);
  PutString(1U, line);
  (void)snprintf(line, sizeof(line), "ENC:%+7ld", (long)actuator_encoder->total_count);
  PutString(2U, line);
  (void)snprintf(line, sizeof(line), "STEP:%+5ld I:%+4.0f",
                 (long)control->stepper_rate_steps_s,
                 (double)control->ball_integral_steps_s);
  PutString(3U, line);
  if (current_ball) {
    (void)snprintf(line, sizeof(line), "B:%+6.1f V:%+5.0f",
                   (double)ball.position_mm, (double)ball.speed_mm_s);
  } else {
    (void)snprintf(line, sizeof(line), "BALL:INVALID");
  }
  PutString(4U, line);
  (void)snprintf(line, sizeof(line), "TGT:%+6.1f V:%u", (double)control->ball_target_mm,
                 VisionUART_IsFresh(now_ms) ? 1U : 0U);
  PutString(5U, line);
  (void)snprintf(line, sizeof(line), "PID:%+6.1f",
                 (double)control->ball_pid_output_steps_s);
  PutString(6U, line);
  (void)snprintf(line, sizeof(line), "FLT:%s O:%02X", FaultName(application->fault),
                 (unsigned)(oled_address_8bit >> 1));
  PutString(7U, line);
}

static bool SendBlocking(const uint8_t *data, uint16_t length)
{
  return HAL_I2C_Master_Transmit(&hi2c1, oled_address_8bit,
                                 (uint8_t *)data, length, 10U) == HAL_OK;
}

static bool InitializeController(void)
{
  static const uint8_t ssd1306_init[] = {
    0x00,0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,0x20,0x02,
    0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,0xDB,0x40,0xA4,0xA6,0xAF
  };
  static const uint8_t ch1116_init[] = {
    0x00,0xAE,0x02,0x10,0x40,0xB0,0x81,0xCF,0xA1,0xA6,0xA8,0x3F,0xAD,
    0x8B,0x33,0xC8,0xD3,0x00,0xD5,0xC0,0xD9,0x1F,0xDA,0x12,0xDB,0x40,0xAF
  };
  if (oled_address_8bit == (0x3DU << 1)) {
    return SendBlocking(ch1116_init, sizeof(ch1116_init));
  }
  return SendBlocking(ssd1306_init, sizeof(ssd1306_init));
}

static bool ProbeAndInitialize(void)
{
  static const uint8_t addresses[] = {(0x3CU << 1), (0x3DU << 1)};
  for (unsigned i = 0U; i < sizeof(addresses); ++i) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, addresses[i], 2U, 5U) == HAL_OK) {
      oled_address_8bit = addresses[i];
      if (InitializeController()) return true;
    }
  }
  oled_address_8bit = 0U;
  return false;
}

static bool StartCommand(uint8_t page)
{
  command_buffer[0] = 0x00U;
  command_buffer[1] = (uint8_t)(0xB0U + page);
  command_buffer[2] = 0x02U;
  command_buffer[3] = 0x10U;
  transfer_complete = false;
  transfer_error = false;
  if (HAL_I2C_Master_Transmit_IT(&hi2c1, oled_address_8bit,
                                 command_buffer, sizeof(command_buffer)) != HAL_OK) {
    return false;
  }
  tx_state = OLED_TX_WAIT_COMMAND;
  return true;
}

static bool StartData(uint8_t page)
{
  data_buffer[0] = 0x40U;
  memcpy(&data_buffer[1], framebuffer[page], OLED_WIDTH);
  transfer_complete = false;
  transfer_error = false;
  if (HAL_I2C_Master_Transmit_IT(&hi2c1, oled_address_8bit,
                                 data_buffer, sizeof(data_buffer)) != HAL_OK) {
    return false;
  }
  tx_state = OLED_TX_WAIT_DATA;
  return true;
}

void OledDisplay_Init(uint32_t now_ms)
{
  oled_present = false;
  oled_address_8bit = 0U;
  tx_state = OLED_TX_IDLE;
  transfer_complete = false;
  transfer_error = false;
  next_refresh_ms = now_ms;
  next_probe_ms = now_ms + 50U;
}

void OledDisplay_Service(uint32_t now_ms)
{
  if (!oled_present) {
    if ((int32_t)(now_ms - next_probe_ms) < 0) return;
    next_probe_ms = now_ms + OLED_REPROBE_MS;
    oled_present = ProbeAndInitialize();
    if (!oled_present) return;
    next_refresh_ms = now_ms;
  }
  if (transfer_error) {
    transfer_error = false;
    transfer_complete = false;
    tx_state = OLED_TX_IDLE;
    oled_present = false;
    next_probe_ms = now_ms + OLED_REPROBE_MS;
    return;
  }
  if (transfer_complete) {
    transfer_complete = false;
    if (tx_state == OLED_TX_WAIT_COMMAND) {
      if (!StartData(current_page)) transfer_error = true;
      return;
    }
    if (tx_state == OLED_TX_WAIT_DATA) {
      ++current_page;
      if (current_page < OLED_PAGES) {
        if (!StartCommand(current_page)) transfer_error = true;
      } else {
        tx_state = OLED_TX_IDLE;
      }
      return;
    }
  }
  if ((tx_state == OLED_TX_IDLE) &&
      ((int32_t)(now_ms - next_refresh_ms) >= 0)) {
    next_refresh_ms = now_ms + OLED_REFRESH_MS;
    BuildFrame(now_ms);
    current_page = 0U;
    if (!StartCommand(current_page)) transfer_error = true;
  }
}

bool OledDisplay_IsPresent(void)
{
  return oled_present;
}

uint8_t OledDisplay_GetAddress(void)
{
  return oled_address_8bit >> 1;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) transfer_complete = true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1) transfer_error = true;
}
