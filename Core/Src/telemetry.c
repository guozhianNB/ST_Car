#include "telemetry.h"
#include "app.h"
#include "button.h"
#include "control_loops.h"
#include "encoder.h"
#include "line_sensor.h"
#include "oled_display.h"
#include "usart.h"
#include "vision_uart.h"
#include <stdio.h>

void Telemetry_Update(uint32_t now_ms)
{
  static char buffer[256];
  const AppStatus *app = App_GetStatus();
  const ControlStatus *control = ControlLoops_GetStatus();
  const EncoderSample *left = Encoder_Get(ENCODER_LEFT);
  const EncoderSample *right = Encoder_Get(ENCODER_RIGHT);
  const EncoderSample *actuator = Encoder_Get(ENCODER_ACTUATOR);
  VisionBallSample ball = {0};
  const LineSensorSample *line = LineSensor_Get();
  int length;

  (void)VisionUART_GetSnapshot(&ball);
  if (hlpuart1.gState != HAL_UART_STATE_READY) return;
  length = snprintf(buffer, sizeof(buffer),
    "t=%lu,state=%u,mode=%u,fault=%u,seq=%u,vl=%d,vr=%d,pwl=%d,pwr=%d,"
    "actenc=%ld,dactenc=%ld,srate=%ld,starget=%ld,pidrate=%d,pref=%d,ppos=%d,bi=%d,ball=%d,vball=%d,bref=%d,line=%d,mask=%02X,"
    "bu=%u,bd=%u,oled=%02X\r\n",
    (unsigned long)now_ms, (unsigned)app->state, (unsigned)app->selected_mode,
    (unsigned)app->fault, (unsigned)control->ball_sequence_stage,
    (int)left->speed_mm_s, (int)right->speed_mm_s,
    control->left_pwm, control->right_pwm,
    (long)actuator->total_count, (long)actuator->delta_count,
    (long)control->stepper_rate_steps_s,
    (long)control->stepper_target_rate_steps_s,
    (int)control->ball_pid_output_steps_s,
    (int)control->ball_target_position_steps,
    (int)control->ball_command_position_steps,
    (int)control->ball_integral_steps_s,
    (int)(ball.position_mm * 10.0f), (int)(ball.speed_mm_s * 10.0f),
    (int)(control->ball_target_mm * 10.0f),
    (int)(line->error * 1000.0f), line->active_mask,
    Button_IsRawPressed(BUTTON_LEVEL_UP) ? 1U : 0U,
    Button_IsRawPressed(BUTTON_LEVEL_DOWN) ? 1U : 0U,
    (unsigned)OledDisplay_GetAddress());
  if (length > 0) {
    if (length >= (int)sizeof(buffer)) length = (int)sizeof(buffer) - 1;
    (void)HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)buffer, (uint16_t)length);
  }
}
