#include "telemetry.h"
#include "app.h"
#include "control_loops.h"
#include "encoder.h"
#include "line_sensor.h"
#include "sa100.h"
#include "usart.h"
#include "vision_uart.h"
#include <stdio.h>

void Telemetry_Update(uint32_t now_ms)
{
  static char buffer[192];
  const AppStatus *app = App_GetStatus();
  const ControlStatus *control = ControlLoops_GetStatus();
  const EncoderSample *left = Encoder_Get(ENCODER_LEFT);
  const EncoderSample *right = Encoder_Get(ENCODER_RIGHT);
  const Sa100Sample *angle = SA100_Get();
  const VisionBallSample *ball = VisionUART_Get();
  const LineSensorSample *line = LineSensor_Get();
  int length;

  if (hlpuart1.gState != HAL_UART_STATE_READY) return;
  length = snprintf(buffer, sizeof(buffer),
    "t=%lu,state=%u,mode=%u,fault=%u,vl=%d,vr=%d,pwl=%d,pwr=%d,ang=%d,ball=%d,line=%d,mask=%02X\r\n",
    (unsigned long)now_ms, (unsigned)app->state, (unsigned)app->selected_mode,
    (unsigned)app->fault, (int)left->speed_mm_s, (int)right->speed_mm_s,
    control->left_pwm, control->right_pwm, (int)(angle->beam_angle_deg * 100.0f),
    (int)(ball->position_mm * 10.0f), (int)(line->error * 1000.0f),
    line->active_mask);
  if (length > 0) {
    if (length >= (int)sizeof(buffer)) length = (int)sizeof(buffer) - 1;
    (void)HAL_UART_Transmit_IT(&hlpuart1, (uint8_t *)buffer, (uint16_t)length);
  }
}
