#include "app.h"
#include "app_config.h"
#include "bench_debug.h"
#include "button.h"
#include "control_loops.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "oled_display.h"
#include "stepper.h"
#include "telemetry.h"
#include "tim.h"
#include "usart.h"
#include "vision_uart.h"
#include <math.h>

static AppStatus app;
static volatile uint8_t start_event;
static uint32_t last_button_ms;
static uint32_t run_start_ms;
static uint32_t stage_start_ms;
static uint32_t settle_start_ms;
static uint32_t settle_last_measurement_ms;
static uint32_t settle_accumulated_ms;
static uint32_t cross_start_ms;
static uint32_t fast_tick_ms;
static uint32_t line_tick_ms;
static uint32_t state_tick_ms;
static uint32_t telemetry_tick_ms;
static int64_t lap_left_start;
static int64_t lap_right_start;
static bool lap_left_start_line;
static bool lap_armed;
static bool passing_finish_line;
static int64_t finish_left_start;
static int64_t finish_right_start;
static uint32_t centering_start_ms;
static uint32_t centering_last_measurement_ms;
static uint32_t centering_accumulated_ms;
static bool centering_control_active;
static bool centering_fine_active;
static float moving_target_mm = 50.0f;

static bool ModeNeedsLine(AppMode mode)
{
  return mode == APP_MODE_LINE_ONLY || mode == APP_MODE_MOVING_CENTER_AB ||
         mode == APP_MODE_MOVING_CENTER_LAP || mode == APP_MODE_MOVING_TARGET;
}

static bool ModeNeedsBall(AppMode mode)
{
  return mode == APP_MODE_STATIC_BALL || mode == APP_MODE_MOVING_CENTER_AB ||
         mode == APP_MODE_MOVING_CENTER_LAP || mode == APP_MODE_MOVING_TARGET ||
         mode == APP_MODE_HOLD_CURRENT;
}

static bool ModeNeedsActuator(AppMode mode)
{
  return ModeNeedsBall(mode);
}

static float WheelTravelMm(EncoderId id, int64_t start_count)
{
  const EncoderSample *encoder = Encoder_Get(id);
  float cpr = (id == ENCODER_LEFT) ? APP_ENCODER_LEFT_CPR : APP_ENCODER_RIGHT_CPR;
  return ((float)(encoder->total_count - start_count) * APP_WHEEL_DIAMETER_MM *
          3.14159265359f) / cpr;
}

static void App_Stop(AppRunState end_state)
{
  ControlLoops_EnableChassis(false);
  ControlLoops_EnableBall(false);
  ControlLoops_EnableActuator(false);
  app.state = end_state;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                    (end_state == APP_STATE_FINISHED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_FinishStaticBallHold(void)
{
  /* Freeze the scored time but keep the final -50 mm balance target active.
     The finished-state branch continues all actuator/vision safety checks until
     the operator presses the start button again or a fault is detected. */
  ControlLoops_EnableChassis(false);
  app.state = APP_STATE_FINISHED;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}

static void App_Standby(void)
{
  Motor_EmergencyStop();
  ControlLoops_Reset();
  ControlLoops_SetBallGains(APP_BALL_KP, APP_BALL_KI, APP_BALL_KD,
                            APP_BALL_CONTROL_SIGN);
  ControlLoops_SetStepperRateLimit(APP_BALL_RATE_LIMIT_STEPS_S);
  app.state = APP_STATE_STANDBY;
  app.fault = FAULT_NONE;
  app.run_time_ms = 0;
  app.stage = 0;
  settle_start_ms = 0;
  settle_last_measurement_ms = 0U;
  settle_accumulated_ms = 0U;
  centering_last_measurement_ms = 0U;
  centering_accumulated_ms = 0U;
  centering_control_active = false;
  centering_fine_active = false;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
}

static void App_EnterFault(FaultCode fault, uint32_t now_ms)
{
  (void)now_ms;
  if (fault == FAULT_NONE || app.state == APP_STATE_FAULT) return;
  app.fault = fault;
  app.state = APP_STATE_FAULT;
  ControlLoops_EnableChassis(false);
  ControlLoops_EnableBall(false);
  ControlLoops_EnableActuator(false);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}

static void App_Start(uint32_t now_ms)
{
  const EncoderSample *left = Encoder_Get(ENCODER_LEFT);
  const EncoderSample *right = Encoder_Get(ENCODER_RIGHT);
  VisionBallSample ball = {0};
  float expected_start_mm = ((app.selected_mode == APP_MODE_MOVING_TARGET) ||
                             (app.selected_mode == APP_MODE_HOLD_CURRENT))
                            ? moving_target_mm : 0.0f;
  bool current_ball = VisionUART_GetCurrentMeasurement(
    &ball, now_ms, APP_VISION_COAST_MS);
  LineSensor_Update();
  if ((ModeNeedsLine(app.selected_mode) && !LineSensor_Get()->line_found) ||
      (ModeNeedsBall(app.selected_mode) &&
       (!current_ball ||
        fabsf(ball.position_mm - expected_start_mm) > APP_BALL_START_TOLERANCE_MM))) {
    App_EnterFault(FAULT_STARTUP_CHECK, now_ms);
    return;
  }
  ControlLoops_SetBallGains(APP_BALL_KP, APP_BALL_KI, APP_BALL_KD,
                            APP_BALL_CONTROL_SIGN);
  ControlLoops_SetStepperRateLimit(APP_BALL_RATE_LIMIT_STEPS_S);
  ControlLoops_Reset();
  SafetyMonitor_Begin(now_ms);
  run_start_ms = now_ms;
  stage_start_ms = now_ms;
  settle_start_ms = 0;
  settle_last_measurement_ms = 0U;
  settle_accumulated_ms = 0U;
  cross_start_ms = 0;
  app.fault = FAULT_NONE;
  app.stage = 0;
  app.state = APP_STATE_RUNNING;
  app.commanded_ball_target_mm = 0.0f;
  lap_left_start = left->total_count;
  lap_right_start = right->total_count;
  lap_left_start_line = false;
  lap_armed = false;
  passing_finish_line = false;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);

  if (ModeNeedsLine(app.selected_mode)) {
    ControlLoops_SetBaseSpeed(APP_CHASSIS_BASE_SPEED_MM_S);
    ControlLoops_EnableChassis(true);
  }
  if (ModeNeedsActuator(app.selected_mode)) {
    ControlLoops_EnableActuator(true);
  }
  if (ModeNeedsBall(app.selected_mode)) {
    if (app.selected_mode == APP_MODE_STATIC_BALL) app.commanded_ball_target_mm = 50.0f;
    else if ((app.selected_mode == APP_MODE_MOVING_TARGET) ||
             (app.selected_mode == APP_MODE_HOLD_CURRENT))
      app.commanded_ball_target_mm = moving_target_mm;
    ControlLoops_SetBallTarget(app.commanded_ball_target_mm);
    ControlLoops_EnableBall(true);
    if (app.selected_mode == APP_MODE_STATIC_BALL) {
      ControlLoops_StartBallSequence(now_ms);
    }
  }
}

static bool BallSettled(uint32_t now_ms)
{
  VisionBallSample ball = {0};
  if (!VisionUART_GetCurrentMeasurement(&ball, now_ms,
                                        APP_VISION_COAST_MS) ||
      fabsf(ball.position_mm - app.commanded_ball_target_mm) > APP_BALL_SETTLE_ERROR_MM ||
      fabsf(ball.speed_mm_s) > APP_BALL_SETTLE_SPEED_MM_S) {
    settle_start_ms = 0;
    settle_last_measurement_ms = 0U;
    settle_accumulated_ms = 0U;
    return false;
  }
  if (ball.timestamp_ms != settle_last_measurement_ms) {
    if (settle_last_measurement_ms == 0U) {
      settle_start_ms = ball.timestamp_ms;
    } else {
      uint32_t interval_ms = ball.timestamp_ms - settle_last_measurement_ms;
      if (interval_ms > APP_VISION_CONTROL_HOLD_MS) {
        settle_start_ms = ball.timestamp_ms;
        settle_accumulated_ms = 0U;
      } else {
        settle_accumulated_ms += interval_ms;
      }
    }
    settle_last_measurement_ms = ball.timestamp_ms;
  }
  return settle_accumulated_ms >= APP_BALL_SETTLE_TIME_MS;
}

static void UpdateStaticBall(uint32_t now_ms)
{
  const ControlStatus *control = ControlLoops_GetStatus();
  VisionBallSample ball = {0};
  if ((now_ms - run_start_ms) > APP_STATIC_STAGE_TIMEOUT_MS) {
    App_EnterFault(FAULT_STAGE_TIMEOUT, now_ms);
    return;
  }
  if ((app.stage == 0U) &&
      (control->ball_sequence_stage !=
       CONTROL_BALL_SEQUENCE_POSITIVE_CONTROL)) {
    settle_start_ms = 0U;
    settle_last_measurement_ms = 0U;
    settle_accumulated_ms = 0U;
    return;
  }
  if ((app.stage == 1U) &&
      (control->ball_sequence_stage !=
       CONTROL_BALL_SEQUENCE_NEGATIVE_CONTROL)) {
    settle_start_ms = 0U;
    settle_last_measurement_ms = 0U;
    settle_accumulated_ms = 0U;
    return;
  }
  if (app.stage == 0U) {
    /* Requirement 3 says the ball reaches +50 mm and then reverses; only the
       final -50 mm point must remain stable.  Reverse on the first true frame
       inside the +/-10 mm scoring window instead of wasting 250 ms and all
       ball speed at the intermediate point. */
    if (!VisionUART_GetCurrentMeasurement(&ball, now_ms,
                                          APP_VISION_COAST_MS) ||
        fabsf(ball.position_mm - APP_BALL_SEQUENCE_GOAL_MM) >
          APP_BALL_SETTLE_ERROR_MM) {
      return;
    }
    settle_start_ms = 0U;
    settle_last_measurement_ms = 0U;
    settle_accumulated_ms = 0U;
    stage_start_ms = now_ms;
    app.stage = 1U;
    app.commanded_ball_target_mm = -50.0f;
    ControlLoops_StartBallSequenceNegative(now_ms);
  } else {
    if (!BallSettled(now_ms)) return;
    settle_start_ms = 0U;
    settle_last_measurement_ms = 0U;
    settle_accumulated_ms = 0U;
    stage_start_ms = now_ms;
    ControlLoops_FinishBallSequenceHold();
    App_FinishStaticBallHold();
  }
}

static void UpdateLap(uint32_t now_ms)
{
  const LineSensorSample *line = LineSensor_Get();
  float left_mm = fabsf(WheelTravelMm(ENCODER_LEFT, lap_left_start));
  float right_mm = fabsf(WheelTravelMm(ENCODER_RIGHT, lap_right_start));
  float distance_mm = 0.5f * (left_mm + right_mm);

  if (passing_finish_line) {
    float after_left = fabsf(WheelTravelMm(ENCODER_LEFT, finish_left_start));
    float after_right = fabsf(WheelTravelMm(ENCODER_RIGHT, finish_right_start));
    ControlLoops_SetBaseSpeed(APP_CHASSIS_CURVE_SPEED_MM_S);
    if (0.5f * (after_left + after_right) >= APP_AFTER_A_RUN_DISTANCE_MM) {
      App_Stop(APP_STATE_FINISHED);
    }
    return;
  }

  if (!line->cross_line) lap_left_start_line = true;
  if (lap_left_start_line && distance_mm >= APP_LAP_ARM_DISTANCE_MM) lap_armed = true;
  if (lap_armed && line->cross_line) {
    /* The line-only item stops at A. Balance-lap items must first pass A. */
    if (app.selected_mode == APP_MODE_LINE_ONLY) ControlLoops_SetBaseSpeed(0.0f);
    if (cross_start_ms == 0U) cross_start_ms = now_ms;
    if ((now_ms - cross_start_ms) >= APP_LINE_CROSS_CONFIRM_MS) {
      if (app.selected_mode == APP_MODE_LINE_ONLY) {
        App_Stop(APP_STATE_FINISHED);
      } else {
        passing_finish_line = true;
        finish_left_start = Encoder_Get(ENCODER_LEFT)->total_count;
        finish_right_start = Encoder_Get(ENCODER_RIGHT)->total_count;
      }
    }
  } else {
    cross_start_ms = 0;
    ControlLoops_SetBaseSpeed(APP_CHASSIS_BASE_SPEED_MM_S);
  }
}

static void UpdateAB(void)
{
  float left_mm = fabsf(WheelTravelMm(ENCODER_LEFT, lap_left_start));
  float right_mm = fabsf(WheelTravelMm(ENCODER_RIGHT, lap_right_start));
  if (0.5f * (left_mm + right_mm) >= APP_AB_FINISH_DISTANCE_MM) {
    App_Stop(APP_STATE_FINISHED);
  }
}

static void UpdateState(uint32_t now_ms)
{
  FaultCode fault;
  if (app.state == APP_STATE_RUNNING) {
    if (!passing_finish_line) app.run_time_ms = now_ms - run_start_ms;
    fault = SafetyMonitor_Update(now_ms, ModeNeedsLine(app.selected_mode),
                                 ModeNeedsBall(app.selected_mode),
                                 ModeNeedsActuator(app.selected_mode));
    if (fault != FAULT_NONE) {
      App_EnterFault(fault, now_ms);
      return;
    }
    if (app.selected_mode == APP_MODE_STATIC_BALL) UpdateStaticBall(now_ms);
    else if (app.selected_mode == APP_MODE_MOVING_CENTER_AB) UpdateAB();
    else if (ModeNeedsLine(app.selected_mode)) UpdateLap(now_ms);
  } else if (app.state == APP_STATE_CENTERING) {
    VisionBallSample ball = {0};
    bool current = VisionUART_GetCurrentMeasurement(
      &ball, now_ms, APP_VISION_COAST_MS);
    if ((now_ms - centering_start_ms) >
        APP_REQUIREMENT3_CENTER_TIMEOUT_MS) {
      App_EnterFault(FAULT_STARTUP_CHECK, now_ms);
      return;
    }
    if (!centering_control_active) {
      if (!current) return;
      ControlLoops_Reset();
      ControlLoops_SetBallGains(APP_BALL_KP, APP_BALL_KI,
                                APP_REQUIREMENT3_CENTER_KD,
                                APP_BALL_CONTROL_SIGN);
      /* Recovery may begin with a large learned tube pose.  The 2000-step/s
         release catch used for ordinary setpoint changes overshoots center in
         that condition, so centering stays on the normal 400-step/s shaper. */
      ControlLoops_EnableStaticReleaseCatch(false);
      ControlLoops_SetStepperRateLimit(APP_BALL_RATE_LIMIT_STEPS_S);
      ControlLoops_SetBallTarget(0.0f);
      ControlLoops_EnableActuator(true);
      ControlLoops_EnableBall(true);
      SafetyMonitor_Begin(now_ms);
      centering_control_active = true;
    }
    fault = SafetyMonitor_Update(now_ms, false, true, true);
    if (fault != FAULT_NONE) {
      App_EnterFault(fault, now_ms);
      return;
    }
    if (!centering_fine_active &&
        (fabsf(ball.position_mm) <= APP_REQUIREMENT3_FINE_ENTER_MM)) {
      ControlLoops_SetBallGains(APP_BALL_KP, APP_REQUIREMENT3_FINE_KI,
                                APP_REQUIREMENT3_FINE_KD,
                                APP_BALL_CONTROL_SIGN);
      ControlLoops_EnableStaticReleaseCatch(false);
      ControlLoops_SetStepperRateLimit(
        APP_REQUIREMENT3_FINE_RATE_STEPS_S);
      centering_fine_active = true;
    } else if (centering_fine_active &&
               (fabsf(ball.position_mm) >=
                APP_REQUIREMENT3_FINE_EXIT_MM)) {
      ControlLoops_SetBallGains(APP_BALL_KP, APP_BALL_KI,
                                APP_REQUIREMENT3_CENTER_KD,
                                APP_BALL_CONTROL_SIGN);
      ControlLoops_EnableStaticReleaseCatch(false);
      ControlLoops_SetStepperRateLimit(APP_BALL_RATE_LIMIT_STEPS_S);
      centering_fine_active = false;
    }
    if (!current ||
        (fabsf(ball.position_mm) > APP_REQUIREMENT3_CENTER_ERROR_MM) ||
        (fabsf(ball.speed_mm_s) > APP_REQUIREMENT3_CENTER_SPEED_MM_S)) {
      centering_last_measurement_ms = 0U;
      centering_accumulated_ms = 0U;
      return;
    }
    if (ball.timestamp_ms != centering_last_measurement_ms) {
      if (centering_last_measurement_ms != 0U) {
        uint32_t interval_ms =
          ball.timestamp_ms - centering_last_measurement_ms;
        if (interval_ms > APP_VISION_CONTROL_HOLD_MS) {
          centering_accumulated_ms = 0U;
        } else {
          centering_accumulated_ms += interval_ms;
        }
      }
      centering_last_measurement_ms = ball.timestamp_ms;
    }
    if (centering_accumulated_ms >= APP_REQUIREMENT3_CENTER_SETTLE_MS) {
      ControlLoops_EnableBall(false);
      ControlLoops_EnableActuator(false);
      app.state = APP_STATE_STANDBY;
      App_Start(now_ms);
    }
  } else if ((app.state == APP_STATE_FINISHED) &&
             (app.selected_mode == APP_MODE_STATIC_BALL)) {
    /* Requirement 3 explicitly requires remaining stable at -50 mm after the
       timed transfer.  Do not drop safety monitoring while holding. */
    fault = SafetyMonitor_Update(now_ms, false, true, true);
    if (fault != FAULT_NONE) App_EnterFault(fault, now_ms);
  }
}

static void StartRequirement3Centering(uint32_t now_ms)
{
  App_Standby();
  app.selected_mode = APP_MODE_STATIC_BALL;
  app.state = APP_STATE_CENTERING;
  app.commanded_ball_target_mm = 0.0f;
  centering_start_ms = now_ms;
  centering_last_measurement_ms = 0U;
  centering_accumulated_ms = 0U;
  centering_control_active = false;
  centering_fine_active = false;
}

void App_RequestRequirement3(void)
{
  uint32_t now_ms = HAL_GetTick();
  if ((app.state == APP_STATE_CENTERING) ||
      (app.state == APP_STATE_RUNNING)) {
    App_Standby();
    return;
  }
  StartRequirement3Centering(now_ms);
}

static void ProcessEvents(uint32_t now_ms)
{
  if ((now_ms - last_button_ms) < 60U) return;
  if (start_event) {
    start_event = 0;
    last_button_ms = now_ms;
    App_RequestRequirement3();
  }
}

static void ProcessPanelButtons(uint32_t now_ms)
{
  ButtonEvent up = Button_TakeEvent(BUTTON_LEVEL_UP);
  ButtonEvent down = Button_TakeEvent(BUTTON_LEVEL_DOWN);
  if ((up == BUTTON_EVENT_SHORT) || (down == BUTTON_EVENT_SHORT)) {
    last_button_ms = now_ms;
    App_RequestRequirement3();
  }
}

void App_Init(void)
{
  uint32_t now = HAL_GetTick();
  Motor_Init();
  Stepper_Init();
  Encoder_Init();
  VisionUART_Init();
  Button_Init(now);
  LineSensor_Update();
  ControlLoops_Init();
  OledDisplay_Init(now);
  SafetyMonitor_Begin(now);
  app.selected_mode = APP_MODE_STATIC_BALL;
  fast_tick_ms = now;
  line_tick_ms = now;
  state_tick_ms = now;
  telemetry_tick_ms = now;
  last_button_ms = now - 100U;
  App_Standby();
  BenchDebug_Init(now);
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();
  VisionUART_Service();
  Button_Update(now);
  OledDisplay_Service(now);
  if (BenchDebug_Run(now)) {
    Button_DiscardEvents();
    return;
  }
  ProcessPanelButtons(now);
  ProcessEvents(now);

  if ((now - fast_tick_ms) >= APP_CONTROL_FAST_MS) {
    uint32_t elapsed = now - fast_tick_ms;
    fast_tick_ms = now;
    Encoder_Update(elapsed);
    ControlLoops_FastUpdate(elapsed);
  }
  if ((now - line_tick_ms) >= APP_CONTROL_LINE_MS) {
    uint32_t elapsed = now - line_tick_ms;
    line_tick_ms = now;
    LineSensor_Update();
    ControlLoops_LineUpdate(elapsed);
  }
  if ((now - state_tick_ms) >= APP_STATE_UPDATE_MS) {
    state_tick_ms = now;
    UpdateState(now);
  }
  if ((now - telemetry_tick_ms) >= APP_TELEMETRY_MS) {
    telemetry_tick_ms = now;
    /* Keep the bench command line quiet while the normal app is in standby. */
    if (!APP_ENABLE_BENCH_DEBUG || (app.state != APP_STATE_STANDBY)) {
      Telemetry_Update(now);
    }
  }
}

void App_SetMovingTarget(float target_mm)
{
  if (target_mm > APP_BALL_TARGET_LIMIT_MM) target_mm = APP_BALL_TARGET_LIMIT_MM;
  if (target_mm < -APP_BALL_TARGET_LIMIT_MM) target_mm = -APP_BALL_TARGET_LIMIT_MM;
  moving_target_mm = target_mm;
}

void App_SelectMode(AppMode mode)
{
  if ((app.state == APP_STATE_STANDBY) && ((unsigned)mode < APP_MODE_COUNT)) {
    app.selected_mode = mode;
  }
}

const AppStatus *App_GetStatus(void)
{
  return &app;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == START_BUTTON_Pin) {
    if (BenchDebug_IsActive()) BenchDebug_RequestEmergencyStop();
    else start_event = 1;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == UART4) VisionUART_RxEventCallback(size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) VisionUART_ErrorCallback();
  else if (huart->Instance == LPUART1) BenchDebug_UartErrorCallback();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1) BenchDebug_UartRxCpltCallback();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == LPUART1) BenchDebug_UartTxCpltCallback();
}
