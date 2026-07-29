#include "app.h"
#include "app_config.h"
#include "control_loops.h"
#include "encoder.h"
#include "line_sensor.h"
#include "motor.h"
#include "sa100.h"
#include "telemetry.h"
#include "tim.h"
#include "usart.h"
#include "vision_uart.h"
#include <math.h>

static AppStatus app;
static volatile uint8_t start_event;
static volatile uint8_t mode_event;
static volatile uint8_t home_event;
static uint32_t last_button_ms;
static uint32_t run_start_ms;
static uint32_t stage_start_ms;
static uint32_t settle_start_ms;
static uint32_t cross_start_ms;
static uint32_t fault_start_ms;
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
static bool fault_leveling;
static float moving_target_mm = 50.0f;

static bool ModeNeedsLine(AppMode mode)
{
  return mode == APP_MODE_LINE_ONLY || mode == APP_MODE_MOVING_CENTER_AB ||
         mode == APP_MODE_MOVING_CENTER_LAP || mode == APP_MODE_MOVING_TARGET;
}

static bool ModeNeedsBall(AppMode mode)
{
  return mode == APP_MODE_STATIC_BALL || mode == APP_MODE_MOVING_CENTER_AB ||
         mode == APP_MODE_MOVING_CENTER_LAP || mode == APP_MODE_MOVING_TARGET;
}

static bool ModeNeedsBeam(AppMode mode)
{
  return ModeNeedsBall(mode) || mode == APP_MODE_ANGLE_TEST;
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
  ControlLoops_EnableBeam(false);
  app.state = end_state;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                    (end_state == APP_STATE_FINISHED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_Standby(void)
{
  Motor_EmergencyStop();
  ControlLoops_Reset();
  app.state = APP_STATE_STANDBY;
  app.fault = FAULT_NONE;
  app.run_time_ms = 0;
  app.stage = 0;
  settle_start_ms = 0;
  fault_leveling = false;
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
}

static void App_EnterFault(FaultCode fault, uint32_t now_ms)
{
  if (fault == FAULT_NONE || app.state == APP_STATE_FAULT) return;
  app.fault = fault;
  app.state = APP_STATE_FAULT;
  fault_start_ms = now_ms;
  ControlLoops_EnableChassis(false);
  ControlLoops_EnableBall(false);

  /* Level only when feedback and travel limits remain trustworthy. */
  fault_leveling = SA100_IsFresh(now_ms) &&
                   fault != FAULT_BEAM_ANGLE_LIMIT &&
                   fault != FAULT_BEAM_ENCODER_LIMIT &&
                   fault != FAULT_BEAM_STALL &&
                   fault != FAULT_SA100_TIMEOUT &&
                   fault != FAULT_STARTUP_CHECK;
  if (fault_leveling) {
    ControlLoops_SetDirectBeamTarget(0.0f);
    ControlLoops_EnableBeam(true);
  } else {
    ControlLoops_EnableBeam(false);
  }
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}

static void App_Start(uint32_t now_ms)
{
  const EncoderSample *left = Encoder_Get(ENCODER_LEFT);
  const EncoderSample *right = Encoder_Get(ENCODER_RIGHT);
  const Sa100Sample *angle = SA100_Get();
  const VisionBallSample *ball = VisionUART_Get();
  float expected_start_mm = (app.selected_mode == APP_MODE_MOVING_TARGET)
                            ? moving_target_mm : 0.0f;
  LineSensor_Update();
  if ((ModeNeedsLine(app.selected_mode) && !LineSensor_Get()->line_found) ||
      (ModeNeedsBeam(app.selected_mode) &&
       (!SA100_IsFresh(now_ms) || fabsf(angle->beam_angle_deg) > 0.5f)) ||
      (ModeNeedsBall(app.selected_mode) &&
       (!VisionUART_IsFresh(now_ms) ||
        fabsf(ball->position_mm - expected_start_mm) > APP_BALL_START_TOLERANCE_MM))) {
    App_EnterFault(FAULT_STARTUP_CHECK, now_ms);
    return;
  }
  if (ModeNeedsBeam(app.selected_mode)) Encoder_Reset(ENCODER_BEAM);
  ControlLoops_Reset();
  SafetyMonitor_Begin(now_ms);
  run_start_ms = now_ms;
  stage_start_ms = now_ms;
  settle_start_ms = 0;
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
  if (ModeNeedsBeam(app.selected_mode)) {
    ControlLoops_SetDirectBeamTarget(0.0f);
    ControlLoops_EnableBeam(true);
  }
  if (ModeNeedsBall(app.selected_mode)) {
    if (app.selected_mode == APP_MODE_STATIC_BALL) app.commanded_ball_target_mm = 50.0f;
    else if (app.selected_mode == APP_MODE_MOVING_TARGET)
      app.commanded_ball_target_mm = moving_target_mm;
    ControlLoops_SetBallTarget(app.commanded_ball_target_mm);
    ControlLoops_EnableBall(true);
  }
}

static bool BallSettled(uint32_t now_ms)
{
  const VisionBallSample *ball = VisionUART_Get();
  if (!VisionUART_IsFresh(now_ms) ||
      fabsf(ball->position_mm - app.commanded_ball_target_mm) > APP_BALL_SETTLE_ERROR_MM ||
      fabsf(ball->speed_mm_s) > APP_BALL_SETTLE_SPEED_MM_S) {
    settle_start_ms = 0;
    return false;
  }
  if (settle_start_ms == 0U) settle_start_ms = now_ms;
  return (now_ms - settle_start_ms) >= APP_BALL_SETTLE_TIME_MS;
}

static void UpdateStaticBall(uint32_t now_ms)
{
  if ((now_ms - run_start_ms) > APP_STATIC_STAGE_TIMEOUT_MS) {
    App_EnterFault(FAULT_STAGE_TIMEOUT, now_ms);
    return;
  }
  if (!BallSettled(now_ms)) return;
  settle_start_ms = 0;
  stage_start_ms = now_ms;
  if (app.stage == 0U) {
    app.stage = 1U;
    app.commanded_ball_target_mm = -50.0f;
    ControlLoops_SetBallTarget(app.commanded_ball_target_mm);
  } else {
    App_Stop(APP_STATE_FINISHED);
  }
}

static void UpdateAngleTest(uint32_t now_ms)
{
  static const float targets[] = {0.0f, 1.0f, -1.0f, 2.0f, -2.0f, 0.0f};
  if ((now_ms - stage_start_ms) < 1000U) return;
  stage_start_ms = now_ms;
  ++app.stage;
  if (app.stage >= (sizeof(targets) / sizeof(targets[0]))) {
    App_Stop(APP_STATE_FINISHED);
  } else {
    ControlLoops_SetDirectBeamTarget(targets[app.stage]);
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

static void UpdateFault(uint32_t now_ms)
{
  const Sa100Sample *angle = SA100_Get();
  if (!fault_leveling) return;
  if (!SA100_IsFresh(now_ms) || fabsf(angle->beam_angle_deg) < 0.3f ||
      (now_ms - fault_start_ms) >= 1000U) {
    ControlLoops_EnableBeam(false);
    fault_leveling = false;
  }
}

static void UpdateState(uint32_t now_ms)
{
  FaultCode fault;
  if (app.state == APP_STATE_RUNNING) {
    if (!passing_finish_line) app.run_time_ms = now_ms - run_start_ms;
    fault = SafetyMonitor_Update(now_ms, ModeNeedsLine(app.selected_mode),
                                 ModeNeedsBall(app.selected_mode),
                                 ModeNeedsBeam(app.selected_mode));
    if (fault != FAULT_NONE) {
      App_EnterFault(fault, now_ms);
      return;
    }
    if (app.selected_mode == APP_MODE_STATIC_BALL) UpdateStaticBall(now_ms);
    else if (app.selected_mode == APP_MODE_ANGLE_TEST) UpdateAngleTest(now_ms);
    else if (app.selected_mode == APP_MODE_MOVING_CENTER_AB) UpdateAB();
    else if (ModeNeedsLine(app.selected_mode)) UpdateLap(now_ms);
  } else if (app.state == APP_STATE_FAULT) {
    UpdateFault(now_ms);
  }
}

static void ProcessEvents(uint32_t now_ms)
{
  if (home_event) {
    home_event = 0;
    Encoder_Reset(ENCODER_BEAM);
  }
  if ((now_ms - last_button_ms) < 60U) return;
  if (mode_event) {
    mode_event = 0;
    last_button_ms = now_ms;
    if (app.state == APP_STATE_STANDBY) {
      app.selected_mode = (AppMode)(((unsigned)app.selected_mode + 1U) % APP_MODE_COUNT);
    }
  }
  if (start_event) {
    start_event = 0;
    last_button_ms = now_ms;
    if (app.state == APP_STATE_STANDBY) App_Start(now_ms);
    else if (app.state == APP_STATE_RUNNING) App_Stop(APP_STATE_FINISHED);
    else App_Standby();
  }
}

void App_Init(void)
{
  uint32_t now = HAL_GetTick();
  Motor_Init();
  Encoder_Init();
  SA100_Init();
  VisionUART_Init();
  LineSensor_Update();
  ControlLoops_Init();
  SafetyMonitor_Begin(now);
  app.selected_mode = APP_MODE_LINE_ONLY;
  fast_tick_ms = now;
  line_tick_ms = now;
  state_tick_ms = now;
  telemetry_tick_ms = now;
  last_button_ms = now - 100U;
  App_Standby();
}

void App_Run(void)
{
  uint32_t now = HAL_GetTick();
  VisionUART_Service();
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
    Telemetry_Update(now);
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
  if (pin == START_BUTTON_Pin) start_event = 1;
  else if (pin == MODE_BUTTON_Pin) mode_event = 1;
  else if (pin == BEAM_HOME_Pin) home_event = 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM15) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) {
    SA100_CaptureCallback();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == UART4) VisionUART_RxEventCallback(size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4) VisionUART_ErrorCallback();
}
