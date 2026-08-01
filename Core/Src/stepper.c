#include "stepper.h"
#include "app_config.h"
#include "main.h"
#include "tim.h"

static StepperStatus status;
static int8_t physical_direction;
static uint32_t direction_ready_ms;
static bool pulse_running;

#if (APP_STEPPER_COMMAND_STEPS_PER_REV != \
     (APP_STEPPER_FULL_STEPS_PER_REV * APP_STEPPER_MICROSTEPS))
#error "Stepper command steps/rev must equal full steps/rev times microsteps"
#endif
#if ((APP_STEPPER_TIMER_CLOCK_HZ % APP_STEPPER_TIMER_TICK_HZ) != 0U)
#error "Stepper timer clock must be divisible by the requested timer tick"
#endif
#if (APP_STEPPER_MIN_RATE_STEPS_S <= 0) || \
    (APP_STEPPER_MAX_RATE_STEPS_S < APP_STEPPER_MIN_RATE_STEPS_S)
#error "Invalid stepper rate limits"
#endif

static void WriteEnable(bool enable)
{
  /* With 3V3 common-anode TB6600 wiring, PC5 low activates the ENA
     optocoupler's offline function; PC5 high releases offline and enables. */
  GPIO_PinState level = (enable == (APP_STEPPER_ENABLE_ACTIVE_HIGH != 0))
    ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin, level);
}

static int32_t ClampRate(int32_t rate_steps_s)
{
  if (rate_steps_s > APP_STEPPER_MAX_RATE_STEPS_S) {
    return APP_STEPPER_MAX_RATE_STEPS_S;
  }
  if (rate_steps_s < -APP_STEPPER_MAX_RATE_STEPS_S) {
    return -APP_STEPPER_MAX_RATE_STEPS_S;
  }
  if ((rate_steps_s > -APP_STEPPER_MIN_RATE_STEPS_S) &&
      (rate_steps_s < APP_STEPPER_MIN_RATE_STEPS_S)) {
    return 0;
  }
  return rate_steps_s;
}

static void StopPulseTimer(void)
{
  if (pulse_running) {
    if (HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    pulse_running = false;
  }
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COUNTER(&htim15, 0U);
  status.applied_rate_steps_s = 0;
}

static void StartPulseTimer(uint32_t rate_steps_s)
{
  uint32_t period_ticks;

  period_ticks = (APP_STEPPER_TIMER_TICK_HZ + (rate_steps_s / 2U)) /
                 rate_steps_s;
  if (period_ticks < 2U) period_ticks = 2U;
  if (period_ticks > 65536U) period_ticks = 65536U;

  __HAL_TIM_SET_AUTORELOAD(&htim15, period_ticks - 1U);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, period_ticks / 2U);
  if (pulse_running) {
    if (__HAL_TIM_GET_COUNTER(&htim15) >= period_ticks) {
      __HAL_TIM_SET_COUNTER(&htim15, 0U);
    }
    return;
  }
  __HAL_TIM_SET_COUNTER(&htim15, 0U);
  if (HAL_TIM_GenerateEvent(&htim15, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  pulse_running = true;
}

void Stepper_Init(void)
{
  status.enabled = false;
  status.requested_rate_steps_s = 0;
  status.applied_rate_steps_s = 0;
  physical_direction = 0;
  direction_ready_ms = 0U;
  pulse_running = false;
  WriteEnable(false);
  HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEGACY_BEAM_STBY_GPIO_Port, LEGACY_BEAM_STBY_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEGACY_BEAM_PWM_GPIO_Port, LEGACY_BEAM_PWM_Pin,
                    GPIO_PIN_RESET);
  StopPulseTimer();
}

void Stepper_Enable(bool enable)
{
  if (!enable) {
    status.requested_rate_steps_s = 0;
    StopPulseTimer();
    WriteEnable(false);
    status.enabled = false;
    return;
  }

  StopPulseTimer();
  WriteEnable(true);
  status.enabled = true;
}

void Stepper_SetRate(int32_t signed_rate_steps_s, uint32_t now_ms)
{
  int32_t clamped = ClampRate(signed_rate_steps_s);
  int8_t requested_direction;
  uint32_t magnitude;

  status.requested_rate_steps_s = clamped;
  if (!status.enabled || (clamped == 0)) {
    StopPulseTimer();
    return;
  }

  requested_direction = (clamped > 0) ? 1 : -1;
  requested_direction = (int8_t)(requested_direction *
                                  APP_STEPPER_DIRECTION_SIGN);
  if (requested_direction != physical_direction) {
    StopPulseTimer();
    HAL_GPIO_WritePin(STEPPER_DIR_GPIO_Port, STEPPER_DIR_Pin,
                      (requested_direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    physical_direction = requested_direction;
    direction_ready_ms = now_ms + APP_STEPPER_DIRECTION_SETUP_MS;
    return;
  }
  if ((int32_t)(now_ms - direction_ready_ms) < 0) {
    StopPulseTimer();
    return;
  }

  magnitude = (uint32_t)((clamped < 0) ? -clamped : clamped);
  if (pulse_running && (status.applied_rate_steps_s == clamped)) return;
  StartPulseTimer(magnitude);
  status.applied_rate_steps_s = clamped;
}

void Stepper_Stop(void)
{
  status.requested_rate_steps_s = 0;
  StopPulseTimer();
}

const StepperStatus *Stepper_GetStatus(void)
{
  return &status;
}
