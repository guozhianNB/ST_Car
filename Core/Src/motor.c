#include "motor.h"
#include "app_config.h"
#include "tim.h"

typedef struct {
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  uint32_t channel;
  int8_t sign;
  int16_t min_pwm;
  int16_t max_pwm;
} MotorHardware;

static const MotorHardware motor_hw[MOTOR_COUNT] = {
  {MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, MOTOR_L_IN2_GPIO_Port,
   MOTOR_L_IN2_Pin, TIM_CHANNEL_1, APP_MOTOR_LEFT_SIGN,
   APP_MOTOR_LEFT_MIN_PWM, APP_MOTOR_CHASSIS_MAX_PWM},
  {MOTOR_R_IN1_GPIO_Port, MOTOR_R_IN1_Pin, MOTOR_R_IN2_GPIO_Port,
   MOTOR_R_IN2_Pin, TIM_CHANNEL_2, APP_MOTOR_RIGHT_SIGN,
   APP_MOTOR_RIGHT_MIN_PWM, APP_MOTOR_CHASSIS_MAX_PWM},
  {MOTOR_BEAM_IN1_GPIO_Port, MOTOR_BEAM_IN1_Pin, MOTOR_BEAM_IN2_GPIO_Port,
   MOTOR_BEAM_IN2_Pin, TIM_CHANNEL_3, APP_MOTOR_BEAM_SIGN,
   APP_MOTOR_BEAM_MIN_PWM, APP_MOTOR_BEAM_MAX_PWM}
};

static int16_t motor_command[MOTOR_COUNT];

static int16_t LimitCommand(int32_t command, int16_t limit)
{
  if (command > limit) return limit;
  if (command < -limit) return (int16_t)-limit;
  return (int16_t)command;
}

void Motor_Init(void)
{
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  Motor_EmergencyStop();
}

void Motor_EnableChassis(bool enable)
{
  if (!enable) {
    Motor_Coast(MOTOR_LEFT);
    Motor_Coast(MOTOR_RIGHT);
  }
  HAL_GPIO_WritePin(CHASSIS_STBY_GPIO_Port, CHASSIS_STBY_Pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_EnableBeam(bool enable)
{
  if (!enable) Motor_Coast(MOTOR_BEAM);
  HAL_GPIO_WritePin(BEAM_STBY_GPIO_Port, BEAM_STBY_Pin,
                    enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void ApplyCommand(MotorId id, int16_t command, bool apply_min_pwm)
{
  const MotorHardware *hw;
  int16_t magnitude;
  if ((unsigned)id >= MOTOR_COUNT) return;
  hw = &motor_hw[id];
  command = LimitCommand((int32_t)command * hw->sign, hw->max_pwm);
  if (command == 0) {
    Motor_Brake(id);
    return;
  }
  magnitude = (command < 0) ? (int16_t)-command : command;
  if (apply_min_pwm && (magnitude < hw->min_pwm)) magnitude = hw->min_pwm;
  motor_command[id] = (command > 0) ? magnitude : (int16_t)-magnitude;
  if (command > 0) {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
  }
  __HAL_TIM_SET_COMPARE(&htim1, hw->channel,
                        ((uint32_t)magnitude * (htim1.Init.Period + 1U)) /
                        APP_PWM_FULL_SCALE);
}

void Motor_Set(MotorId id, int16_t command)
{
  ApplyCommand(id, command, true);
}

void Motor_SetRaw(MotorId id, int16_t command)
{
  ApplyCommand(id, command, false);
}

int16_t Motor_GetCommand(MotorId id)
{
  return ((unsigned)id < MOTOR_COUNT) ? motor_command[id] : 0;
}

void Motor_Brake(MotorId id)
{
  const MotorHardware *hw;
  if ((unsigned)id >= MOTOR_COUNT) return;
  hw = &motor_hw[id];
  __HAL_TIM_SET_COMPARE(&htim1, hw->channel, 0);
  HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
  motor_command[id] = 0;
}

void Motor_Coast(MotorId id)
{
  const MotorHardware *hw;
  if ((unsigned)id >= MOTOR_COUNT) return;
  hw = &motor_hw[id];
  __HAL_TIM_SET_COMPARE(&htim1, hw->channel, 0);
  HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  motor_command[id] = 0;
}

void Motor_EmergencyStop(void)
{
  Motor_Coast(MOTOR_LEFT);
  Motor_Coast(MOTOR_RIGHT);
  Motor_Coast(MOTOR_BEAM);
  HAL_GPIO_WritePin(CHASSIS_STBY_GPIO_Port, CHASSIS_STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BEAM_STBY_GPIO_Port, BEAM_STBY_Pin, GPIO_PIN_RESET);
}

void Motor_SetSpeed(int speed_left, int speed_right)
{
  Motor_Set(MOTOR_LEFT, LimitCommand(speed_left, APP_PWM_FULL_SCALE));
  Motor_Set(MOTOR_RIGHT, LimitCommand(speed_right, APP_PWM_FULL_SCALE));
}
