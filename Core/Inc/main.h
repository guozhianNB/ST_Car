#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"

void Error_Handler(void);

/* Motor direction pins: TB6612 IN1/IN2, active high. */
#define MOTOR_L_IN1_Pin              GPIO_PIN_0
#define MOTOR_L_IN1_GPIO_Port        GPIOC
#define MOTOR_L_IN2_Pin              GPIO_PIN_1
#define MOTOR_L_IN2_GPIO_Port        GPIOC
#define MOTOR_R_IN1_Pin              GPIO_PIN_2
#define MOTOR_R_IN1_GPIO_Port        GPIOC
#define MOTOR_R_IN2_Pin              GPIO_PIN_3
#define MOTOR_R_IN2_GPIO_Port        GPIOC
#define MOTOR_BEAM_IN1_Pin           GPIO_PIN_4
#define MOTOR_BEAM_IN1_GPIO_Port     GPIOC
#define MOTOR_BEAM_IN2_Pin           GPIO_PIN_5
#define MOTOR_BEAM_IN2_GPIO_Port     GPIOC

/* Two TB6612 boards are enabled separately and default to disabled. */
#define CHASSIS_STBY_Pin             GPIO_PIN_0
#define CHASSIS_STBY_GPIO_Port       GPIOB
#define BEAM_STBY_Pin                GPIO_PIN_1
#define BEAM_STBY_GPIO_Port          GPIOB

/* Eight digital line sensors, ordered from the far left to the far right. */
#define LINE_0_Pin                   GPIO_PIN_10
#define LINE_0_GPIO_Port             GPIOB
#define LINE_1_Pin                   GPIO_PIN_11
#define LINE_1_GPIO_Port             GPIOB
#define LINE_2_Pin                   GPIO_PIN_12
#define LINE_2_GPIO_Port             GPIOB
#define LINE_3_Pin                   GPIO_PIN_13
#define LINE_3_GPIO_Port             GPIOB
#define LINE_4_Pin                   GPIO_PIN_15
#define LINE_4_GPIO_Port             GPIOB
#define LINE_5_Pin                   GPIO_PIN_6
#define LINE_5_GPIO_Port             GPIOC
#define LINE_6_Pin                   GPIO_PIN_7
#define LINE_6_GPIO_Port             GPIOC
#define LINE_7_Pin                   GPIO_PIN_12
#define LINE_7_GPIO_Port             GPIOC

/* NUCLEO PC13 button is active high; MODE is active low. */
#define START_BUTTON_Pin             GPIO_PIN_13
#define START_BUTTON_GPIO_Port       GPIOC
#define START_BUTTON_EXTI_IRQn       EXTI15_10_IRQn
#define MODE_BUTTON_Pin              GPIO_PIN_4
#define MODE_BUTTON_GPIO_Port        GPIOA
#define MODE_BUTTON_EXTI_IRQn        EXTI4_IRQn
/* NUCLEO LD2, active high. */
#define STATUS_LED_Pin               GPIO_PIN_5
#define STATUS_LED_GPIO_Port         GPIOA

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
