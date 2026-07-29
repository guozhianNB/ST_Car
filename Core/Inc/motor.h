#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
  MOTOR_LEFT = 0,
  MOTOR_RIGHT,
  MOTOR_BEAM,
  MOTOR_COUNT
} MotorId;

void Motor_Init(void);
void Motor_EnableChassis(bool enable);
void Motor_EnableBeam(bool enable);
void Motor_Set(MotorId id, int16_t command);
int16_t Motor_GetCommand(MotorId id);
void Motor_Brake(MotorId id);
void Motor_Coast(MotorId id);
void Motor_EmergencyStop(void);

/* Compatibility API for early two-wheel tests. */
void Motor_SetSpeed(int speed_left, int speed_right);

#endif /* MOTOR_H */
