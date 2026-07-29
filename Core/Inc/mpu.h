#ifndef __MPU_H
#define __MPU_H

#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR 0x68               // MPU6050 I2C地址 (AD0引脚接地时为0x68, 接VCC时为0x69)
#define WHO_AM_I_REG 0x75               //  WHO_AM_I寄存器地址
#define PWR_MGMT_1_REG 0x6B             // 电源管理寄存器地址
#define SMPLRT_DIV_REG 0x19             //  采样率分频器寄存器地址
#define ACCEL_CONFIG_REG 0x1C           // 加速度计配置寄存器
#define GYRO_CONFIG_REG 0x1B            // 陀螺仪配置寄存器
#define MPU_CFG_REG 0x1A                // 数字低通滤波器配置寄存器
#define MPU_SAMPLE_RATE_REG 0x19             // 采样率分频器寄存器


#define DEG2RAD 0.0174532925f               // 角度转弧度的转换因子
#define RAD2DEG 57.2957795f             // 弧度转角度的转换因子 
#define GRAVITY_MS2 9.80665f            // 标准重力加速度，单位为m/s²
#define COMPLEMENTARY_ALPHA 0.98f
#define ACC_LPF_ALPHA 0.2f
#define COMPLEMENTARY_ALPHA_STABLE 0.90f
#define QUICK_SETTLE_GYRO_DPS 3.0f
#define QUICK_SETTLE_ACC_TOL_G 0.08f
#define QUICK_SETTLE_BLEND 0.25f
#define STATIONARY_GYRO_DPS 1.2f
#define STATIONARY_ACC_NORM_TOL_G 0.035f
#define STATIONARY_LINACC_MS2 0.15f
#define STATIONARY_CONFIRM_COUNT 8
#define ACC_BIAS_TRIM_ALPHA 0.01f


bool MPU6050_Init(void);
void MPU_Set_LPF(uint16_t lpf);
void MPU_Set_Rate(uint16_t rate);
void MPU_Write_Byte(uint8_t reg, uint8_t data);
void MPU6050_ReadRawData(int16_t* AccelData, int16_t* GyroData);
void MPU6050_GetAccelData(float* AccelData);
void MPU6050_GetGyroData(float* GyroData);


void MPU6050_CalibrateIMU(uint16_t sampleCount, uint16_t sampleDelayMs, float* EulerDeg);
void MPU6050_CalibrateGyroBias(uint16_t sampleCount, uint16_t sampleDelayMs);
void UpdateAttitude(float dt_s, float* EulerDeg);
void Only_for_Roll(float dt_s, float* EulerDeg);
void UpdatePosition(float dt_s, float* EulerDeg, float* Position);
void UpdateAttitudeAndPosition(float dt_s, float* EulerDeg, float* Position);

#endif /* __MPU_H */
