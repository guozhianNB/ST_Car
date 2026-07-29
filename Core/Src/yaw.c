#include "yaw.h"

#include "main.h"
#include "mpu.h"

#define YAW_CALIBRATION_SAMPLES 300U
#define YAW_CALIBRATION_DELAY_MS 2U
#define YAW_DEFAULT_DT_S         0.02f
#define YAW_MAX_DT_S             0.20f

static float s_yaw_deg;
static float s_gyro_x_bias_dps;
static uint32_t s_last_tick;
static bool s_ready;

bool Yaw_Init(void)
{
  float gyro_dps[3];
  float gyro_x_sum = 0.0f;
  uint16_t sample;

  s_yaw_deg = 0.0f;
  s_gyro_x_bias_dps = 0.0f;
  s_last_tick = HAL_GetTick();
  s_ready = false;

  if (!MPU6050_Init())
  {
    return false;
  }

  HAL_Delay(100U);

  for (sample = 0U; sample < YAW_CALIBRATION_SAMPLES; ++sample)
  {
    MPU6050_GetGyroData(gyro_dps);
    gyro_x_sum += gyro_dps[0];
    HAL_Delay(YAW_CALIBRATION_DELAY_MS);
  }

  s_gyro_x_bias_dps = gyro_x_sum / (float)YAW_CALIBRATION_SAMPLES;
  s_last_tick = HAL_GetTick();
  s_ready = true;
  return true;
}

float GetYaw(void)
{
  uint32_t now_tick;
  float dt_s;
  float gyro_dps[3];

  if (!s_ready)
  {
    return 0.0f;
  }

  now_tick = HAL_GetTick();
  dt_s = (float)(now_tick - s_last_tick) * 0.001f;
  s_last_tick = now_tick;
  if (dt_s <= 0.0f || dt_s > YAW_MAX_DT_S)
  {
    dt_s = YAW_DEFAULT_DT_S;
  }

  MPU6050_GetGyroData(gyro_dps);
  s_yaw_deg += (gyro_dps[0] - s_gyro_x_bias_dps) * dt_s;
  return s_yaw_deg;
}
