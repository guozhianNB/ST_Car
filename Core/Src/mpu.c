#include "mpu.h"
#include "i2c_soft.h"
#include "main.h"
#include <stdint.h>
#include <math.h>

/*
使用说明
1. 调用MPU6050_Init()函数初始化MPU6050传感器。
2. 调用MPU6050_CalibrateIMU(样本数量, 采样间隔, 姿态角数组)函数进行陀螺仪和加速度计的校准，获取初始姿态角,并初始化姿态角数组。
3. 在主循环中定期调用UpdateAttitudeAndPosition(dt,姿态角数组,位置数组)函数，传入时间间隔、姿态角数组和位置数组，函数会更新姿态角和位置数据。
4. 姿态角以度为单位，位置以米为单位。可以根据需要将这些数据发送到显示屏或其他设备进行显示或进一步处理。
 */



bool MPU6050_Init(void)
{
    uint8_t check;
    uint8_t data;

  /* MPU6050 wiring: PC8=SCL, PC9=SDA. */
  I2C_Soft_Init(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9);

    // 检查设备ID WHO_AM_I
  if (I2C_Soft_ReadBytes(MPU6050_ADDR, WHO_AM_I_REG, &check, 1) != 0U ||
      check != MPU6050_ADDR)
  {
    return false;
  }
        // Power management register 0X6B we should write all 0's to wake the sensor up
        data = 0;
        I2C_Soft_WriteBytes(MPU6050_ADDR, PWR_MGMT_1_REG, &data, 1);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        data = 0x07;
        I2C_Soft_WriteBytes(MPU6050_ADDR, SMPLRT_DIV_REG, &data, 1);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0 => No self test
        // AFS_SEL=0 => ±2g
        data = 0x00;
        I2C_Soft_WriteBytes(MPU6050_ADDR, ACCEL_CONFIG_REG, &data, 1);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0 => No self test
        // FS_SEL=0 => ±250 °/s
        data = 0x00;
        I2C_Soft_WriteBytes(MPU6050_ADDR, GYRO_CONFIG_REG, &data, 1);

        //设置采样率
        MPU_Set_Rate(50);

  return true;
}

/**********************************************
函数名称：MPU_Set_LPF
函数功能：设置MPU6050的数字低通滤波器
函数参数：lpf:数字低通滤波频率(Hz)
函数返回值：无
**********************************************/
void MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}

/**********************************************
函数名称：MPU_Set_Rate
函数功能：设置MPU6050的采样率(假定Fs=1KHz)
函数参数：rate:4~1000(Hz)  初始化中rate取50
函数返回值：无
**********************************************/
void MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	MPU_Set_LPF(rate/2);											//自动设置LPF为采样率的一半
}

void MPU_Write_Byte(uint8_t reg, uint8_t data)
{
  I2C_Soft_WriteBytes(MPU6050_ADDR, reg, &data, 1);
}

void MPU6050_ReadRawData(int16_t* AccelData, int16_t* GyroData)
{
  uint8_t Rec_Data[14] = {0};
  uint8_t status;

    // Read 14 bytes of data starting from ACCEL_XOUT_H register
  status = I2C_Soft_ReadBytes(MPU6050_ADDR, 0x3B, Rec_Data, 14);
  if (status != 0)
  {
    if (AccelData != NULL)
    {
      AccelData[0] = 0;
      AccelData[1] = 0;
      AccelData[2] = 0;
    }
    if (GyroData != NULL)
    {
      GyroData[0] = 0;
      GyroData[1] = 0;
      GyroData[2] = 0;
    }
    return;
  }

    // Convert the data
    if (AccelData != NULL) {
        AccelData[0] = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);   // Accel X-axis
        AccelData[1] = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);   // Accel Y-axis
        AccelData[2] = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);   // Accel Z-axis
    }

    if (GyroData != NULL) {
        GyroData[0] = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);    // Gyro X-axis
        GyroData[1] = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);  // Gyro Y-axis
        GyroData[2] = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);  // Gyro Z-axis
    }
}

void MPU6050_GetAccelData(float* AccelData)
{
    int16_t RawData[3];
    MPU6050_ReadRawData(RawData, NULL);
    AccelData[0] = RawData[0] / 16384.0; // Convert to g
    AccelData[1] = RawData[1] / 16384.0; // Convert to g
    AccelData[2] = RawData[2] / 16384.0; // Convert to g
}
void MPU6050_GetGyroData(float* GyroData)
{
    int16_t RawData[3];
    MPU6050_ReadRawData(NULL, RawData);
    GyroData[0] = RawData[0] / 131.0; // Convert to °/s
    GyroData[1] = RawData[1] / 131.0; // Convert to °/s
    GyroData[2] = RawData[2] / 131.0; // Convert to °/s
}





/***********************************************************
函数解算部分--基于互补滤波的姿态解算和简单的零速更新位置解算
************************************************************/


static float AccelData[3];
static float GyroData[3];
static float GyroBias[3] = {0.0f, 0.0f, 0.0f};
static float AccelBias[3] = {0.0f, 0.0f, 0.0f};
static float Velocity[3] = {0.0f, 0.0f, 0.0f};
static float LinearAccelWorldLPF[3] = {0.0f, 0.0f, 0.0f};
static float LastAccelBodyG[3] = {0.0f, 0.0f, 0.0f};
static float LastGyroDps[3] = {0.0f, 0.0f, 0.0f};
static float LastLinearAccelWorld[3] = {0.0f, 0.0f, 0.0f};
static float LastAccelNormG = 1.0f;

static uint16_t StationaryCounter = 0;


void MPU6050_CalibrateIMU(uint16_t sampleCount, uint16_t sampleDelayMs, float* EulerDeg)
{
  float gyroSum[3] = {0.0f, 0.0f, 0.0f};
  float accSum[3] = {0.0f, 0.0f, 0.0f};
  float axAvg, ayAvg, azAvg;
  uint16_t i;

  for (i = 0; i < sampleCount; i++)
  {
    MPU6050_GetAccelData(AccelData);
    MPU6050_GetGyroData(GyroData);
    accSum[0] += AccelData[0];
    accSum[1] += AccelData[1];
    accSum[2] += AccelData[2];
    gyroSum[0] += GyroData[0];
    gyroSum[1] += GyroData[1];
    gyroSum[2] += GyroData[2];
    HAL_Delay(sampleDelayMs);
  }

  axAvg = accSum[0] / (float)sampleCount;
  ayAvg = accSum[1] / (float)sampleCount;
  azAvg = accSum[2] / (float)sampleCount;

  GyroBias[0] = gyroSum[0] / (float)sampleCount;
  GyroBias[1] = gyroSum[1] / (float)sampleCount;
  GyroBias[2] = gyroSum[2] / (float)sampleCount;

  AccelBias[0] = axAvg;
  AccelBias[1] = ayAvg;
  AccelBias[2] = azAvg - 1.0f;

  axAvg -= AccelBias[0];
  ayAvg -= AccelBias[1];
  azAvg -= AccelBias[2];

  EulerDeg[0] = atan2f(ayAvg, azAvg) * RAD2DEG;
  EulerDeg[1] = atan2f(-axAvg, sqrtf(ayAvg * ayAvg + azAvg * azAvg)) * RAD2DEG;
  EulerDeg[2] = 0.0f;
}

void MPU6050_CalibrateGyroBias(uint16_t sampleCount, uint16_t sampleDelayMs)
{
  float gyroSum[3] = {0.0f, 0.0f, 0.0f};
  uint16_t i;

  if (sampleCount == 0)
  {
    return;
  }

  for (i = 0; i < sampleCount; i++)
  {
    MPU6050_GetGyroData(GyroData);
    gyroSum[0] += GyroData[0];
    gyroSum[1] += GyroData[1];
    gyroSum[2] += GyroData[2];
    HAL_Delay(sampleDelayMs);
  }

  GyroBias[0] = gyroSum[0] / (float)sampleCount;
  GyroBias[1] = gyroSum[1] / (float)sampleCount;
  GyroBias[2] = gyroSum[2] / (float)sampleCount;
}





void UpdateAttitude(float dt_s, float* EulerDeg)
{
  float gx_dps, gy_dps, gz_dps;
  float ax_g, ay_g, az_g;
  float ax_ms2, ay_ms2, az_ms2;
  float rollAccDeg, pitchAccDeg;
  float rollRad, pitchRad, yawRad;
  float sr, cr, sp, cp, sy, cy;
  float axWorld, ayWorld, azWorld;
  float accelNormG;
  float alpha;
  uint8_t isQuickSettleNow;

  if (EulerDeg == NULL)
  {
    return;
  }

  MPU6050_GetAccelData(AccelData);  // 单位: g
  MPU6050_GetGyroData(GyroData);    // 单位: deg/s

  ax_g = AccelData[0] - AccelBias[0];
  ay_g = AccelData[1] - AccelBias[1];
  az_g = AccelData[2] - AccelBias[2];

  gx_dps = GyroData[0] - GyroBias[0];
  gy_dps = GyroData[1] - GyroBias[1];
  gz_dps = GyroData[2] - GyroBias[2];

  LastAccelBodyG[0] = ax_g;
  LastAccelBodyG[1] = ay_g;
  LastAccelBodyG[2] = az_g;
  LastGyroDps[0] = gx_dps;
  LastGyroDps[1] = gy_dps;
  LastGyroDps[2] = gz_dps;

  accelNormG = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  LastAccelNormG = accelNormG;

  rollAccDeg = atan2f(ay_g, az_g) * RAD2DEG;
  pitchAccDeg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD2DEG;

  isQuickSettleNow = (fabsf(gx_dps) < QUICK_SETTLE_GYRO_DPS) &&
                     (fabsf(gy_dps) < QUICK_SETTLE_GYRO_DPS) &&
                     (fabsf(gz_dps) < QUICK_SETTLE_GYRO_DPS) &&
                     (fabsf(accelNormG - 1.0f) < QUICK_SETTLE_ACC_TOL_G);

  alpha = isQuickSettleNow ? COMPLEMENTARY_ALPHA_STABLE : COMPLEMENTARY_ALPHA;

  EulerDeg[0] = alpha * (EulerDeg[0] + gx_dps * dt_s) + (1.0f - alpha) * rollAccDeg;
  EulerDeg[1] = alpha * (EulerDeg[1] + gy_dps * dt_s) + (1.0f - alpha) * pitchAccDeg;
  EulerDeg[2] += gz_dps * dt_s;

  if (isQuickSettleNow)
  {
    EulerDeg[0] += QUICK_SETTLE_BLEND * (rollAccDeg - EulerDeg[0]);
    EulerDeg[1] += QUICK_SETTLE_BLEND * (pitchAccDeg - EulerDeg[1]);
  }

  ax_ms2 = ax_g * GRAVITY_MS2;
  ay_ms2 = ay_g * GRAVITY_MS2;
  az_ms2 = az_g * GRAVITY_MS2;

  rollRad = EulerDeg[0] * DEG2RAD;
  pitchRad = EulerDeg[1] * DEG2RAD;
  yawRad = EulerDeg[2] * DEG2RAD;

  sr = sinf(rollRad);
  cr = cosf(rollRad);
  sp = sinf(pitchRad);
  cp = cosf(pitchRad);
  sy = sinf(yawRad);
  cy = cosf(yawRad);

  axWorld = cy * cp * ax_ms2 + (cy * sp * sr - sy * cr) * ay_ms2 + (cy * sp * cr + sy * sr) * az_ms2;
  ayWorld = sy * cp * ax_ms2 + (sy * sp * sr + cy * cr) * ay_ms2 + (sy * sp * cr - cy * sr) * az_ms2;
  azWorld = -sp * ax_ms2 + cp * sr * ay_ms2 + cp * cr * az_ms2;

  azWorld -= GRAVITY_MS2;

  LinearAccelWorldLPF[0] += ACC_LPF_ALPHA * (axWorld - LinearAccelWorldLPF[0]);
  LinearAccelWorldLPF[1] += ACC_LPF_ALPHA * (ayWorld - LinearAccelWorldLPF[1]);
  LinearAccelWorldLPF[2] += ACC_LPF_ALPHA * (azWorld - LinearAccelWorldLPF[2]);

  axWorld = LinearAccelWorldLPF[0];
  ayWorld = LinearAccelWorldLPF[1];
  azWorld = LinearAccelWorldLPF[2];

  if (fabsf(axWorld) < 0.12f) axWorld = 0.0f;
  if (fabsf(ayWorld) < 0.12f) ayWorld = 0.0f;
  if (fabsf(azWorld) < 0.12f) azWorld = 0.0f;

  LastLinearAccelWorld[0] = axWorld;
  LastLinearAccelWorld[1] = ayWorld;
  LastLinearAccelWorld[2] = azWorld;
}

void UpdatePosition(float dt_s, float* EulerDeg, float* Position)
{
  float axWorld, ayWorld, azWorld;
  float linAccNorm;
  uint8_t isStationaryNow;

  if (Position == NULL)
  {
    return;
  }

  axWorld = LastLinearAccelWorld[0];
  ayWorld = LastLinearAccelWorld[1];
  azWorld = LastLinearAccelWorld[2];

  linAccNorm = sqrtf(axWorld * axWorld + ayWorld * ayWorld + azWorld * azWorld);

  isStationaryNow = (fabsf(LastGyroDps[0]) < STATIONARY_GYRO_DPS) &&
                    (fabsf(LastGyroDps[1]) < STATIONARY_GYRO_DPS) &&
                    (fabsf(LastGyroDps[2]) < STATIONARY_GYRO_DPS) &&
                    (fabsf(LastAccelNormG - 1.0f) < STATIONARY_ACC_NORM_TOL_G) &&
                    (linAccNorm < STATIONARY_LINACC_MS2);

  if (isStationaryNow)
  {
    if (StationaryCounter < 65535)
    {
      StationaryCounter++;
    }
  }
  else
  {
    StationaryCounter = 0;
  }

  Velocity[0] += axWorld * dt_s;
  Velocity[1] += ayWorld * dt_s;
  Velocity[2] += azWorld * dt_s;

  if (StationaryCounter >= STATIONARY_CONFIRM_COUNT)
  {
    Velocity[0] = 0.0f;
    Velocity[1] = 0.0f;
    Velocity[2] = 0.0f;

    LinearAccelWorldLPF[0] = 0.0f;
    LinearAccelWorldLPF[1] = 0.0f;
    LinearAccelWorldLPF[2] = 0.0f;
    LastLinearAccelWorld[0] = 0.0f;
    LastLinearAccelWorld[1] = 0.0f;
    LastLinearAccelWorld[2] = 0.0f;

    if (EulerDeg != NULL)
    {
      float ax_g = AccelData[0] - AccelBias[0];
      float ay_g = AccelData[1] - AccelBias[1];
      float az_g = AccelData[2] - AccelBias[2];
      EulerDeg[0] = atan2f(ay_g, az_g) * RAD2DEG;
      EulerDeg[1] = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD2DEG;
    }
  }

  Position[0] += Velocity[0] * dt_s;
  Position[1] += Velocity[1] * dt_s;
  Position[2] += Velocity[2] * dt_s;
}

void UpdateAttitudeAndPosition(float dt_s, float* EulerDeg, float* Position)
{
  UpdateAttitude(dt_s, EulerDeg);
  UpdatePosition(dt_s, EulerDeg, Position);
}


void Only_for_Roll(float dt_s, float* EulerDeg) //dt_s单位为秒，EulerDeg[0]为Roll角，单位为度
{
  if (EulerDeg == NULL)
  {
    return;
  }

  if (dt_s <= 0.0f || dt_s > 0.2f)
  {
    dt_s = 0.02f;
  }

  MPU6050_GetGyroData(GyroData);    // 单位: deg/s

  {
    static uint16_t stillCounter = 0;
    float gx_dps = GyroData[0] - GyroBias[0];
    float gy_dps = GyroData[1] - GyroBias[1];
    float gz_dps = GyroData[2] - GyroBias[2];

    EulerDeg[0] += gx_dps * dt_s;

    if (fabsf(gx_dps) < 1.2f && fabsf(gy_dps) < 1.2f && fabsf(gz_dps) < 1.2f)
    {
      if (stillCounter < 65535)
      {
        stillCounter++;
      }

      if (stillCounter >= 8)
      {
        GyroBias[0] = 0.995f * GyroBias[0] + 0.005f * GyroData[0];
      }
    }
    else
    {
      stillCounter = 0;
    }
  }
}
