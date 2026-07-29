/*
 * 电机控制模块
 * 输入范围[-1000, 1000], 正数表示前进，负数表示后退
 */

#include "stm32g4xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include "motor.h"

#define FW_L_GPIO_Port GPIOC
#define BW_L_GPIO_Port GPIOC

#define HTIM_PWM_L htim4 // 左电机使用定时器4
#define HTIM_PWM_R htim4 // 右电机使用定时器4
#define TIM_CHANNEL_L TIM_CHANNEL_1 // 左电机使用定时器4的通道1
#define TIM_CHANNEL_R TIM_CHANNEL_2 // 右电机使用定时器4的通道2

/* -------- 电机方向控制引脚 -------- */
#define FW_L_Port GPIOC          // 左电机前进引脚端口
#define FW_L_Pin  GPIO_PIN_0     // 左电机前进引脚
#define FW_R_GPIO_Port GPIOC     // 右电机前进引脚端口
#define FW_R_Pin  GPIO_PIN_1     // 右电机前进引脚
#define BW_L_Port GPIOC          // 左电机后退引脚端口
#define BW_L_Pin  GPIO_PIN_2     // 左电机后退引脚
#define BW_R_GPIO_Port GPIOC     // 右电机后退引脚端口
#define BW_R_Pin  GPIO_PIN_3     // 右电机后退引脚

/* -------- 编码器定时器配置 -------- */
#define HTIM_Encoder_L htim2     // 左电机编码器使用定时器2
#define HTIM_Encoder_R htim3     // 右电机编码器使用定时器3

/* -------- 电机参数 -------- */
#define MOTOR_OUTPUT_LIMIT   1000  // 电机输出的最大值，单位为千分数
#define MOTOR_OUTPUT_DEADBAND 60   // 电机输出死区，小于该值直接按0处理，减少抖动
#define CIRCLE_COUNT 780          // 轮子每转一圈编码器计数的次数（需根据实际编码器调整）



void Motor_Init(void)
{
    // 初始化电机
    HAL_TIM_PWM_Start(&HTIM_PWM_L, TIM_CHANNEL_L); // 启动定时器4的通道1
    HAL_TIM_PWM_Start(&HTIM_PWM_R, TIM_CHANNEL_R); // 启动定时器4的通道2
    HAL_TIM_Encoder_Start(&HTIM_Encoder_L, TIM_CHANNEL_ALL); // 启动定时器2的编码器模式
    HAL_TIM_Encoder_Start(&HTIM_Encoder_R, TIM_CHANNEL_ALL); // 启动定时器3的编码器模式
}

//============================编码器操作===========================
int Motor_GetEncoderCount(void)
{
    // 获取电机编码器计数值
    int countL = (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_L); // 获取定时器2的计数值
    int countR = (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_R); // 获取定时器3的计数值
    return (countL + countR) / 2; // 返回左右电机编码器计数值的平均值
}
int Motor_GetEncoderCountL(void)
{
    // 获取左电机编码器计数值
    return (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_L); // 获取定时器2的计数值
}
int Motor_GetEncoderCountR(void)
{
    // 获取右电机编码器计数值
    return (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_R); // 获取定时器3的计数值
}
//=================================================================

void Motor_SetSpeed(int speedL, int speedR) //-1000~+1000
{
    /*
     * 设置电机速度
     * speedL: 左电机速度，范围[-1000, 1000]，正数表示前进，负数表示后退
     * speedR: 右电机速度，范围[-1000, 1000], 正数表示前进，负数表示后退
     */
    if(speedL > MOTOR_OUTPUT_LIMIT) speedL = MOTOR_OUTPUT_LIMIT;
    if(speedL < -MOTOR_OUTPUT_LIMIT) speedL = -MOTOR_OUTPUT_LIMIT;
    if(speedR > MOTOR_OUTPUT_LIMIT) speedR = MOTOR_OUTPUT_LIMIT;
    if(speedR < -MOTOR_OUTPUT_LIMIT) speedR = -MOTOR_OUTPUT_LIMIT;

    //死区处理
    if (speedL < MOTOR_OUTPUT_DEADBAND && speedL > -MOTOR_OUTPUT_DEADBAND)
    {
        speedL = 0;
    }
    if (speedR < MOTOR_OUTPUT_DEADBAND && speedR > -MOTOR_OUTPUT_DEADBAND)
    {
        speedR = 0;
    }
    
    // 控制左电机
    if (speedL > 0)
    {
        // 前进
        HAL_GPIO_WritePin(FW_L_GPIO_Port, FW_L_Pin, GPIO_PIN_SET); // 左电机正转
        HAL_GPIO_WritePin(BW_L_GPIO_Port, BW_L_Pin, GPIO_PIN_RESET); // 左电机反转
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_L, TIM_CHANNEL_L, speedL); // 设置PWM占空比
    }
    else if (speedL < 0)
    {
        // 后退
        HAL_GPIO_WritePin(FW_L_GPIO_Port, FW_L_Pin, GPIO_PIN_RESET); // 左电机正转
        HAL_GPIO_WritePin(BW_L_GPIO_Port, BW_L_Pin, GPIO_PIN_SET); // 左电机反转
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_L, TIM_CHANNEL_L, -speedL); // 设置PWM占空比
    }
    else
    {
        // 刹车
        HAL_GPIO_WritePin(FW_L_GPIO_Port, FW_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BW_L_GPIO_Port, BW_L_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_L, TIM_CHANNEL_L, 0);
    }
    // 控制右电机
    if (speedR > 0)
    {
        // 前进
        HAL_GPIO_WritePin(FW_R_GPIO_Port, FW_R_Pin, GPIO_PIN_SET); // 右电机正转
        HAL_GPIO_WritePin(BW_R_GPIO_Port, BW_R_Pin, GPIO_PIN_RESET); // 右电机反转
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_R, TIM_CHANNEL_R, speedR); // 设置PWM占空比
    }
    else if (speedR < 0)
    {
        // 后退
        HAL_GPIO_WritePin(FW_R_GPIO_Port, FW_R_Pin, GPIO_PIN_RESET); // 右电机正转
        HAL_GPIO_WritePin(BW_R_GPIO_Port, BW_R_Pin, GPIO_PIN_SET); // 右电机反转
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_R, TIM_CHANNEL_R, -speedR); // 设置PWM占空比
    }
    else
    {
        // 刹车
        HAL_GPIO_WritePin(FW_R_GPIO_Port, FW_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BW_R_GPIO_Port, BW_R_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&HTIM_PWM_R, TIM_CHANNEL_R, 0);
    }
}

double Get_SpeedL(int dt_ms)
{
    static int lastCounterL = 0;
    int currentCounterL = (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_L);
    int deltaCountL = currentCounterL - lastCounterL;
    lastCounterL = currentCounterL;

    if (dt_ms <= 0) return 0.0;

    // 计算速度（归一化值，单位：编码器圈数/毫秒）
    double speedL = (double)deltaCountL / CIRCLE_COUNT / dt_ms;
    return speedL;
}

double Get_SpeedR(int dt_ms)
{
    static int lastCounterR = 0;
    int currentCounterR = (int)__HAL_TIM_GET_COUNTER(&HTIM_Encoder_R);
    int deltaCountR = currentCounterR - lastCounterR;
    lastCounterR = currentCounterR;

    if (dt_ms <= 0) return 0.0;

    // 计算速度（归一化值，单位：编码器圈数/毫秒）
    double speedR = (double)deltaCountR / CIRCLE_COUNT / dt_ms;
    return speedR;
}
