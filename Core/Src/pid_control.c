/*
 * PID 控制模块
 *
 * 架构：
 *               ┌─────────────────────────────┐
 *  track_error ─┤ 寻迹 PID (Tracking PID)     ├─┐
 *               └─────────────────────────────┘ │
 *                                               ├─→ total_correction
 *               ┌─────────────────────────────┐ │
 *  speedL-R   ──┤ 编码器直线修正 (Encoder PID) ├─┘
 *               └─────────────────────────────┘
 *
 * 电机输出：
 *   outL = target_speed - total_correction
 *   outR = target_speed + total_correction
 */

#include "pid_control.h"
#include "motor.h"

/* ============================ 常量 ============================ */
#define SPEED_LIMIT      1000     // 电机速度限幅
#define DT_MS_DEFAULT    20       // 默认时间间隔（ms）
#define DT_MS_MAX        100      // 安全上限，超过此值使用默认值

/* ======================== PID 控制器结构体 ======================== */
typedef struct {
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    float integral;        // 积分累积值
    float last_error;      // 上一次误差
    float integral_limit;  // 积分限幅
    float output_limit;    // 输出限幅
    float deadband;        // 死区（误差绝对值小于此值不调节）
} PID_Controller;

/* ====================== 静态实例 ====================== */
/*
 * 编码器直线修正 PID
 * 作用：保持左右轮速度一致，抵抗机械偏差、地面不平等干扰
 * 误差 = speedL - speedR
 */
static PID_Controller g_encoder_pid = {
    .kp             = 0.5f,
    .ki             = 0.02f,
    .kd             = 0.1f,
    .integral       = 0.0f,
    .last_error     = 0.0f,
    .integral_limit = 50.0f,
    .output_limit   = 200.0f,
    .deadband       = 0.001f,
};

/*
 * 寻迹 PID
 * 作用：根据红外传感器返回的偏离误差，控制小车沿路径行驶
 * 误差 = track_error（外部输入，无单位）
 */
static PID_Controller g_tracking_pid = {
    .kp             = 1.0f,
    .ki             = 0.05f,
    .kd             = 0.2f,
    .integral       = 0.0f,
    .last_error     = 0.0f,
    .integral_limit = 100.0f,
    .output_limit   = 300.0f,
    .deadband       = 0.01f,
};

/* ====================== 内部函数 ====================== */

/**
 * @brief  通用 PID 更新
 * @param  pid     PID 控制器指针
 * @param  error   当前误差
 * @param  dt_s    时间间隔（秒）
 * @return         修正值
 */
static float PID_Update(PID_Controller *pid, float error, float dt_s)
{
    /* 安全处理 dt_s */
    if (dt_s <= 0.0f || dt_s > 1.0f) {
        dt_s = DT_MS_DEFAULT / 1000.0f;
    }

    /* 死区：小误差不做调节，避免频繁抖动 */
    if (error > -pid->deadband && error < pid->deadband) {
        pid->integral = 0.0f;
        pid->last_error = 0.0f;
        return 0.0f;
    }

    /* 过零清除积分，防止积分饱和过冲 */
    if (error * pid->last_error < 0.0f) {
        pid->integral = 0.0f;
    }

    /* === PID 三要素 === */
    float proportional = pid->kp * error;

    pid->integral += error * dt_s;
    /* 积分限幅 */
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    float integral_term = pid->ki * pid->integral;

    float derivative = (error - pid->last_error) / dt_s;
    float derivative_term = pid->kd * derivative;

    float output = proportional + integral_term + derivative_term;

    /* 输出限幅 */
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    pid->last_error = error;
    return output;
}

/* ====================== 接口实现 ====================== */

void PID_Control_Init(void)
{
    /* 恢复默认参数 —— 直接使用静态初始化时的值，不做额外操作 */
    g_encoder_pid.integral     = 0.0f;
    g_encoder_pid.last_error   = 0.0f;

    g_tracking_pid.integral    = 0.0f;
    g_tracking_pid.last_error  = 0.0f;
}

void PID_Control_SetTarget(int target_speed, float track_error,
                           int *out_speedL, int *out_speedR)
{
    int speedL_val = 0, speedR_val = 0;

    /* 空指针保护 */
    if (!out_speedL) out_speedL = &speedL_val;
    if (!out_speedR) out_speedR = &speedR_val;

    /* 1. 计算时间差 dt ───────────────────────────────── */
    static uint32_t last_tick = 0;
    uint32_t now_tick = HAL_GetTick();
    uint32_t dt_ms = now_tick - last_tick;
    last_tick = now_tick;

    /* 安全检查：首次调用或异常间隔使用默认值 */
    if (dt_ms < 1 || dt_ms > DT_MS_MAX) {
        dt_ms = DT_MS_DEFAULT;
    }
    float dt_s = dt_ms / 1000.0f;

    /* 2. 读取编码器速度 ──────────────────────────────── */
    double speedL = Get_SpeedL((int)dt_ms);
    double speedR = Get_SpeedR((int)dt_ms);

    /* 3. 编码器直线修正 PID ──────────────────────────── */
    float speed_diff = (float)(speedL - speedR);
    float encoder_correction = PID_Update(&g_encoder_pid, speed_diff, dt_s);

    /* 4. 寻迹 PID ────────────────────────────────────── */
    float track_correction = PID_Update(&g_tracking_pid, track_error, dt_s);

    /* 5. 叠加修正 ────────────────────────────────────── */
    float total_correction = encoder_correction + track_correction;

    /* 6. 计算电机输出 ────────────────────────────────── */
    /*
     * 当 total_correction > 0 时：
     *   左轮减速、右轮加速 → 左转
     * 当 total_correction < 0 时：
     *   左轮加速、右轮减速 → 右转
     */
    *out_speedL = target_speed - (int)total_correction;
    *out_speedR = target_speed + (int)total_correction;

    /* 7. 限幅保护 ────────────────────────────────────── */
    if (*out_speedL > SPEED_LIMIT)  *out_speedL = SPEED_LIMIT;
    if (*out_speedL < -SPEED_LIMIT) *out_speedL = -SPEED_LIMIT;
    if (*out_speedR > SPEED_LIMIT)  *out_speedR = SPEED_LIMIT;
    if (*out_speedR < -SPEED_LIMIT) *out_speedR = -SPEED_LIMIT;
}

void PID_Control_Reset(void)
{
    g_encoder_pid.integral    = 0.0f;
    g_encoder_pid.last_error  = 0.0f;

    g_tracking_pid.integral   = 0.0f;
    g_tracking_pid.last_error = 0.0f;
}

/* ====================== 在线调参 ====================== */

void PID_Control_TuneEncoderPID(float kp, float ki, float kd)
{
    if (kp >= 0.0f) g_encoder_pid.kp = kp;
    if (ki >= 0.0f) g_encoder_pid.ki = ki;
    if (kd >= 0.0f) g_encoder_pid.kd = kd;
}

void PID_Control_TuneTrackingPID(float kp, float ki, float kd)
{
    if (kp >= 0.0f) g_tracking_pid.kp = kp;
    if (ki >= 0.0f) g_tracking_pid.ki = ki;
    if (kd >= 0.0f) g_tracking_pid.kd = kd;
}
