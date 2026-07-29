#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Project-wide unit contract: mm, mm/s, ms and degrees. */
#define APP_PWM_FULL_SCALE                 1000
#define APP_CONTROL_FAST_MS                5U
#define APP_CONTROL_LINE_MS                10U
#define APP_STATE_UPDATE_MS                10U
#define APP_TELEMETRY_MS                   100U

/*
 * Removable balancing-bench console over ST-Link VCP (LPUART1, 115200 8N1).
 * Set APP_ENABLE_BENCH_DEBUG to 0 after calibration; the module then compiles
 * to inert stubs and cannot enable a motor.
 */
#define APP_ENABLE_BENCH_DEBUG              1
#define APP_BENCH_OPEN_LOOP_PWM_LIMIT       250
#define APP_BENCH_PULSE_MAX_MS              500U
#define APP_BENCH_CLOSED_LOOP_PWM_LIMIT     300.0f
#define APP_BENCH_ANGLE_LIMIT_DEG            2.0f
#define APP_BENCH_CLOSED_LOOP_MAX_MS      30000U
#define APP_BENCH_TELEMETRY_MS              100U
#define APP_BENCH_INITIAL_ANGLE_KP           30.0f
#define APP_BENCH_INITIAL_ANGLE_KI            0.0f
#define APP_BENCH_INITIAL_ANGLE_KD            0.0f
#define APP_BENCH_INITIAL_BALL_KP             0.005f
#define APP_BENCH_INITIAL_BALL_KD             0.0f

/* Encoder counts are OUTPUT-shaft counts after the timer's TI12 decoding. */
#define APP_ENCODER_LEFT_CPR               390.0f
#define APP_ENCODER_RIGHT_CPR              390.0f
#define APP_ENCODER_BEAM_CPR               3200.0f
#define APP_ENCODER_LEFT_SIGN              1.0f
#define APP_ENCODER_RIGHT_SIGN             1.0f
#define APP_ENCODER_BEAM_SIGN              1.0f
#define APP_WHEEL_DIAMETER_MM              66.0f
#define APP_WHEEL_TRACK_MM                 180.0f /* Replace with measured value. */

#define APP_MOTOR_LEFT_SIGN                1
#define APP_MOTOR_RIGHT_SIGN               1
#define APP_MOTOR_BEAM_SIGN                1
#define APP_MOTOR_LEFT_MIN_PWM             80
#define APP_MOTOR_RIGHT_MIN_PWM            80
#define APP_MOTOR_BEAM_MIN_PWM             80
#define APP_MOTOR_CHASSIS_MAX_PWM          900
#define APP_MOTOR_BEAM_MAX_PWM             700

/* Safe P60 range relative to the mechanically centered startup/home count. */
#define APP_BEAM_RANGE_VERIFIED              0
#define APP_BEAM_ENCODER_MIN_COUNT          (-260)
#define APP_BEAM_ENCODER_MAX_COUNT          260
#define APP_BEAM_STALL_PWM                  350
#define APP_BEAM_STALL_DELTA_COUNT          1
#define APP_BEAM_STALL_TIMEOUT_MS           300U

/* SA100 PWM conversion. Calibrate offset and sign on the assembled mechanism. */
#define APP_SA100_CALIBRATION_VERIFIED        0
#define APP_SA100_DUTY_TO_DEG               360.0f
#define APP_SA100_HORIZONTAL_RAW_DEG        180.0f
#define APP_SA100_ANGLE_SIGN                1.0f
#define APP_SA100_PERIOD_MIN_US             100U
#define APP_SA100_PERIOD_MAX_US             60000U
#define APP_SA100_TIMEOUT_MS                100U
#define APP_BEAM_ANGLE_SOFT_LIMIT_DEG       3.0f
#define APP_BEAM_INITIAL_CMD_LIMIT_DEG      2.0f

/* Vision protocol: "$B,<x_mm>,<status>\n", 1=measured, 2=held, 0=invalid. */
#define APP_VISION_TIMEOUT_MS               200U
#define APP_BALL_POSITION_LIMIT_MM          105.0f
#define APP_BALL_TARGET_LIMIT_MM            80.0f
#define APP_BALL_SPEED_FILTER_ALPHA         0.25f

/* Eight digital sensors are active-low by default. */
#define APP_LINE_ACTIVE_LEVEL               GPIO_PIN_RESET
#define APP_LINE_LOST_TIMEOUT_MS            500U
#define APP_LINE_STARTUP_GRACE_MS           500U
#define APP_LINE_CROSS_MIN_ACTIVE           6U
#define APP_LINE_CROSS_CONFIRM_MS           30U
#define APP_LAP_ARM_DISTANCE_MM             300.0f
#define APP_AB_FINISH_DISTANCE_MM            1550.0f /* Pass B; verify on track. */
#define APP_AFTER_A_RUN_DISTANCE_MM           200.0f

/* Initial untuned controller values; hardware tuning is mandatory. */
#define APP_WHEEL_KP                        0.7f
#define APP_WHEEL_KI                        2.0f
#define APP_WHEEL_PWM_LIMIT                 850.0f
#define APP_WHEEL_ACCEL_LIMIT_MM_S2         500.0f
#define APP_LINE_KP                         180.0f
#define APP_LINE_KD                         25.0f
#define APP_LINE_STEER_LIMIT_MM_S           300.0f
#define APP_CHASSIS_BASE_SPEED_MM_S         300.0f
#define APP_CHASSIS_CURVE_SPEED_MM_S        180.0f

#define APP_BEAM_ANGLE_KP                   180.0f
#define APP_BEAM_ANGLE_KI                   5.0f
#define APP_BEAM_ANGLE_KD                   15.0f
#define APP_BALL_KP                         0.035f
#define APP_BALL_KD                         0.012f
#define APP_BALL_CONTROL_SIGN               1.0f
#define APP_BALL_ANGLE_LIMIT_DEG            2.0f

#define APP_BALL_SETTLE_ERROR_MM            10.0f
#define APP_BALL_START_TOLERANCE_MM          15.0f
#define APP_BALL_SETTLE_SPEED_MM_S          25.0f
#define APP_BALL_SETTLE_TIME_MS             250U
#define APP_STATIC_STAGE_TIMEOUT_MS          5000U

#endif /* APP_CONFIG_H */
