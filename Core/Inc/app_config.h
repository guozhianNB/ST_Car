#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Project-wide unit contract: mm, mm/s, ms, degrees and command steps/s. */
#define APP_PWM_FULL_SCALE                 1000
#define APP_CONTROL_FAST_MS                  5U
#define APP_CONTROL_LINE_MS                 10U
#define APP_STATE_UPDATE_MS                 10U
#define APP_TELEMETRY_MS                   100U

/* All three installed buttons start requirement 3 on a short press. */
#define APP_BUTTON_DEBOUNCE_MS              30U
#define APP_BUTTON_LONG_PRESS_MS          1000U
#define APP_REQUIREMENT3_CENTER_TIMEOUT_MS 45000U
#define APP_REQUIREMENT3_CENTER_ERROR_MM      8.0f
#define APP_REQUIREMENT3_CENTER_SPEED_MM_S    4.0f
#define APP_REQUIREMENT3_CENTER_SETTLE_MS    800U
#define APP_REQUIREMENT3_CENTER_KD             4.0f
#define APP_REQUIREMENT3_FINE_ENTER_MM         20.0f
#define APP_REQUIREMENT3_FINE_EXIT_MM          40.0f
#define APP_REQUIREMENT3_FINE_KI                2.0f
#define APP_REQUIREMENT3_FINE_KD                6.0f
#define APP_REQUIREMENT3_FINE_RATE_STEPS_S    200.0f

/* Moving-platform zero regulator.  This is an independent direct-rate LQI
 * controller; it does not use the legacy ball PID pose target, endpoint
 * release, or static-release state machines.  Initial limits are deliberately
 * low for post-repair plant identification and are runtime-tunable in bench. */
#define APP_ZERO_KX_STEPS_S_PER_MM               4.0f
#define APP_ZERO_KV_STEPS_PER_MM                  2.0f
#define APP_ZERO_KI_STEPS_S_PER_MM_S              0.0f
#define APP_ZERO_KQ_PER_S                          4.0f
#define APP_ZERO_KA_STEPS_PER_MM_S2                0.0f
#define APP_ZERO_RATE_LIMIT_STEPS_S              100.0f
#define APP_ZERO_POSE_LIMIT_STEPS                100.0f
#define APP_ZERO_INTEGRAL_LIMIT_MM_S             100.0f
#define APP_ZERO_INTEGRAL_LEAK_PER_S               0.20f
#define APP_ZERO_VELOCITY_FILTER_ALPHA             0.60f
#define APP_ZERO_ACCEL_FILTER_ALPHA                0.25f
#define APP_ZERO_VISION_HOLD_MS                    80U

/* Removable balancing-bench console over ST-Link VCP (115200 8N1). */
#define APP_ENABLE_BENCH_DEBUG                1
#define APP_BENCH_OPEN_LOOP_RATE_LIMIT_STEPS_S 100
#define APP_BENCH_RUN_MAX_MS                500U
#define APP_BENCH_RUN_MAX_COMMAND_PULSES      10U
#define APP_BENCH_CLOSED_LOOP_RATE_LIMIT_STEPS_S 100.0f
#define APP_BENCH_CLOSED_LOOP_MAX_MS      30000U
#define APP_BENCH_TELEMETRY_MS              100U
#define APP_BENCH_INITIAL_BALL_KP             APP_BALL_KP
#define APP_BENCH_INITIAL_BALL_KI             APP_BALL_KI
#define APP_BENCH_INITIAL_BALL_KD             APP_BALL_KD

/* Wheel encoder counts are output-shaft TI12 counts per revolution. */
#define APP_ENCODER_LEFT_CPR               390.0f
#define APP_ENCODER_RIGHT_CPR              390.0f
#define APP_ENCODER_LEFT_SIGN                1.0f
#define APP_ENCODER_RIGHT_SIGN               1.0f
/*
 * VERIFIED (user bench test, 2026-08-01): positive step command turns the
 * MS42CG clockwise, raises the tube, and produces -4096 raw TIM4 counts/rev.
 * Negate the encoder so positive command motion is positive in telemetry.
 * The actuator encoder remains stall/debug-only and is not PID feedback.
 */
#define APP_ENCODER_ACTUATOR_CPR            4096.0f
#define APP_ENCODER_ACTUATOR_SIGN             -1.0f
#define APP_WHEEL_DIAMETER_MM               66.0f
#define APP_WHEEL_TRACK_MM                 180.0f /* UNVERIFIED: measure. */

#define APP_MOTOR_LEFT_SIGN                    1
#define APP_MOTOR_RIGHT_SIGN                   1
#define APP_MOTOR_LEFT_MIN_PWM                80
#define APP_MOTOR_RIGHT_MIN_PWM               80
#define APP_MOTOR_CHASSIS_MAX_PWM            900

/*
 * MS42CG + PUFEIDE TB6600 optocoupler driver command contract.
 * Wiring: 3V3 common-anode; PB14/PC4/PC5 drive PUL-/DIR-/ENA-.
 * A high PC5 level makes ENA optocoupler inactive and enables the driver.
 * VERIFIED (user bench test, 2026-08-01): direction sign +1 makes a positive
 * command rotate clockwise and raise the tube. VERIFIED (user switch and
 * bounded jog, 2026-08-01): SW1=OFF, SW2=OFF, SW3=ON, i.e. 16 microsteps and
 * 3200 command pulses/rev. Rates and gains below remain deliberately
 * conservative until the loaded mechanism is tuned.
 */
#define APP_STEPPER_TIMER_TICK_HZ        1000000U
#define APP_STEPPER_TIMER_CLOCK_HZ     170000000U
#define APP_STEPPER_FULL_STEPS_PER_REV        200U
#define APP_STEPPER_MICROSTEPS                  16U
#define APP_STEPPER_COMMAND_STEPS_PER_REV     3200U
#define APP_STEPPER_DIRECTION_SIGN               1
#define APP_STEPPER_ENABLE_ACTIVE_HIGH            1
#define APP_STEPPER_MIN_RATE_STEPS_S            20
#define APP_STEPPER_MAX_RATE_STEPS_S          2000
#define APP_STEPPER_ACCEL_STEPS_S2         20000.0f
#define APP_STEPPER_DIRECTION_SETUP_MS           1U
#define APP_STEPPER_STALL_MIN_RATE_STEPS_S       32
#define APP_STEPPER_STALL_DELTA_COUNT             0
#define APP_STEPPER_STALL_TIMEOUT_MS            750U
#define APP_STEPPER_STALL_RESET_MS              500U
/* VERIFIED 2026-08-01 after repair: firmware is flashed with the actuator
 * backed 10 command pulses away from the physical right-tilt stop.  Positive
 * encoder travel moves away from that stop.  The left bound is deliberately
 * conservative until both endpoints are remeasured.  Do not reset the
 * actuator encoder after boot while this reference is in use. */
#define APP_ACTUATOR_RIGHT_GUARD_COUNT              0
#define APP_ACTUATOR_LEFT_GUARD_COUNT            1200

/* Vision protocol: "$B,<x_mm>,<status>\n", 1=measured, 2=held, 0=invalid. */
#define APP_VISION_COAST_MS                   500U
#define APP_VISION_CONTROL_HOLD_MS            150U
#define APP_VISION_TIMEOUT_MS                3000U
#define APP_VISION_SPEED_RESET_MS             300U
#define APP_VISION_MAX_SPEED_MM_S           2200.0f
#define APP_BALL_POSITION_LIMIT_MM            140.0f
#define APP_BALL_TARGET_LIMIT_MM               80.0f
#define APP_BALL_SPEED_FILTER_ALPHA             0.25f

/* Eight digital line sensors are active-low by default. */
#define APP_LINE_ACTIVE_LEVEL           GPIO_PIN_RESET
#define APP_LINE_LOST_TIMEOUT_MS              500U
#define APP_LINE_STARTUP_GRACE_MS              500U
#define APP_LINE_CROSS_MIN_ACTIVE                 6U
#define APP_LINE_CROSS_CONFIRM_MS               30U
#define APP_LAP_ARM_DISTANCE_MM                300.0f
#define APP_AB_FINISH_DISTANCE_MM             1550.0f /* UNVERIFIED on track. */
#define APP_AFTER_A_RUN_DISTANCE_MM            200.0f

/* Wheel/line initial values; final hardware tuning is mandatory. */
#define APP_WHEEL_KP                            0.7f
#define APP_WHEEL_KI                            2.0f
#define APP_WHEEL_PWM_LIMIT                   850.0f
#define APP_WHEEL_ACCEL_LIMIT_MM_S2           500.0f
#define APP_LINE_KP                           180.0f
#define APP_LINE_KD                            25.0f
#define APP_LINE_STEER_LIMIT_MM_S             300.0f
#define APP_CHASSIS_BASE_SPEED_MM_S           300.0f
#define APP_CHASSIS_CURVE_SPEED_MM_S          180.0f

/*
 * The visual outer loop commands a bounded relative tube-pose request.  A
 * pulse accumulator and rate shaper realize that request; the actuator
 * encoder remains diagnostic-only and is not feedback for this controller.
 * VERIFIED on the loaded mechanism on 2026-08-01: the static requirement-3
 * sequence completed in 4859 ms and 3116 ms in two consecutive runs.
 */
#define APP_BALL_KP                             1.2f
#define APP_BALL_KI                             2.0f
#define APP_BALL_KD                             2.0f
#define APP_BALL_KA                             0.0f
#define APP_BALL_ACCEL_FILTER_ALPHA             0.45f
#define APP_BALL_ACCEL_SAMPLE_MIN_MS             80U
#define APP_BALL_ACCEL_SAMPLE_MAX_MS            250U
#define APP_BALL_ACCEL_LIMIT_MM_S2            1500.0f
#define APP_BALL_CONTROL_SIGN                   -1.0f
#define APP_BALL_RATE_LIMIT_STEPS_S            400.0f
/* The visual PID commands a relative step position.  The following open-loop
 * pulse accumulator is actuator command shaping, not encoder feedback. */
#define APP_BALL_POSITION_LIMIT_STEPS         1600.0f
#define APP_BALL_PID_POSITION_LIMIT_STEPS      300.0f
#define APP_BALL_ENDPOINT_POSITION_LIMIT_STEPS 1200.0f
#define APP_BALL_ENDPOINT_RATE_LIMIT_STEPS_S    100.0f
#define APP_BALL_ENDPOINT_CATCH_RATE_STEPS_S   2000.0f
#define APP_BALL_ENDPOINT_CATCH_EXIT_SPEED_MM_S 25.0f
#define APP_BALL_POSITION_RATE_KP               10.0f
#define APP_BALL_POSITION_ERROR_STOP_STEPS       1.0f
#define APP_BALL_INTEGRAL_POSITION_LIMIT_STEPS 600.0f
#define APP_BALL_INTEGRAL_ACTIVE_SPEED_MM_S      4.0f
#define APP_BALL_INTEGRAL_RESET_SPEED_MM_S      10.0f
#define APP_BALL_INTEGRAL_ARM_TIME_MS           200U
#define APP_BALL_INTEGRAL_BIAS_RATE_STEPS_S     600.0f
#define APP_BALL_STATIC_RELEASE_TRAVEL_MM          2.0f
#define APP_BALL_PID_HOLD_ERROR_MM               5.0f
#define APP_BALL_PID_HOLD_SPEED_MM_S             8.0f
#define APP_BALL_PID_HOLD_ACCEL_MM_S2           20.0f
/* At a physical tube end the ball can remain static near level because the
 * end contact and rolling friction hide tube attitude.  Probe toward the
 * interior only from low speed, then hand back to PD as soon as measured
 * inward velocity proves that the ball has released. */
#define APP_BALL_ENDPOINT_ENTER_ABS_MM          115.0f
#define APP_BALL_ENDPOINT_EXIT_ABS_MM           112.0f
#define APP_BALL_ENDPOINT_RELEASE_SPEED_MM_S     12.0f
#define APP_BALL_ENDPOINT_RELEASE_CONFIRM_FRAMES     2U

#define APP_BALL_SETTLE_ERROR_MM                10.0f
#define APP_BALL_START_TOLERANCE_MM             12.0f
#define APP_BALL_SETTLE_SPEED_MM_S              25.0f
#define APP_BALL_SETTLE_TIME_MS                 250U
#define APP_BALL_SEQUENCE_GOAL_MM                50.0f
/* Requirement 3 is a traverse, not two independent settle operations.  Keep
 * the judged targets at +/-50 mm, but lead the internal visual-PD target so
 * derivative braking cannot strand the ball before the +40 mm transition.
 * On the return leg, remove the lead early enough to brake into -50 +/-10 mm. */
#define APP_BALL_SEQUENCE_POSITIVE_LEAD_MM        70.0f
#define APP_BALL_SEQUENCE_NEGATIVE_LEAD_MM        70.0f
#define APP_BALL_SEQUENCE_NEGATIVE_BRAKE_MM      -35.0f
#define APP_BALL_SEQUENCE_CHASE_KD_SCALE           1.00f
#define APP_STATIC_STAGE_TIMEOUT_MS            5000U

#endif /* APP_CONFIG_H */
