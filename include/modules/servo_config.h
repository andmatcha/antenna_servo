#ifndef ANTENNA_SERVO_SERVO_CONFIG_H
#define ANTENNA_SERVO_SERVO_CONFIG_H

/*
 * Tune these values for the actual servo/gearbox assembly.
 * Angles are in 0.1 degree units, PWM timings are in timer ticks (1 tick = 1 us).
 */
#define SERVO_HOME_ANGLE_TENTHS                  1350
#define SERVO_MIN_ANGLE_TENTHS                   0
#define SERVO_MAX_ANGLE_TENTHS                   2700

#define SERVO_PWM_NEUTRAL_US                     1500U
#define SERVO_PWM_FULL_SPEED_OFFSET_US           400U
#define SERVO_PWM_MIN_US                         1000U
#define SERVO_PWM_MAX_US                         2000U

#define SERVO_POSITION_SPEED_LIMIT_PER_MILLE     1000
#define SERVO_POSITION_HOME_SPEED_PER_MILLE      1000
#define SERVO_POSITION_TOLERANCE_TENTHS          10
#define SERVO_POSITION_KP_PER_TENTH              8
#define SERVO_POSITION_MIN_COMMAND_PER_MILLE     120

#define SERVO_FEEDBACK_DUTY_MIN_PER_MILLE        29U
#define SERVO_FEEDBACK_DUTY_MAX_PER_MILLE        971U
#define SERVO_FEEDBACK_FULL_SCALE_TENTHS         3600U
#define SERVO_FEEDBACK_STALE_TIMEOUT_MS          100U
#define SERVO_FEEDBACK_MIN_PERIOD_US             500U
#define SERVO_FEEDBACK_MAX_PERIOD_US             2500U

#define SERVO_CONTROL_DIRECTION                  1
#define SERVO_FEEDBACK_INVERTED                  0

#endif /* ANTENNA_SERVO_SERVO_CONFIG_H */
