#ifndef ANTENNA_SERVO_SERVO_CONFIG_H
#define ANTENNA_SERVO_SERVO_CONFIG_H

/*
 * Tune these values for the actual servo/gearbox assembly.
 * Angles are in 0.1 degree units, PWM timings are in timer ticks (1 tick = 1 us).
 */
#define SERVO_HOME_ANGLE_TENTHS                  1350
#define SERVO_MIN_ANGLE_TENTHS                   0
#define SERVO_MAX_ANGLE_TENTHS                   2700

#define SERVO_PWM_MIN_US                         1000U
#define SERVO_PWM_MAX_US                         3000U
#define SERVO_PHYSICAL_RANGE_TENTHS              2700

#define SERVO_POSITION_SPEED_LIMIT_PER_MILLE        1000
#define SERVO_POSITION_HOME_SPEED_PER_MILLE         1000
#define SERVO_MANUAL_RATE_FULL_SCALE_TENTHS_PER_SEC 2700

#endif /* ANTENNA_SERVO_SERVO_CONFIG_H */
