#ifndef ANTENNA_SERVO_SERVO_CONTROLLER_H
#define ANTENNA_SERVO_SERVO_CONTROLLER_H

#include "stm32f3xx_hal.h"

#include <stdint.h>

typedef struct
{
    uint16_t target_angle_tenths;
    int16_t rate_per_mille;
    int16_t position_speed_limit_per_mille;
    uint16_t pwm_us;
} ServoControllerSettings;

void servo_controller_init(void);
void servo_controller_poll(void);
void servo_controller_set_position(uint16_t angle_tenths);
void servo_controller_set_rate(int16_t rate_per_mille);
void servo_controller_stop(void);
void servo_controller_home(void);
void servo_controller_get_settings(ServoControllerSettings *settings);
void servo_controller_on_feedback_capture(TIM_HandleTypeDef *htim);

#endif /* ANTENNA_SERVO_SERVO_CONTROLLER_H */
