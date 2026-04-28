#ifndef ANTENNA_SERVO_RSSI_MONITOR_H
#define ANTENNA_SERVO_RSSI_MONITOR_H

#include "stm32f3xx_hal.h"

void rssi_monitor_init(void);
void rssi_monitor_poll(void);
void rssi_monitor_on_capture(TIM_HandleTypeDef *htim);

#endif /* ANTENNA_SERVO_RSSI_MONITOR_H */
