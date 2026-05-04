#ifndef ANTENNA_SERVO_RSSI_MONITOR_H
#define ANTENNA_SERVO_RSSI_MONITOR_H

#include "stm32f3xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int32_t value;
    uint32_t duty_per_mille;
    uint32_t age_ms;
    bool fresh;
} RssiMonitorReading;

void rssi_monitor_init(void);
void rssi_monitor_poll(void);
void rssi_monitor_on_capture(TIM_HandleTypeDef *htim);
bool rssi_monitor_get_reading(RssiMonitorReading *reading);

#endif /* ANTENNA_SERVO_RSSI_MONITOR_H */
