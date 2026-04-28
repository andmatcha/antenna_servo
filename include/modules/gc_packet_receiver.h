#ifndef ANTENNA_SERVO_GC_PACKET_RECEIVER_H
#define ANTENNA_SERVO_GC_PACKET_RECEIVER_H

#include "stm32f3xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    GC_PACKET_TYPE_AUTO = 0x01,
    GC_PACKET_TYPE_MANUAL_POSITION = 0x02,
    GC_PACKET_TYPE_MANUAL_RATE = 0x03,
    GC_PACKET_TYPE_STOP = 0x04,
    GC_PACKET_TYPE_HOME = 0x05,
} GcPacketType;

typedef struct
{
    uint16_t seq;
    uint8_t type;
    int16_t value;
} GcPacket;

void gc_packet_receiver_init(void);
void gc_packet_receiver_poll(void);
bool gc_packet_receiver_pop(GcPacket *packet);
void gc_packet_receiver_on_uart_rx_complete(UART_HandleTypeDef *huart);
void gc_packet_receiver_on_uart_error(UART_HandleTypeDef *huart);

#endif /* ANTENNA_SERVO_GC_PACKET_RECEIVER_H */
