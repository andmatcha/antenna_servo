#include "app.h"

#include "debug_log.h"
#include "modules/gc_packet_receiver.h"
#include "modules/servo_controller.h"

void init(void)
{
    servo_controller_init();
    gc_packet_receiver_init();
    LOG("[antenna_servo] init complete\r\n");
}

void poll(void)
{
    GcPacket packet;

    gc_packet_receiver_poll();
    while (gc_packet_receiver_pop(&packet)) {
        switch (packet.type) {
        case GC_PACKET_TYPE_AUTO:
        case GC_PACKET_TYPE_MANUAL_POSITION:
            if (packet.value >= 0) {
                LOG("[antenna_servo] seq=%u position=%d\r\n",
                    (unsigned int)packet.seq,
                    (int)packet.value);
                servo_controller_set_position((uint16_t)packet.value);
            }
            break;

        case GC_PACKET_TYPE_MANUAL_RATE:
            LOG("[antenna_servo] seq=%u rate=%d\r\n",
                (unsigned int)packet.seq,
                (int)packet.value);
            servo_controller_set_rate(packet.value);
            break;

        case GC_PACKET_TYPE_STOP:
            LOG("[antenna_servo] seq=%u stop\r\n",
                (unsigned int)packet.seq);
            servo_controller_stop();
            break;

        case GC_PACKET_TYPE_HOME:
            LOG("[antenna_servo] seq=%u home\r\n",
                (unsigned int)packet.seq);
            servo_controller_home();
            break;

        default:
            break;
        }
    }

    servo_controller_poll();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    gc_packet_receiver_on_uart_rx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    gc_packet_receiver_on_uart_error(huart);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    servo_controller_on_feedback_capture(htim);
}
