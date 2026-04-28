#include "modules/gc_packet_receiver.h"

#include "debug_log.h"
#include "main.h"

#include <string.h>

#define GC_PACKET_UART huart2
#define GC_PACKET_LEN 9U
#define GC_PACKET_QUEUE_DEPTH 4U
#define GC_PACKET_HEADER_0 'G'
#define GC_PACKET_HEADER_1 'C'

typedef enum
{
    GC_RX_WAIT_G = 0,
    GC_RX_WAIT_C,
    GC_RX_COLLECT,
} GcRxState;

typedef struct
{
    uint8_t rx_byte;
    uint8_t frame[GC_PACKET_LEN];
    uint8_t frame_index;
    GcRxState rx_state;
    GcPacket queue[GC_PACKET_QUEUE_DEPTH];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    volatile uint8_t queue_count;
} GcPacketReceiverContext;

#if DEBUG_LOG_ENABLED
typedef struct
{
    uint32_t crc_errors;
    uint32_t invalid_packets;
    uint32_t queue_overwrites;
    uint32_t uart_errors;
} GcPacketReceiverDebugCounters;

static volatile GcPacketReceiverDebugCounters g_gc_packet_receiver_debug;
#endif

static GcPacketReceiverContext g_gc_packet_receiver;

extern UART_HandleTypeDef GC_PACKET_UART;

static uint32_t enter_critical_section(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical_section(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

#if DEBUG_LOG_ENABLED
static void increment_debug_counter(volatile uint32_t *counter)
{
    (*counter)++;
}

static GcPacketReceiverDebugCounters take_debug_counters(void)
{
    GcPacketReceiverDebugCounters counters;
    uint32_t primask = enter_critical_section();

    counters.crc_errors = g_gc_packet_receiver_debug.crc_errors;
    counters.invalid_packets = g_gc_packet_receiver_debug.invalid_packets;
    counters.queue_overwrites = g_gc_packet_receiver_debug.queue_overwrites;
    counters.uart_errors = g_gc_packet_receiver_debug.uart_errors;

    g_gc_packet_receiver_debug.crc_errors = 0U;
    g_gc_packet_receiver_debug.invalid_packets = 0U;
    g_gc_packet_receiver_debug.queue_overwrites = 0U;
    g_gc_packet_receiver_debug.uart_errors = 0U;

    exit_critical_section(primask);
    return counters;
}
#endif

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool is_known_type(uint8_t type)
{
    return (type == GC_PACKET_TYPE_AUTO) ||
           (type == GC_PACKET_TYPE_MANUAL_POSITION) ||
           (type == GC_PACKET_TYPE_MANUAL_RATE) ||
           (type == GC_PACKET_TYPE_STOP) ||
           (type == GC_PACKET_TYPE_HOME);
}

static bool packet_value_is_valid(uint8_t type, int16_t value)
{
    switch (type) {
    case GC_PACKET_TYPE_AUTO:
    case GC_PACKET_TYPE_MANUAL_POSITION:
        return (value >= 0) && (value <= 3599);

    case GC_PACKET_TYPE_MANUAL_RATE:
        return (value >= -1000) && (value <= 1000);

    case GC_PACKET_TYPE_STOP:
    case GC_PACKET_TYPE_HOME:
        return value == 0;

    default:
        return false;
    }
}

static bool enqueue_packet(const GcPacket *packet)
{
    uint32_t primask = enter_critical_section();

    if (g_gc_packet_receiver.queue_count >= GC_PACKET_QUEUE_DEPTH) {
#if DEBUG_LOG_ENABLED
        increment_debug_counter(&g_gc_packet_receiver_debug.queue_overwrites);
#endif
        g_gc_packet_receiver.queue[g_gc_packet_receiver.queue_tail] = *packet;
        g_gc_packet_receiver.queue_tail =
            (uint8_t)((g_gc_packet_receiver.queue_tail + 1U) %
                      GC_PACKET_QUEUE_DEPTH);
        g_gc_packet_receiver.queue_head = g_gc_packet_receiver.queue_tail;
        exit_critical_section(primask);
        return true;
    }

    g_gc_packet_receiver.queue[g_gc_packet_receiver.queue_tail] = *packet;
    g_gc_packet_receiver.queue_tail =
        (uint8_t)((g_gc_packet_receiver.queue_tail + 1U) % GC_PACKET_QUEUE_DEPTH);
    g_gc_packet_receiver.queue_count++;

    exit_critical_section(primask);
    return true;
}

static void accept_frame(const uint8_t *frame)
{
    GcPacket packet;
    uint16_t received_crc = read_u16_le(&frame[7]);
    uint16_t calculated_crc = crc16_ccitt_false(frame, 7U);

    if (received_crc != calculated_crc) {
#if DEBUG_LOG_ENABLED
        increment_debug_counter(&g_gc_packet_receiver_debug.crc_errors);
#endif
        return;
    }

    packet.seq = read_u16_le(&frame[2]);
    packet.type = frame[4];
    packet.value = read_i16_le(&frame[5]);

    if (!is_known_type(packet.type) ||
        !packet_value_is_valid(packet.type, packet.value)) {
#if DEBUG_LOG_ENABLED
        increment_debug_counter(&g_gc_packet_receiver_debug.invalid_packets);
#endif
        return;
    }

    (void)enqueue_packet(&packet);
}

static void reset_frame_with_header(void)
{
    g_gc_packet_receiver.frame[0] = (uint8_t)GC_PACKET_HEADER_0;
    g_gc_packet_receiver.frame[1] = (uint8_t)GC_PACKET_HEADER_1;
    g_gc_packet_receiver.frame_index = 2U;
    g_gc_packet_receiver.rx_state = GC_RX_COLLECT;
}

static void feed_byte(uint8_t byte)
{
    switch (g_gc_packet_receiver.rx_state) {
    case GC_RX_WAIT_G:
        if (byte == (uint8_t)GC_PACKET_HEADER_0) {
            g_gc_packet_receiver.rx_state = GC_RX_WAIT_C;
        }
        break;

    case GC_RX_WAIT_C:
        if (byte == (uint8_t)GC_PACKET_HEADER_1) {
            reset_frame_with_header();
        } else if (byte != (uint8_t)GC_PACKET_HEADER_0) {
            g_gc_packet_receiver.rx_state = GC_RX_WAIT_G;
        }
        break;

    case GC_RX_COLLECT:
        g_gc_packet_receiver.frame[g_gc_packet_receiver.frame_index++] = byte;
        if (g_gc_packet_receiver.frame_index >= GC_PACKET_LEN) {
            accept_frame(g_gc_packet_receiver.frame);
            g_gc_packet_receiver.rx_state = GC_RX_WAIT_G;
            g_gc_packet_receiver.frame_index = 0U;

            if (byte == (uint8_t)GC_PACKET_HEADER_0) {
                g_gc_packet_receiver.rx_state = GC_RX_WAIT_C;
            }
        }
        break;

    default:
        g_gc_packet_receiver.rx_state = GC_RX_WAIT_G;
        g_gc_packet_receiver.frame_index = 0U;
        break;
    }
}

static void restart_uart_reception(void)
{
    if (HAL_UART_Receive_IT(&GC_PACKET_UART,
                            &g_gc_packet_receiver.rx_byte,
                            1U) != HAL_OK) {
        Error_Handler();
    }
}

void gc_packet_receiver_init(void)
{
    memset(&g_gc_packet_receiver, 0, sizeof(g_gc_packet_receiver));
    g_gc_packet_receiver.rx_state = GC_RX_WAIT_G;
    restart_uart_reception();
}

void gc_packet_receiver_poll(void)
{
#if DEBUG_LOG_ENABLED
    GcPacketReceiverDebugCounters counters = take_debug_counters();

    if ((counters.crc_errors != 0U) ||
        (counters.invalid_packets != 0U) ||
        (counters.queue_overwrites != 0U) ||
        (counters.uart_errors != 0U)) {
        LOG("[antenna_servo] gc rx diag: crc=%lu invalid=%lu overwrite=%lu uart=%lu\r\n",
            (unsigned long)counters.crc_errors,
            (unsigned long)counters.invalid_packets,
            (unsigned long)counters.queue_overwrites,
            (unsigned long)counters.uart_errors);
    }
#endif
}

bool gc_packet_receiver_pop(GcPacket *packet)
{
    uint32_t primask;

    if (packet == NULL) {
        return false;
    }

    primask = enter_critical_section();
    if (g_gc_packet_receiver.queue_count == 0U) {
        exit_critical_section(primask);
        return false;
    }

    *packet = g_gc_packet_receiver.queue[g_gc_packet_receiver.queue_head];
    g_gc_packet_receiver.queue_head =
        (uint8_t)((g_gc_packet_receiver.queue_head + 1U) % GC_PACKET_QUEUE_DEPTH);
    g_gc_packet_receiver.queue_count--;
    exit_critical_section(primask);

    return true;
}

void gc_packet_receiver_on_uart_rx_complete(UART_HandleTypeDef *huart)
{
    if (huart != &GC_PACKET_UART) {
        return;
    }

    feed_byte(g_gc_packet_receiver.rx_byte);
    restart_uart_reception();
}

void gc_packet_receiver_on_uart_error(UART_HandleTypeDef *huart)
{
    if (huart != &GC_PACKET_UART) {
        return;
    }

#if DEBUG_LOG_ENABLED
    increment_debug_counter(&g_gc_packet_receiver_debug.uart_errors);
#endif
    g_gc_packet_receiver.rx_state = GC_RX_WAIT_G;
    g_gc_packet_receiver.frame_index = 0U;
    restart_uart_reception();
}
