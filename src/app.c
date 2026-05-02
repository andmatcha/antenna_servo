#include "app.h"

#include "debug_log.h"
#include "main.h"
#include "modules/board_led.h"
#include "modules/gc_packet_receiver.h"
#include "modules/rssi_monitor.h"
#include "modules/servo_controller.h"

#if DEBUG_LOG_ENABLED
#define SERVO_STATUS_LOG_INTERVAL_MS 250U

static ServoControllerSettings g_last_logged_servo_settings;
static uint32_t g_last_servo_status_log_ms;
static uint8_t g_servo_status_log_initialized;

static void log_reset_message(void)
{
    uint8_t pin_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U);
    uint8_t por_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U);
    uint8_t software_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0U);
    uint8_t iwdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0U);
    uint8_t wwdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != 0U);
    uint8_t low_power_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != 0U);
    uint8_t option_byte_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST) != 0U);
    uint8_t no_reset_flags =
        ((pin_reset | por_reset | software_reset | iwdg_reset |
          wwdg_reset | low_power_reset | option_byte_reset) == 0U);

    LOG("[antenna_servo] reset: cause=%s%s%s%s%s%s%s%s\r\n",
        pin_reset ? " PIN" : "",
        por_reset ? " POR" : "",
        software_reset ? " SOFTWARE" : "",
        iwdg_reset ? " IWDG" : "",
        wwdg_reset ? " WWDG" : "",
        low_power_reset ? " LOW_POWER" : "",
        option_byte_reset ? " OPTION_BYTE" : "",
        no_reset_flags ? " UNKNOWN" : "");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

static const char *gc_packet_type_name(uint8_t type)
{
    switch (type) {
    case GC_PACKET_TYPE_AUTO:
        return "AUTO";

    case GC_PACKET_TYPE_MANUAL_POSITION:
        return "MANUAL_POSITION";

    case GC_PACKET_TYPE_MANUAL_RATE:
        return "MANUAL_RATE";

    case GC_PACKET_TYPE_STOP:
        return "STOP";

    case GC_PACKET_TYPE_HOME:
        return "HOME";

    default:
        return "UNKNOWN";
    }
}

static uint8_t servo_status_log_should_update(
    const ServoControllerSettings *settings)
{
    return (g_servo_status_log_initialized == 0U) ||
           (settings->target_angle_tenths !=
            g_last_logged_servo_settings.target_angle_tenths) ||
           (settings->rate_per_mille !=
            g_last_logged_servo_settings.rate_per_mille) ||
           (settings->position_speed_limit_per_mille !=
            g_last_logged_servo_settings.position_speed_limit_per_mille);
}

static void remember_logged_servo_status(
    const ServoControllerSettings *settings,
    uint32_t now_ms)
{
    g_last_logged_servo_settings = *settings;
    g_last_servo_status_log_ms = now_ms;
    g_servo_status_log_initialized = 1U;
}

static void log_applied_gc_packet(const GcPacket *packet)
{
    ServoControllerSettings settings;
    uint32_t now_ms = HAL_GetTick();

    servo_controller_get_settings(&settings);
    LOG("[antenna_servo] gc applied: seq=%u gc_mode=%s servo_target=%u servo_rate=%d servo_speed_limit=%d servo_pwm=%uus\r\n",
        (unsigned int)packet->seq,
        gc_packet_type_name(packet->type),
        (unsigned int)settings.target_angle_tenths,
        (int)settings.rate_per_mille,
        (int)settings.position_speed_limit_per_mille,
        (unsigned int)settings.pwm_us);
    remember_logged_servo_status(&settings, now_ms);
}

static void log_servo_status_if_changed(void)
{
    ServoControllerSettings settings;
    uint32_t now_ms = HAL_GetTick();

    servo_controller_get_settings(&settings);
    if (!servo_status_log_should_update(&settings)) {
        return;
    }

    if ((g_servo_status_log_initialized != 0U) &&
        ((now_ms - g_last_servo_status_log_ms) <
         SERVO_STATUS_LOG_INTERVAL_MS)) {
        return;
    }

    LOG("[antenna_servo] servo update: servo_target=%u servo_rate=%d servo_speed_limit=%d servo_pwm=%uus\r\n",
        (unsigned int)settings.target_angle_tenths,
        (int)settings.rate_per_mille,
        (int)settings.position_speed_limit_per_mille,
        (unsigned int)settings.pwm_us);
    remember_logged_servo_status(&settings, now_ms);
}
#else
#define log_reset_message() do { } while (0)
#define log_applied_gc_packet(packet) do { } while (0)
#define log_servo_status_if_changed() do { } while (0)
#endif

void init(void)
{
    log_reset_message();
    board_led_init();
    servo_controller_init();
    rssi_monitor_init();
    gc_packet_receiver_init();
    LOG("[antenna_servo] init complete\r\n");
}

void poll(void)
{
    GcPacket packet;

    gc_packet_receiver_poll();
    while (gc_packet_receiver_pop(&packet)) {
        board_led_on_gc_packet_received();

        switch (packet.type) {
        case GC_PACKET_TYPE_AUTO:
        case GC_PACKET_TYPE_MANUAL_POSITION:
            if (packet.value >= 0) {
                servo_controller_set_position((uint16_t)packet.value);
                servo_controller_poll();
                log_applied_gc_packet(&packet);
            }
            break;

        case GC_PACKET_TYPE_MANUAL_RATE:
            servo_controller_set_rate(packet.value);
            servo_controller_poll();
            log_applied_gc_packet(&packet);
            break;

        case GC_PACKET_TYPE_STOP:
            servo_controller_stop();
            servo_controller_poll();
            log_applied_gc_packet(&packet);
            break;

        case GC_PACKET_TYPE_HOME:
            servo_controller_home();
            servo_controller_poll();
            log_applied_gc_packet(&packet);
            break;

        default:
            break;
        }
    }

    servo_controller_poll();
    log_servo_status_if_changed();
    rssi_monitor_poll();
    board_led_poll();
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
    rssi_monitor_on_capture(htim);
}
