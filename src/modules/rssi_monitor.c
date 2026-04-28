#include "modules/rssi_monitor.h"

#include "debug_log.h"
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define RSSI_LOG_INTERVAL_MS          1000U
#define RSSI_STALE_TIMEOUT_MS         1500U
#define RSSI_MIN_PERIOD_US            100U
#define RSSI_MAX_PERIOD_US            12000U
#define RSSI_XR_MIN_PERIOD_US         800U
#define RSSI_XR_MAX_PERIOD_US         1200U
#define RSSI_900HP_MIN_PERIOD_US      7000U
#define RSSI_900HP_MAX_PERIOD_US      9500U
#define RSSI_900HP_ZERO_DB_PER_MILLE  325U
#define RSSI_900HP_PER_MILLE_PER_DB   15U

typedef struct
{
    volatile uint32_t period_us;
    volatile uint32_t high_us;
    volatile uint32_t last_capture_ms;
    volatile uint32_t capture_count;
    volatile bool valid;
} RssiMonitorContext;

static RssiMonitorContext g_rssi_monitor;

extern TIM_HandleTypeDef htim2;

#if DEBUG_LOG_ENABLED
typedef struct
{
    uint32_t period_us;
    uint32_t high_us;
    uint32_t last_capture_ms;
    uint32_t capture_count;
    bool valid;
} RssiSnapshot;
#endif

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

static bool period_is_in_range(uint32_t period_us)
{
    return (period_us >= RSSI_MIN_PERIOD_US) &&
           (period_us <= RSSI_MAX_PERIOD_US);
}

#if DEBUG_LOG_ENABLED
static uint32_t calculate_duty_per_mille(uint32_t period_us, uint32_t high_us)
{
    if (period_us == 0U) {
        return 0U;
    }

    return ((high_us * 1000U) + (period_us / 2U)) / period_us;
}

static bool period_matches_xbee_xr(uint32_t period_us)
{
    return (period_us >= RSSI_XR_MIN_PERIOD_US) &&
           (period_us <= RSSI_XR_MAX_PERIOD_US);
}

static bool period_matches_xbee_900hp(uint32_t period_us)
{
    return (period_us >= RSSI_900HP_MIN_PERIOD_US) &&
           (period_us <= RSSI_900HP_MAX_PERIOD_US);
}

static int32_t estimate_xbee_xr_dbm(uint32_t duty_per_mille)
{
    return -((int32_t)((duty_per_mille + 5U) / 10U));
}

static uint32_t estimate_xbee_900hp_margin_db(uint32_t duty_per_mille)
{
    if (duty_per_mille <= RSSI_900HP_ZERO_DB_PER_MILLE) {
        return 0U;
    }

    return (duty_per_mille - RSSI_900HP_ZERO_DB_PER_MILLE +
            (RSSI_900HP_PER_MILLE_PER_DB / 2U)) /
           RSSI_900HP_PER_MILLE_PER_DB;
}

static RssiSnapshot take_snapshot(void)
{
    RssiSnapshot snapshot;
    uint32_t primask = enter_critical_section();

    snapshot.period_us = g_rssi_monitor.period_us;
    snapshot.high_us = g_rssi_monitor.high_us;
    snapshot.last_capture_ms = g_rssi_monitor.last_capture_ms;
    snapshot.capture_count = g_rssi_monitor.capture_count;
    snapshot.valid = g_rssi_monitor.valid;

    exit_critical_section(primask);
    return snapshot;
}
#endif

static void start_input_capture_if_ready(uint32_t channel)
{
    if (HAL_TIM_GetChannelState(&htim2, channel) != HAL_TIM_CHANNEL_STATE_READY) {
        return;
    }

    if (HAL_TIM_IC_Start_IT(&htim2, channel) != HAL_OK) {
        Error_Handler();
    }
}

void rssi_monitor_init(void)
{
    uint32_t primask = enter_critical_section();

    g_rssi_monitor.period_us = 0U;
    g_rssi_monitor.high_us = 0U;
    g_rssi_monitor.last_capture_ms = 0U;
    g_rssi_monitor.capture_count = 0U;
    g_rssi_monitor.valid = false;

    exit_critical_section(primask);

    start_input_capture_if_ready(TIM_CHANNEL_1);
    start_input_capture_if_ready(TIM_CHANNEL_2);
}

void rssi_monitor_poll(void)
{
#if DEBUG_LOG_ENABLED
    static uint32_t last_log_ms = 0U;
    RssiSnapshot snapshot;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms;
    uint32_t duty_per_mille;

    if ((now_ms - last_log_ms) < RSSI_LOG_INTERVAL_MS) {
        return;
    }
    last_log_ms = now_ms;

    snapshot = take_snapshot();
    age_ms = now_ms - snapshot.last_capture_ms;

    if (!snapshot.valid || (age_ms > RSSI_STALE_TIMEOUT_MS)) {
        LOG("[antenna_servo] rssi: no fresh pwm age=%lums captures=%lu\r\n",
            (unsigned long)age_ms,
            (unsigned long)snapshot.capture_count);
        return;
    }

    duty_per_mille =
        calculate_duty_per_mille(snapshot.period_us, snapshot.high_us);

    if (period_matches_xbee_xr(snapshot.period_us)) {
        LOG("[antenna_servo] rssi: period=%luus high=%luus duty=%lu.%lu%% xr_est=%lddBm age=%lums\r\n",
            (unsigned long)snapshot.period_us,
            (unsigned long)snapshot.high_us,
            (unsigned long)(duty_per_mille / 10U),
            (unsigned long)(duty_per_mille % 10U),
            (long)estimate_xbee_xr_dbm(duty_per_mille),
            (unsigned long)age_ms);
    } else if (period_matches_xbee_900hp(snapshot.period_us)) {
        LOG("[antenna_servo] rssi: period=%luus high=%luus duty=%lu.%lu%% 900hp_margin=%ludB age=%lums\r\n",
            (unsigned long)snapshot.period_us,
            (unsigned long)snapshot.high_us,
            (unsigned long)(duty_per_mille / 10U),
            (unsigned long)(duty_per_mille % 10U),
            (unsigned long)estimate_xbee_900hp_margin_db(duty_per_mille),
            (unsigned long)age_ms);
    } else {
        LOG("[antenna_servo] rssi: period=%luus high=%luus duty=%lu.%lu%% age=%lums\r\n",
            (unsigned long)snapshot.period_us,
            (unsigned long)snapshot.high_us,
            (unsigned long)(duty_per_mille / 10U),
            (unsigned long)(duty_per_mille % 10U),
            (unsigned long)age_ms);
    }
#endif
}

void rssi_monitor_on_capture(TIM_HandleTypeDef *htim)
{
    uint32_t period_us;
    uint32_t high_us;

    if ((htim->Instance != TIM2) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    period_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    high_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (!period_is_in_range(period_us) || (high_us > period_us)) {
        return;
    }

    g_rssi_monitor.period_us = period_us;
    g_rssi_monitor.high_us = high_us;
    g_rssi_monitor.last_capture_ms = HAL_GetTick();
    g_rssi_monitor.capture_count++;
    g_rssi_monitor.valid = true;
}
