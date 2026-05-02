#include "modules/servo_controller.h"

#include "main.h"
#include "modules/servo_config.h"

#define SERVO_RATE_DELTA_DENOMINATOR 1000000LL

typedef enum
{
    SERVO_MODE_STOPPED = 0,
    SERVO_MODE_POSITION,
    SERVO_MODE_RATE,
} ServoMode;

typedef struct
{
    uint16_t target_angle_tenths;
    int16_t rate_per_mille;
    int16_t position_speed_limit_per_mille;
    int32_t rate_delta_remainder;
    uint32_t last_rate_update_ms;
    uint16_t pwm_us;
    ServoMode mode;
} ServoControllerContext;

extern TIM_HandleTypeDef htim3;
static ServoControllerContext g_servo_controller;

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void set_pwm_us(uint16_t pulse_us)
{
    pulse_us = clamp_u16(pulse_us, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);
    g_servo_controller.pwm_us = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

static uint16_t angle_to_pwm_us(uint16_t angle_tenths)
{
    uint32_t pwm_span = SERVO_PWM_MAX_US - SERVO_PWM_MIN_US;
    uint32_t pulse_offset;

    angle_tenths = clamp_u16(angle_tenths,
                             SERVO_MIN_ANGLE_TENTHS,
                             SERVO_MAX_ANGLE_TENTHS);
    pulse_offset =
        (((uint32_t)angle_tenths * pwm_span) +
         (SERVO_PHYSICAL_RANGE_TENTHS / 2U)) /
        SERVO_PHYSICAL_RANGE_TENTHS;

    return clamp_u16((uint16_t)(SERVO_PWM_MIN_US + pulse_offset),
                     SERVO_PWM_MIN_US,
                     SERVO_PWM_MAX_US);
}

static void apply_target_pwm(void)
{
    set_pwm_us(angle_to_pwm_us(g_servo_controller.target_angle_tenths));
}

static void command_position(uint16_t angle_tenths, int16_t speed_limit_per_mille)
{
    g_servo_controller.target_angle_tenths =
        clamp_u16(angle_tenths,
                  SERVO_MIN_ANGLE_TENTHS,
                  SERVO_MAX_ANGLE_TENTHS);
    g_servo_controller.position_speed_limit_per_mille =
        clamp_i16(speed_limit_per_mille, 0, 1000);
    g_servo_controller.rate_per_mille = 0;
    g_servo_controller.rate_delta_remainder = 0;
    g_servo_controller.mode = SERVO_MODE_POSITION;
    apply_target_pwm();
}

static void poll_position_mode(void)
{
    apply_target_pwm();
}

static void update_rate_target(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - g_servo_controller.last_rate_update_ms;
    int64_t scaled_delta;
    int32_t delta_tenths;
    int32_t target_angle_tenths;

    g_servo_controller.last_rate_update_ms = now_ms;
    if (elapsed_ms == 0U) {
        return;
    }

    scaled_delta =
        ((int64_t)g_servo_controller.rate_per_mille *
         (int64_t)SERVO_MANUAL_RATE_FULL_SCALE_TENTHS_PER_SEC *
         (int64_t)elapsed_ms) +
        (int64_t)g_servo_controller.rate_delta_remainder;
    delta_tenths = (int32_t)(scaled_delta / SERVO_RATE_DELTA_DENOMINATOR);
    g_servo_controller.rate_delta_remainder =
        (int32_t)(scaled_delta % SERVO_RATE_DELTA_DENOMINATOR);

    if (delta_tenths == 0) {
        return;
    }

    target_angle_tenths =
        (int32_t)g_servo_controller.target_angle_tenths + delta_tenths;
    if (target_angle_tenths > (int32_t)SERVO_MAX_ANGLE_TENTHS) {
        target_angle_tenths = (int32_t)SERVO_MAX_ANGLE_TENTHS;
        g_servo_controller.rate_delta_remainder = 0;
    } else if (target_angle_tenths < (int32_t)SERVO_MIN_ANGLE_TENTHS) {
        target_angle_tenths = (int32_t)SERVO_MIN_ANGLE_TENTHS;
        g_servo_controller.rate_delta_remainder = 0;
    }

    g_servo_controller.target_angle_tenths = (uint16_t)target_angle_tenths;
}

static void poll_rate_mode(uint32_t now_ms)
{
    update_rate_target(now_ms);
    apply_target_pwm();
}

void servo_controller_init(void)
{
    g_servo_controller.target_angle_tenths = SERVO_HOME_ANGLE_TENTHS;
    g_servo_controller.rate_per_mille = 0;
    g_servo_controller.position_speed_limit_per_mille =
        SERVO_POSITION_HOME_SPEED_PER_MILLE;
    g_servo_controller.rate_delta_remainder = 0;
    g_servo_controller.last_rate_update_ms = HAL_GetTick();
    g_servo_controller.pwm_us = angle_to_pwm_us(SERVO_HOME_ANGLE_TENTHS);
    g_servo_controller.mode = SERVO_MODE_POSITION;

    apply_target_pwm();
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

void servo_controller_poll(void)
{
    uint32_t now_ms = HAL_GetTick();

    switch (g_servo_controller.mode) {
    case SERVO_MODE_POSITION:
        poll_position_mode();
        break;

    case SERVO_MODE_RATE:
        poll_rate_mode(now_ms);
        break;

    case SERVO_MODE_STOPPED:
    default:
        apply_target_pwm();
        break;
    }
}

void servo_controller_set_position(uint16_t angle_tenths)
{
    command_position(angle_tenths, SERVO_POSITION_SPEED_LIMIT_PER_MILLE);
}

void servo_controller_set_rate(int16_t rate_per_mille)
{
    uint32_t now_ms = HAL_GetTick();

    if (g_servo_controller.mode == SERVO_MODE_RATE) {
        update_rate_target(now_ms);
    }

    g_servo_controller.rate_per_mille =
        clamp_i16(rate_per_mille, -1000, 1000);
    g_servo_controller.last_rate_update_ms = now_ms;
    g_servo_controller.rate_delta_remainder = 0;

    if (g_servo_controller.rate_per_mille == 0) {
        servo_controller_stop();
        return;
    }

    g_servo_controller.mode = SERVO_MODE_RATE;
    apply_target_pwm();
}

void servo_controller_stop(void)
{
    g_servo_controller.rate_per_mille = 0;
    g_servo_controller.rate_delta_remainder = 0;
    g_servo_controller.mode = SERVO_MODE_STOPPED;
    apply_target_pwm();
}

void servo_controller_home(void)
{
    command_position(SERVO_HOME_ANGLE_TENTHS,
                     SERVO_POSITION_HOME_SPEED_PER_MILLE);
}

void servo_controller_get_settings(ServoControllerSettings *settings)
{
    if (settings == NULL) {
        return;
    }

    settings->target_angle_tenths = g_servo_controller.target_angle_tenths;
    settings->rate_per_mille = g_servo_controller.rate_per_mille;
    settings->position_speed_limit_per_mille =
        g_servo_controller.position_speed_limit_per_mille;
    settings->pwm_us = g_servo_controller.pwm_us;
}
