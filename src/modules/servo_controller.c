#include "modules/servo_controller.h"

#include "main.h"
#include "modules/servo_config.h"

#include <stdbool.h>
#include <stdlib.h>

typedef enum
{
    SERVO_MODE_STOPPED = 0,
    SERVO_MODE_POSITION,
    SERVO_MODE_RATE,
} ServoMode;

typedef struct
{
    volatile uint16_t angle_tenths;
    volatile uint32_t last_feedback_ms;
    volatile bool feedback_valid;
    uint16_t target_angle_tenths;
    int16_t rate_per_mille;
    int16_t position_speed_limit_per_mille;
    ServoMode mode;
} ServoControllerContext;

static ServoControllerContext g_servo_controller;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

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

static bool feedback_is_fresh(uint32_t now_ms)
{
    return g_servo_controller.feedback_valid &&
           ((now_ms - g_servo_controller.last_feedback_ms) <=
            SERVO_FEEDBACK_STALE_TIMEOUT_MS);
}

static void set_pwm_us(uint16_t pulse_us)
{
    pulse_us = clamp_u16(pulse_us, SERVO_PWM_MIN_US, SERVO_PWM_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

static void drive_stop(void)
{
    set_pwm_us(SERVO_PWM_NEUTRAL_US);
}

static void drive_rate_per_mille(int16_t rate_per_mille)
{
    int32_t pulse_us;
    int32_t offset_us;

    rate_per_mille = clamp_i16(rate_per_mille, -1000, 1000);
    rate_per_mille = (int16_t)(rate_per_mille * SERVO_CONTROL_DIRECTION);

    offset_us = ((int32_t)SERVO_PWM_FULL_SPEED_OFFSET_US *
                 (int32_t)rate_per_mille) / 1000;
    pulse_us = (int32_t)SERVO_PWM_NEUTRAL_US + offset_us;

    set_pwm_us((uint16_t)clamp_i16((int16_t)pulse_us,
                                   (int16_t)SERVO_PWM_MIN_US,
                                   (int16_t)SERVO_PWM_MAX_US));
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
    g_servo_controller.mode = SERVO_MODE_POSITION;
}

static uint16_t calculate_feedback_angle(uint32_t period_us, uint32_t high_us)
{
    uint32_t duty_per_mille;
    uint32_t angle_tenths;
    uint32_t duty_span;

    duty_per_mille = ((high_us * 1000U) + (period_us / 2U)) / period_us;

    if (duty_per_mille < SERVO_FEEDBACK_DUTY_MIN_PER_MILLE) {
        duty_per_mille = SERVO_FEEDBACK_DUTY_MIN_PER_MILLE;
    } else if (duty_per_mille > SERVO_FEEDBACK_DUTY_MAX_PER_MILLE) {
        duty_per_mille = SERVO_FEEDBACK_DUTY_MAX_PER_MILLE;
    }

    duty_span = SERVO_FEEDBACK_DUTY_MAX_PER_MILLE -
                SERVO_FEEDBACK_DUTY_MIN_PER_MILLE;
    angle_tenths =
        ((duty_per_mille - SERVO_FEEDBACK_DUTY_MIN_PER_MILLE) *
         SERVO_FEEDBACK_FULL_SCALE_TENTHS +
         (duty_span / 2U)) / duty_span;

    if (angle_tenths >= SERVO_FEEDBACK_FULL_SCALE_TENTHS) {
        angle_tenths = SERVO_FEEDBACK_FULL_SCALE_TENTHS - 1U;
    }

#if SERVO_FEEDBACK_INVERTED
    angle_tenths = (SERVO_FEEDBACK_FULL_SCALE_TENTHS - 1U) - angle_tenths;
#endif

    return (uint16_t)angle_tenths;
}

static int16_t calculate_position_command(int16_t error_tenths,
                                          int16_t speed_limit_per_mille)
{
    int16_t command_per_mille;
    int16_t abs_error = (int16_t)abs(error_tenths);

    command_per_mille =
        (int16_t)(abs_error * SERVO_POSITION_KP_PER_TENTH);
    command_per_mille =
        clamp_i16(command_per_mille,
                  SERVO_POSITION_MIN_COMMAND_PER_MILLE,
                  speed_limit_per_mille);

    if (error_tenths < 0) {
        command_per_mille = (int16_t)-command_per_mille;
    }

    return command_per_mille;
}

static void poll_position_mode(uint32_t now_ms)
{
    int16_t error_tenths;

    if (!feedback_is_fresh(now_ms)) {
        drive_stop();
        return;
    }

    error_tenths =
        (int16_t)g_servo_controller.target_angle_tenths -
        (int16_t)g_servo_controller.angle_tenths;

    if (abs(error_tenths) <= SERVO_POSITION_TOLERANCE_TENTHS) {
        drive_stop();
        g_servo_controller.mode = SERVO_MODE_STOPPED;
        return;
    }

    drive_rate_per_mille(calculate_position_command(
        error_tenths,
        g_servo_controller.position_speed_limit_per_mille));
}

static void poll_rate_mode(uint32_t now_ms)
{
    if (!feedback_is_fresh(now_ms)) {
        drive_stop();
        return;
    }

    if ((g_servo_controller.rate_per_mille > 0) &&
        (g_servo_controller.angle_tenths >= SERVO_MAX_ANGLE_TENTHS)) {
        drive_stop();
        return;
    }

    if ((g_servo_controller.rate_per_mille < 0) &&
        (g_servo_controller.angle_tenths <= SERVO_MIN_ANGLE_TENTHS)) {
        drive_stop();
        return;
    }

    drive_rate_per_mille(g_servo_controller.rate_per_mille);
}

void servo_controller_init(void)
{
    g_servo_controller.angle_tenths = SERVO_HOME_ANGLE_TENTHS;
    g_servo_controller.last_feedback_ms = 0U;
    g_servo_controller.feedback_valid = false;
    g_servo_controller.target_angle_tenths = SERVO_HOME_ANGLE_TENTHS;
    g_servo_controller.rate_per_mille = 0;
    g_servo_controller.position_speed_limit_per_mille =
        SERVO_POSITION_HOME_SPEED_PER_MILLE;
    g_servo_controller.mode = SERVO_MODE_POSITION;

    set_pwm_us(SERVO_PWM_NEUTRAL_US);
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    servo_controller_home();
}

void servo_controller_poll(void)
{
    uint32_t now_ms = HAL_GetTick();

    switch (g_servo_controller.mode) {
    case SERVO_MODE_POSITION:
        poll_position_mode(now_ms);
        break;

    case SERVO_MODE_RATE:
        poll_rate_mode(now_ms);
        break;

    case SERVO_MODE_STOPPED:
    default:
        drive_stop();
        break;
    }
}

void servo_controller_set_position(uint16_t angle_tenths)
{
    command_position(angle_tenths, SERVO_POSITION_SPEED_LIMIT_PER_MILLE);
}

void servo_controller_set_rate(int16_t rate_per_mille)
{
    g_servo_controller.rate_per_mille =
        clamp_i16(rate_per_mille, -1000, 1000);

    if (g_servo_controller.rate_per_mille == 0) {
        servo_controller_stop();
        return;
    }

    if (g_servo_controller.feedback_valid) {
        g_servo_controller.target_angle_tenths =
            g_servo_controller.angle_tenths;
    }

    g_servo_controller.mode = SERVO_MODE_RATE;
}

void servo_controller_stop(void)
{
    if (g_servo_controller.feedback_valid) {
        g_servo_controller.target_angle_tenths =
            g_servo_controller.angle_tenths;
    }

    g_servo_controller.rate_per_mille = 0;
    g_servo_controller.mode = SERVO_MODE_STOPPED;
    drive_stop();
}

void servo_controller_home(void)
{
    command_position(SERVO_HOME_ANGLE_TENTHS,
                     SERVO_POSITION_HOME_SPEED_PER_MILLE);
}

void servo_controller_on_feedback_capture(TIM_HandleTypeDef *htim)
{
    uint32_t period_us;
    uint32_t high_us;

    if ((htim->Instance != TIM2) ||
        (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) {
        return;
    }

    period_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    high_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if ((period_us < SERVO_FEEDBACK_MIN_PERIOD_US) ||
        (period_us > SERVO_FEEDBACK_MAX_PERIOD_US) ||
        (high_us > period_us)) {
        return;
    }

    g_servo_controller.angle_tenths =
        calculate_feedback_angle(period_us, high_us);
    g_servo_controller.last_feedback_ms = HAL_GetTick();
    g_servo_controller.feedback_valid = true;
}
