#include "modules/board_led.h"

#include "main.h"

#include <stdbool.h>

#define BOARD_LED_HOLD_MS 120U

typedef struct
{
    uint32_t lit_until_ms;
    bool is_lit;
} BoardLedContext;

static BoardLedContext g_board_led;

static void set_led(bool lit)
{
    HAL_GPIO_WritePin(BOARD_LED_GPIO_Port,
                      BOARD_LED_Pin,
                      lit ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_board_led.is_lit = lit;
}

void board_led_init(void)
{
    g_board_led.lit_until_ms = 0U;
    set_led(false);
}

void board_led_poll(void)
{
    if (g_board_led.is_lit &&
        ((int32_t)(HAL_GetTick() - g_board_led.lit_until_ms) >= 0)) {
        set_led(false);
    }
}

void board_led_on_gc_packet_received(void)
{
    g_board_led.lit_until_ms = HAL_GetTick() + BOARD_LED_HOLD_MS;
    set_led(true);
}
