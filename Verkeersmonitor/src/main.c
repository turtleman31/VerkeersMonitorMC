#include <Arduino.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#define HOSE1_MASK  (1 << PD2)

#define DEBOUNCE_MS 20

typedef struct
{
    uint8_t  pin_mask;
    bool     pressed;
    bool     raw_pressed;
    uint32_t last_change_ms;
} button_t;

static void initialize_io(void)
{
    DDRD &= ~HOSE1_MASK;
    PORTD |= HOSE1_MASK; // pull-up, so pressed reads low
}

// true when the debounced state changed
static bool button_state(button_t *button, uint32_t now_ms)
{
    bool raw_pressed = !(PIND & button->pin_mask);

    if (raw_pressed != button->raw_pressed)
    {
        button->raw_pressed = raw_pressed;
        button->last_change_ms = now_ms;
        return false;
    }

    if (raw_pressed == button->pressed || now_ms - button->last_change_ms < DEBOUNCE_MS)
    {
        return false;
    }

    button->pressed = raw_pressed;
    return true;
}

// an axle counts when the hose is released again
static bool axle_detected(button_t *hose, uint32_t now_ms)
{
    return button_state(hose, now_ms) && !hose->pressed;
}

int main(void)
{
    button_t hose1 = { HOSE1_MASK, false, false, 0 };

    init(); // arduino core, needed for millis()
    initialize_io();

    for (;;)
    {
        uint32_t now_ms = millis();

        axle_detected(&hose1, now_ms);
    }

    return 0;
}
