#include <Arduino.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#define HOSE1_MASK      (1 << PD2)
#define COUNT_LED_MASK  ((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3))

#define DEBOUNCE_MS          20
#define AXLE_INTERVAL_MAX_MS 1000
#define VEHICLE_COUNT_MAX    15

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

    DDRC |= COUNT_LED_MASK;
    PORTC &= ~COUNT_LED_MASK;
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

// two axles within the interval make one vehicle, a late second axle starts over
static bool vehicle_passed(bool axle, uint32_t now_ms)
{
    static bool     awaiting_second_axle = false;
    static uint32_t first_axle_ms = 0;

    if (!axle)
    {
        return false;
    }

    if (awaiting_second_axle && now_ms - first_axle_ms < AXLE_INTERVAL_MAX_MS)
    {
        awaiting_second_axle = false;
        return true;
    }

    awaiting_second_axle = true;
    first_axle_ms = now_ms;
    return false;
}

static void display_counter(uint8_t count)
{
    PORTC = (PORTC & ~COUNT_LED_MASK) | (count & COUNT_LED_MASK);
}

int main(void)
{
    button_t hose1 = { HOSE1_MASK, false, false, 0 };
    uint8_t  vehicle_count = 0;

    init(); // arduino core, needed for millis()
    initialize_io();

    for (;;)
    {
        uint32_t now_ms = millis();

        if (vehicle_passed(axle_detected(&hose1, now_ms), now_ms))
        {
            vehicle_count++;
            if (vehicle_count > VEHICLE_COUNT_MAX)
            {
                vehicle_count = 0;
            }
        }

        display_counter(vehicle_count);
    }

    return 0;
}
