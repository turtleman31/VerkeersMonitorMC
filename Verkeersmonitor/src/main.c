#include <Arduino.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#define HOSE1_MASK          (1 << PD2)
#define COUNT_LED_MASK      ((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3))
#define SEGMENT_AF_MASK     ((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5))
#define SEGMENT_G_MASK      (1 << PC4)
#define DECIMAL_POINT_MASK  (1 << PD4)
#define DIGIT_MASK          ((1 << PD5) | (1 << PD6) | (1 << PD7))

#define DEBOUNCE_MS          20
#define AXLE_INTERVAL_MAX_MS 1000
#define VEHICLE_COUNT_MAX    15
#define MULTIPLEX_MS         2
#define DIGIT_COUNT          3
#define UNITS_DIGIT          1

#define SEG_A (1 << 0)
#define SEG_B (1 << 1)
#define SEG_C (1 << 2)
#define SEG_D (1 << 3)
#define SEG_E (1 << 4)
#define SEG_F (1 << 5)
#define SEG_G (1 << 6)

static const uint8_t DIGIT_PATTERNS[] =
{
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
    SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
    SEG_B | SEG_C | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
};
static const uint8_t  DIGIT_PINS[DIGIT_COUNT]     = { 1 << PD5, 1 << PD6, 1 << PD7 };
static const uint16_t DIGIT_DIVISORS[DIGIT_COUNT] = { 100, 10, 1 };

typedef struct
{
    uint8_t  pin_mask;
    bool     pressed;
    bool     raw_pressed;
    uint32_t last_change_ms;
} button_t;

typedef struct
{
    uint16_t value;
    bool     blank;
    uint8_t  digit;
    uint32_t last_step_ms;
} display_t;

static void initialize_io(void)
{
    DDRD &= ~HOSE1_MASK;
    PORTD |= HOSE1_MASK; // pull-up, so pressed reads low

    DDRC |= COUNT_LED_MASK;
    PORTC &= ~COUNT_LED_MASK;

    DDRB |= SEGMENT_AF_MASK;
    DDRC |= SEGMENT_G_MASK;
    DDRD |= DECIMAL_POINT_MASK | DIGIT_MASK;
    PORTD |= DIGIT_MASK; // common cathode, high is off
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

static void display_speed(display_t *display, uint16_t speed_x10)
{
    display->value = speed_x10;
    display->blank = false;
}

static void set_segments(uint8_t pattern, bool decimal_point)
{
    PORTB = (PORTB & ~SEGMENT_AF_MASK) | (pattern & SEGMENT_AF_MASK); // a..f sit on PB0..PB5 in order
    PORTC = (PORTC & ~SEGMENT_G_MASK) | ((pattern & SEG_G) ? SEGMENT_G_MASK : 0);
    PORTD = (PORTD & ~DECIMAL_POINT_MASK) | (decimal_point ? DECIMAL_POINT_MASK : 0);
}

// one digit lit at a time, switching every 2 ms is fast enough to look steady
static void display_refresh(display_t *display, uint32_t now_ms)
{
    if (now_ms - display->last_step_ms < MULTIPLEX_MS)
    {
        return;
    }
    display->last_step_ms = now_ms;

    PORTD |= DIGIT_MASK;

    display->digit = (display->digit + 1) % DIGIT_COUNT;
    uint8_t number = display->value / DIGIT_DIVISORS[display->digit] % 10;
    bool leading_zero = display->digit == 0 && number == 0;

    if (display->blank || leading_zero)
    {
        return;
    }

    set_segments(DIGIT_PATTERNS[number], display->digit == UNITS_DIGIT);
    PORTD &= ~DIGIT_PINS[display->digit];
}

int main(void)
{
    button_t  hose1 = { HOSE1_MASK, false, false, 0 };
    display_t display = { 0, true, 0, 0 };
    uint8_t   vehicle_count = 0;

    init(); // arduino core, needed for millis()
    initialize_io();
    display_speed(&display, 123); // test value until the speed measurement exists

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
        display_refresh(&display, now_ms);
    }

    return 0;
}
