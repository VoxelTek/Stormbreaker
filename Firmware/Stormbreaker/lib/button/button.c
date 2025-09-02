#include "button.h"

#include <stdbool.h>

void button_init(struct button *btn, PORT_t *port, uint8_t pin,
                 button_press_callback press_callback, button_hold_callback hold_callback)
{
    btn->port           = port;
    btn->pin            = pin;
    btn->press_callback = press_callback;
    btn->hold_callback  = hold_callback;
    btn->button_state   = BUTTON_RELEASED;

    // Set the button pin as an input, with pull-up enabled
    btn->port->DIRCLR = (1 << btn->pin);
    (&btn->port->PIN0CTRL)[btn->pin] |= PORT_PULLUPEN_bm;
}

void button_update(struct button *btn, uint32_t millis)
{
    // Read the current state of the power button (active low)
    bool gpio_state = btn->port->IN & (1 << btn->pin);

    // Reset hold time and flags if the button state has changed
    if (gpio_state != btn->last_gpio_state) {
        btn->last_millis  = millis;
        btn->button_state = BUTTON_RELEASED;
    }

    // If the button is down, check for hold time thresholds
    if (gpio_state == false) {
        // Raise event flags when hold time thresholds are reached
        if (millis - btn->last_millis >= BTN_PRESS_MS && btn->button_state < BUTTON_PRESSED) {
            if (btn->press_callback) {
                btn->press_callback();
            }

            btn->button_state = BUTTON_PRESSED;
        } else if (millis - btn->last_millis >= BTN_HOLD_MS && btn->button_state < BUTTON_HELD) {
            if (btn->hold_callback) {
                btn->hold_callback();
            }

            btn->button_state = BUTTON_HELD;
        }
    }

    // Save the current state for comparison next time
    btn->last_gpio_state = gpio_state;
}