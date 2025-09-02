#pragma once

#include <avr/io.h>
#include <stdint.h>

// Button timing
static const uint16_t BTN_PRESS_MS = 10;
static const uint16_t BTN_HOLD_MS  = 2000;

// Button state
enum button_state { BUTTON_RELEASED, BUTTON_PRESSED, BUTTON_HELD };

// Callbacks
typedef void (*button_press_callback)(void);
typedef void (*button_hold_callback)(void);

// Button struct
struct button {
    PORT_t *port;
    uint8_t pin;
    uint8_t last_gpio_state;
    uint32_t last_millis;
    enum button_state button_state;
    button_press_callback press_callback;
    button_hold_callback hold_callback;
};

// Initialize button
void button_init(struct button *btn, PORT_t *port, uint8_t pin,
                 button_press_callback press_callback, button_hold_callback hold_callback);

// Update button state
void button_update(struct button *btn, uint32_t millis);