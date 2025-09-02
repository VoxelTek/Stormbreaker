#include <avr/io.h>
#include <stdio.h>

#include "console.h"

// Calculate the USART baud rate register value
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

// Print a character to the USART
static int usart_putchar(char c, FILE *stream)
{
    // Wait for the transmit buffer to be empty
    while (!(USART0.STATUS & USART_DREIF_bm))
        ;

    // Put the character in the transmit buffer
    USART0.TXDATAL = c;

    return 0;
}

// Initialize a stdio stream for the USART
static FILE USART_stream = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

void console_init(uint32_t baud_rate)
{
    // Set TX pin as output
    PORTB.DIR |= PIN2_bm;

    // Set the baud rate
    USART0.BAUD = (uint16_t)USART0_BAUD_RATE(baud_rate);

    // Enable the USART transmitter
    USART0.CTRLB |= USART_TXEN_bm;

    // Attach stdout to the USART stream
    stdout = &USART_stream;
}