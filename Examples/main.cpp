/**
 * example.cpp
 *
 * Demonstrates the use of avr_easyio.h on an ATmega‑based board.
 * Button on PD7 controls an LED on PD6; additionally, the code samples the
 * analog value on PC0 (ADC0) every 50 ms (e.g., for a potentiometer).
 *
 * Build (avr‑gcc):
 *   avr-gcc -mmcu=atmega328p -DF_CPU=8000000UL -Os example.cpp -o example.elf
 *   avr-objcopy -O ihex example.elf example.hex
 *   avrdude -p m328p -c usbasp -U flash:w:example.hex
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "avr_easyio.h"

int main(void)
{
    // Configure PD7 as input with pull‑up (button)
    pinMode(7, &DDRD, &PORTD, INPUT_PULLUP);

    // Ensure LED on PD6 is output and LOW at start
    digitalWrite(6, &DDRD, &PORTD, 0);

    // Optional: set up UART here to print analog values, omitted for brevity

    uint8_t  button = 0;
    uint16_t potVal = 0;

    while (1)
    {
        // Read button state (active‑low)
        button = digitalRead(7, &DDRD, &PIND);

        if (button == 0)
        {
            digitalWrite(6, &DDRD, &PORTD, 0); // LED off
        }
        else
        {
            digitalWrite(6, &DDRD, &PORTD, 1); // LED on
        }

        // Read potentiometer on PC0 (ADC0)
        potVal = analogRead(0, &DDRC, &PORTC);

        _delay_ms(50);
    }
}

