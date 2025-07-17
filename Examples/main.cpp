/**
 * example.cpp
 *
 * Demonstrates the use of avr_easyio.h on an ATmega‑based board.
 * Button on PD7 controls an LED’s brightness on PB2 (OC1B, Timer1) via PWM;
 * additionally, the code samples the analog value on PC0 (ADC0) every 50 ms
 * (e.g., for a potentiometer) to adjust the PWM duty cycle.
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

    // Initialize LED on PB2 (PWM-capable, OC1B) to off
    setPWM(PWM_PINB2, 0);

    // Optional: set up UART here to print analog values, omitted for brevity

    uint8_t  button = 0;
    uint16_t potVal = 0;
    uint8_t  pwmVal = 0;

    while (1)
    {
        // Read button state (active‑low)
        button = digitalRead(7, &DDRD, &PIND);

        // Read potentiometer on PC0 (ADC0)
        potVal = analogRead(0, &DDRC, &PORTC);

        // Map 10-bit ADC (0–1023) to 8-bit PWM (0–255)
        pwmVal = potVal >> 2; // Divide by 4 (1024 / 4 = 256)

        // Adjust PWM based on button state
        if (button == 0)
        {
            setPWM(PWM_PINB2, pwmVal); // LED brightness from potentiometer
        }
        else
        {
            setPWM(PWM_PINB2, 64); // LED at ~25% brightness
        }

        _delay_ms(50);
    }
}
