/**
 * avr_easyio.h
 * Lightweight GPIO and ADC utilities for AVR bare-metal development
 *
 * Author: Subrata
 * Date: 2025-07-12
 * Licence: MIT
 */

#ifndef AVR_EASYIO_H
#define AVR_EASYIO_H

#include <avr/io.h>
#include <stdint.h>

// Pin mode constants
#define OUTPUT 0
#define INPUT 1
#define INPUT_PULLUP 2

/**
 * Configure a pin as INPUT, INPUT_PULLUP, or OUTPUT.
 * @param pin   Bit position within the port register (0–7)
 * @param ddr   Pointer to DDRx register for the port
 * @param port  Pointer to PORTx register for the port (used for pull‑ups)
 * @param mode  One of OUTPUT / INPUT / INPUT_PULLUP
 */
static inline void pinMode(uint8_t pin,
                           volatile uint8_t *ddr,
                           volatile uint8_t *port,
                           uint8_t mode)
{
    if (mode == OUTPUT)
    {
        *ddr |= (1 << pin);  // Output
    }
    else
    {
        *ddr &= ~(1 << pin); // Input
        if (mode == INPUT_PULLUP)
        {
            *port |= (1 << pin);  // Enable pull‑up
        }
        else
        {
            *port &= ~(1 << pin); // High‑Z
        }
    }
}

/**
 * Drive a pin HIGH or LOW (forces pin to OUTPUT first).
 */
static inline void digitalWrite(uint8_t pin,
                                volatile uint8_t *ddr,
                                volatile uint8_t *port,
                                uint8_t value)
{
    *ddr |= (1 << pin); // ensure output

    if (value)
    {
        *port |= (1 << pin);
    }
    else
    {
        *port &= ~(1 << pin);
    }
}

/**
 * Read a digital pin.
 * The *ddr parameter is ignored but kept for interface symmetry.
 */
static inline uint8_t digitalRead(uint8_t pin,
                                  volatile uint8_t *ddr,
                                  volatile uint8_t *pin_reg)
{
    (void)ddr; // unused
    return (*pin_reg & (1 << pin)) ? 1 : 0;
}

/**
 * Perform a single analog‑to‑digital conversion on the specified ADC channel
 * (only 0‑5 on ATmega328P). The pin is automatically set to input and its
 * pull‑up disabled. Uses AVcc (5 V) as reference and a prescaler of 128 →
 * 62.5 kHz conversion clock at F_CPU = 8 MHz.
 *
 * @returns 10‑bit ADC result (0–1023); returns 0 if the channel is invalid.
 */
static inline uint16_t analogRead(uint8_t pin,
                                  volatile uint8_t *ddr,
                                  volatile uint8_t *port)
{
    if (pin > 5)
        return 0;

    // set pin as input and disable pull‑up
    *ddr &= ~(1 << pin);
    *port &= ~(1 << pin);

    // disable digital input buffer for this channel
    DIDR0 |= (1 << pin);

    // select AVcc reference and channel
    ADMUX = (1 << REFS0) | (pin & 0x0F);

    // enable ADC and set prescaler to 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // start conversion
    ADCSRA |= (1 << ADSC);

    // wait until complete
    while (ADCSRA & (1 << ADSC))
        ;

    return ADC;
}

#endif // AVR_EASYIO_H

