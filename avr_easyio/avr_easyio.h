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

// PWM pin constants
typedef enum {
    PWM_PIND3 = (1 << 0),   // 0x01, PIND3 (OC2B, Timer2), ~976Hz
    PWM_PIND5 = (1 << 1),   // 0x02, PIND5 (OC0B, Timer0), ~486Hz
    PWM_PIND6 = (1 << 2),   // 0x04, PIND6 (OC0A, Timer0), ~486Hz
    PWM_PINB1 = (1 << 3),   // 0x08, PINB1 (OC1A, Timer1), ~1kHz
    PWM_PINB2 = (1 << 4),   // 0x10, PINB2 (OC1B, Timer1), ~1kHz
    PWM_PINB3 = (1 << 5)    // 0x20, PINB3 (OC2A, Timer2), ~976Hz
} pwm_pin_t;

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

/**
 * Configure PWM on a specified pin using Timer0, Timer1, or Timer2 (ATmega328P).
 * Sets the pin as OUTPUT and configures fast PWM mode. Timer1 uses a TOP value of 999
 * for ~1 kHz frequency (F_CPU = 8 MHz, prescaler = 8). Timer0 and Timer2 use a
 * prescaler of 32 for ~486 Hz and ~976 Hz, respectively.
 *
 * @param pin      PWM pin identifier (pwm_pin_t enum)
 * @param pwmVal   Duty cycle (0–255, where 255 is 100%)
 * @returns        0 on success, 1 if invalid pin
 */
static inline uint8_t setPWM(pwm_pin_t pin, uint8_t pwmVal)
{
    if (pwmVal > 255) {
        pwmVal = 255;
    }

    switch (pin) {
        case PWM_PIND3:
            DDRD |= (1 << DDD3); // Set PIND3 as output
            TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, TOP = 0xFF
            TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler = 32
            OCR2B = pwmVal;
            break;
        case PWM_PIND5:
            DDRD |= (1 << DDD5); // Set PIND5 as output
            TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, TOP = 0xFF
            TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler = 32
            OCR0B = pwmVal;
            break;
        case PWM_PIND6:
            DDRD |= (1 << DDD6); // Set PIND6 as output
            TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast PWM, TOP = 0xFF
            TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler = 32
            OCR0A = pwmVal;
            break;
        case PWM_PINB1:
            DDRB |= (1 << DDB1); // Set PINB1 as output
            TCCR1A = (1 << COM1A1) | (1 << WGM11); // Fast PWM, TOP = ICR1
            TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
            ICR1 = 999; // TOP for 1 kHz
            OCR1A = (uint16_t)((pwmVal * 999UL) / 255); // Scale to TOP = 999
            break;
        case PWM_PINB2:
            DDRB |= (1 << DDB2); // Set PINB2 as output
            TCCR1A = (1 << COM1B1) | (1 << WGM11); // Fast PWM, TOP = ICR1
            TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
            ICR1 = 999; // TOP for 1 kHz
            OCR1B = (uint16_t)((pwmVal * 999UL) / 255); // Scale to TOP = 999
            break;
        case PWM_PINB3:
            DDRB |= (1 << DDB3); // Set PINB3 as output
            TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, TOP = 0xFF
            TCCR2B = (1 << CS21) | (1 << CS20); // Prescaler = 32
            OCR2A = pwmVal;
            break;
        default:
            return 1; // Invalid pin
    }

    return 0;
}

#endif // AVR_EASYIO_H

