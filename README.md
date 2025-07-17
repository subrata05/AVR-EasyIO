# AVR EasyIO

**AVR EasyIO** is a lightweight utility header for simplifying GPIO, ADC and PWM operations in bare-metal AVR programming. It abstracts the verbose register manipulations into readable and efficient helper functions, without sacrificing performance or control.

## Features

- `pinMode()` – Configure pins as `INPUT`, `INPUT_PULLUP`, or `OUTPUT`.
- `digitalWrite()` – Set digital pins `HIGH` or `LOW`.
- `digitalRead()` – Read digital pin values.
- `analogRead()` – Sample 10-bit analog values from ADC pins (0–5).
- `setPWM()` – Configure PWM on Timer0, Timer1, or Timer2 pins (e.g., PINB1, PINB2, PIND3, PIND5, PIND6, PINB3) with duty cycle control.

All functions operate directly on AVR registers for maximum speed and transparency.

## Getting Started

### Requirements

- AVR-GCC toolchain
- Supported AVR microcontroller (e.g. ATmega328P/PB)
- Optional: `avrdude` for flashing via ISP

### Example Circuit

- **Button** connected to PIND7 (with internal pull-up)
- **LED** connected to PINB2 (PWM-capable, OC1B, Timer1) for brightness control
- **Potentiometer** connected to PINC0 (ADC0)

### Example Code

See `Examples/main.cpp` for a minimal demonstration.

```cpp
#define F_CPU 8000000UL
#include <util/delay.h>
#include "avr_easyio.h"

int main(void)
{
    pinMode(7, &DDRD, &PORTD, INPUT_PULLUP); // Configure button pin
    setPWM(PWM_PINB2, 0); // Initialize PWM on PB2 (LED off)

    while (1) {
        if (digitalRead(7, &DDRD, &PIND) == 0) { // Button pressed
            setPWM(PWM_PINB2, 255); // LED at full brightness
        } else {
            setPWM(PWM_PINB2, 64); // LED at ~25% brightness
        }

        uint16_t adcVal = analogRead(0, &DDRC, &PORTC); // Read potentiometer
        _delay_ms(50);
    }
}
```
