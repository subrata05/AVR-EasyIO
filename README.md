# AVR EasyIO

**AVR EasyIO** is a lightweight utility header for simplifying GPIO and ADC operations in bare-metal AVR programming. It abstracts the verbose register manipulations into readable and efficient helper functions, without sacrificing performance or control.

## Features

- `pinMode()` – Configure pins as `INPUT`, `INPUT_PULLUP`, or `OUTPUT`.
- `digitalWrite()` – Set digital pins `HIGH` or `LOW`.
- `digitalRead()` – Read digital pin values.
- `analogRead()` – Sample 10-bit analog values from ADC pins (0–5).

All functions operate directly on AVR registers for maximum speed and transparency.

## Getting Started

### Requirements

- AVR-GCC toolchain
- Supported AVR microcontroller (e.g. ATmega328P/PB)
- Optional: `avrdude` for flashing via ISP

### Example Circuit

- **Button** connected to PD7 (with internal pull-up)
- **LED** connected to PD6
- **Potentiometer** connected to PC0 (ADC0)

### Example Code

See `Examples/main.cpp` for a minimal demonstration.

```cpp
#include "avr_easyio.h"

int main(void)
{
    pinMode(7, &DDRD, &PORTD, INPUT_PULLUP);
    digitalWrite(6, &DDRD, &PORTD, 0); // LED off

    while (1) {
        if (digitalRead(7, &DDRD, &PIND) == 0) {
            digitalWrite(6, &DDRD, &PORTD, 0);
        } else {
            digitalWrite(6, &DDRD, &PORTD, 1);
        }

        uint16_t adcVal = analogRead(0, &DDRC, &PORTC);
        _delay_ms(50);
    }
}
```
