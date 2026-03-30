// sd_cs.cpp
#include <avr/io.h>
#include <stdint.h>

#include "ArduinoCompat.h"
#include "SdFat.h"
// Choose your SD CS pin here: PB2 (Arduino D10 / SS pin)
static constexpr uint8_t CS_BIT = PB2;

void sdCsInit(SdCsPin_t /*pin*/) {
    // ignore 'pin' (Arduino pin number); we hardwire PB2
    DDRB  |= (1 << DDB2);        // PB2 output
    PORTB |= (1 << CS_BIT);      // CS high (inactive)
}

void sdCsWrite(SdCsPin_t /*pin*/, bool level) {
    if (level)  PORTB |=  (1 << CS_BIT);   // high = deselect
    else        PORTB &= ~(1 << CS_BIT);   // low  = select
}
