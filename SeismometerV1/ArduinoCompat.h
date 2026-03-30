// ArduinoCompat.h
#pragma once

#ifndef SS
// Arduino UNO's SS is digital pin 10 (PB2). SdFat uses this only as a default value.
// If your external driver ignores csPin, any value is fine.
#define SS 10
#endif
//extern "C" void sdCsInit(uint8_t pin);
//extern "C" void sdCsWrite(uint8_t pin, bool level);
