#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

byte nibble(char c);
uint8_t hexCharacterStringToBytes(uint8_t *byteArray, const uint8_t *hexString, size_t length);

#endif /* FUNCTIONS_H */