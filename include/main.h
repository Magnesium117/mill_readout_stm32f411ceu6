#pragma once
#include <stdint.h>
int decodeFrames(uint32_t *message);
uint8_t getDigit(uint8_t data);
int dispalyData(uint8_t *data);
int decodeFrames(uint32_t *message);
void transmitData(uint32_t *message);
