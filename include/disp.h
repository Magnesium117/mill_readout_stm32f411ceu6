#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_usart.h"
#include <stdint.h>
void initDisplay(GPIO_TypeDef *Port, uint16_t DIO,
                 USART_HandleTypeDef *HUSART2);
int displayDataInt(uint8_t *data, int size);
int fillOutputBuffer(uint8_t *data_buffer, int size, uint16_t *odr_buffer);
int sendMessage(uint8_t *data_buffer, int size);
int transmitMessage(uint8_t *data_buffer, int size);
void checkDisplayMessageBuffer();
int displayOn();
