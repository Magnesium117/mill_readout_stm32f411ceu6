#pragma once
#include "Legacy/stm32_hal_legacy.h"
// #include "stm32f334x8.h"
#include "stm32f4xx.h"
// #include "stm32f3xx_hal.h"
// #include "stm32f3xx_hal_cortex.h"
// #include "stm32f3xx_hal_def.h"
// #include "stm32f3xx_hal_exti.h"
// #include "stm32f3xx_hal_gpio.h"
// #include "stm32f3xx_hal_gpio_ex.h"
// #include "stm32f3xx_hal_rcc.h"
// #include "stm32f3xx_hal_tim.h"
// #include "stm32f3xx_hal_uart.h"
// #include "stm32f3xx_hal_uart_ex.h"
// #include "stm32f3xx_hal_usart.h"
// #include "stm32f3xx_ll_gpio.h"
#include <stdint.h>
// #include <wchar.h>
void initDisplay(GPIO_TypeDef *Port, uint16_t DIO,
                 USART_HandleTypeDef *HUSART2);
int displayDataInt(uint8_t *data, int size);
int fillOutputBuffer(uint8_t *data_buffer, int size, uint16_t *odr_buffer);
int sendMessage(uint8_t *data_buffer, int size);
