#include "main.h"
// #include "Legacy/stm32_hal_legacy.h"
// #include "disp.h"
// #include "stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_uart.h"
// #include "stm32f4xx_hal_uart_ex.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdint.h>

#define DATA_PIN_A GPIO_PIN_4

#define DEBUG_PIN_B GPIO_PIN_0
#define debugToggle GPIOB->ODR ^= DEBUG_PIN_B

#define ERROR_LIMIT 5

#define HEADER_1 0b100000000010
#define HEADER_2 0b100000000110
#define HEADER_3 0b100001010010
#define ADDRESS_1 0xA0
#define ADDRESS_2 0xA1
#define ADDRESS_3 0xA2
#define ADDRESS_4 0xA3
#define MESSAGE_FILTER 0b00000000111111110
#define ADDRESS_FILTER 0b11111111000000000
#define HEADER_FILTER 0b111111111111
#define MESSAGE_FRAMES 7
#define HEADER_FRAMES 3
#define DATA_FRAMES 4
#define MESSAGE_BITS (HEADER_FRAMES * 12 + DATA_FRAMES * 17)

uint32_t message[MESSAGE_FRAMES] = {0};
uint32_t *message_buffer = message;
uint32_t dumping_ground = 0;
uint32_t frame_counter = 0;
// uint32_t bit_buffer[256] = {0};
// uint32_t bit_counter = 0;
// uint32_t* frame_indicator[MESSAGE_FRAMES]={0};
// frame_indicator[0]=message_buffer;
uint8_t data[3] = {0};
uint8_t hw[5] = "HW\r\n";
uint8_t err[4] = "Err";
uint8_t newline[3] = "\r\n";
USART_HandleTypeDef USART2_H;
EXTI_HandleTypeDef ClkLine;
EXTI_HandleTypeDef CSLine;
uint8_t heartbeatFlag = 0;
int main() {
  //
  // Init RCC
  //
  RCC_OscInitTypeDef RCCOscInitStruct = {0};
  RCCOscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // 25MHz
  RCCOscInitStruct.HSIState = RCC_HSI_OFF;
  RCCOscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCCOscInitStruct.PLL.PLLM = 25;            // 1MHz
  RCCOscInitStruct.PLL.PLLN = 200;           // 200MHz
  RCCOscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 100MHz
  RCCOscInitStruct.PLL.PLLQ = 4;
  RCCOscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  HAL_RCC_OscConfig(&RCCOscInitStruct);

  RCC_ClkInitTypeDef RCCClkInitStruct = {0};
  RCCClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCCClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCCClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCCClkInitStruct.APB1CLKDivider = RCC_SYSCLK_DIV2; // 50MHz
  RCCClkInitStruct.APB2CLKDivider = RCC_SYSCLK_DIV1; // 100MHz
  HAL_RCC_ClockConfig(&RCCClkInitStruct, FLASH_LATENCY_2);
  //
  // Init GPIOS
  //
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIOInitStruct = {0};
  // GPIOInitStruct.Pin = GPIO_PIN_13;
  GPIOInitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIOInitStruct.Pull = GPIO_NOPULL;
  GPIOInitStruct.Speed = GPIO_SPEED_HIGH;
  // HAL_GPIO_Init(GPIOC, &GPIOInitStruct); // Init Push Button
  GPIOInitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIOInitStruct); // Init CLK: PIN A0 and !CS: PIN A1
  GPIOInitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &GPIOInitStruct); // Init CLK: PIN A0 and !CS: PIN A1
  GPIOInitStruct.Pin = DATA_PIN_A;
  GPIOInitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOA, &GPIOInitStruct); // Init Data Pin
  GPIOInitStruct.Pin = GPIO_PIN_13;
  GPIOInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &GPIOInitStruct); // Init LED
  GPIOInitStruct.Pin = DEBUG_PIN_B;
  HAL_GPIO_Init(GPIOB, &GPIOInitStruct); // init debug Pin
  GPIOInitStruct.Pin = GPIO_PIN_2;
  GPIOInitStruct.Mode = GPIO_MODE_AF_PP;
  GPIOInitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIOInitStruct); // init USART2 TX pin
  //
  // Init EXTI
  //
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  ClkLine.Line = EXTI_LINE_0;
  CSLine.Line = EXTI_LINE_1;
  EXTI_ConfigTypeDef EXTILINE_config;
  EXTILINE_config.Line = EXTI_LINE_0;
  EXTILINE_config.Mode = EXTI_MODE_INTERRUPT;
  EXTILINE_config.Trigger = EXTI_TRIGGER_FALLING;
  EXTILINE_config.GPIOSel = EXTI_GPIOA;
  HAL_EXTI_SetConfigLine(&ClkLine, &EXTILINE_config);
  EXTILINE_config.Line = EXTI_LINE_1;
  EXTILINE_config.Trigger = EXTI_TRIGGER_RISING;
  HAL_EXTI_SetConfigLine(&CSLine, &EXTILINE_config);

  // EXTI_HandleTypeDef EXTIInitStruct={0};
  // EXTIInitStruct.Line=EXTI_LINE_13;
  //
  // Init USART
  //
  __HAL_RCC_USART2_CLK_ENABLE();
  USART_InitTypeDef USARTInitStruct;
  USARTInitStruct.BaudRate = 115200;
  USARTInitStruct.WordLength = USART_WORDLENGTH_8B;
  USARTInitStruct.StopBits = USART_STOPBITS_1;
  USARTInitStruct.Parity = USART_PARITY_NONE;
  USARTInitStruct.Mode = USART_MODE_TX;
  USARTInitStruct.CLKPolarity = USART_POLARITY_LOW;
  USARTInitStruct.CLKPhase = USART_PHASE_1EDGE;
  USARTInitStruct.CLKLastBit = USART_LASTBIT_DISABLE;
  USART2_H.Init = USARTInitStruct;
  USART2_H.Instance = USART2;
  HAL_USART_Init(&USART2_H);

  //
  // Enable Interrupt
  //
  uint32_t PriorityGroup;
  PriorityGroup = NVIC_EncodePriority(2, 2, 1);
  NVIC_SetPriority(EXTI15_10_IRQn, PriorityGroup);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  PriorityGroup = NVIC_EncodePriority(2, 0, 0);
  NVIC_SetPriority(EXTI0_IRQn, PriorityGroup); // configure interrupt for Clock
  NVIC_EnableIRQ(EXTI0_IRQn);                  // enable interrupt for Clock
  PriorityGroup = NVIC_EncodePriority(2, 1, 0);
  NVIC_SetPriority(EXTI1_IRQn, PriorityGroup); // configure interrupt for !CS
  NVIC_EnableIRQ(EXTI1_IRQn);                  // enable interrupt for !CS
  //
  // Initialize debug pins
  //
  initDisplay(GPIOA, GPIO_PIN_7, &USART2_H);
  GPIOA->ODR |= GPIO_PIN_5;
  GPIOB->ODR |= DEBUG_PIN_B;
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  char test_send[5] = "AB\r\n";
  HAL_USART_Transmit(&USART2_H, (uint8_t *)test_send, 4, 1);
  GPIOA->ODR |= GPIO_PIN_5;
  GPIOB->ODR |= DEBUG_PIN_B;
  while (1) {
    __WFI();
  }
}

void EXTI15_10_IRQHandler() {
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_13)) {
    __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_13);
    GPIOA->ODR ^= GPIO_PIN_5;
    GPIOB->ODR ^= DEBUG_PIN_B;
    HAL_USART_Transmit(&USART2_H, hw, 4, 1);
    // uint8_t data_buffer[11] = {0xFF, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    // sendMessage(data_buffer, 11);
    uint8_t data_buffer[6] = {1, 0, 2, 0, 17, 16};
    displayDataInt(data_buffer, 6);
  }
}
void EXTI0_IRQHandler() {
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_0);
  // debugToggle;
  // GPIOA->ODR ^= DEBUG_PIN_A;
  *message_buffer = *message_buffer << 1;
  *message_buffer |= (GPIOA->IDR & 0b10000) >> 4;
  // *message_buffer |= 1;
  // *message_buffer = 0xFF;
  // bit_buffer[bit_counter++] = GPIOA->IDR;
  // debugToggle;
  // GPIOB->ODR ^= DEBUG_PIN_B;
  // GPIOA->ODR ^= GPIO_PIN_5;
}
void EXTI1_IRQHandler() {
  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);
  // debugToggle;
  frame_counter++;
  // uint8_t buff = (uint8_t)frame_counter + 0x30;
  // HAL_USART_Transmit(&USART2_H, &buff, 1, 1);
  if (frame_counter >= MESSAGE_FRAMES) {
    dumping_ground = 0;
    message_buffer =
        &dumping_ground; // ignore all future bits; keep the message clean
    // decode the message
    // transmitData(message);
    // GPIOB->ODR ^= DEBUG_PIN_B;
    uint8_t buff = decodeFrames(message);
    if (buff != 0) {
      HAL_USART_Transmit(&USART2_H, err, 3, 1);
      buff = buff + 0x30;
      HAL_USART_Transmit(&USART2_H, &buff, 1, 1);
      // uint32_t dumping_ground_cmp = 0;
      // while (dumping_ground != dumping_ground_cmp) {
      // dumping_ground_cmp = dumping_ground;
      // int delay = 10e3;
      // for (int i = 0; i < delay; i++) {
      // __NOP();
      // }
      // }
      for (int i = 0; i < MESSAGE_FRAMES - 1; i++) {
        message[i] = message[i + 1];
      }
      message[6] = dumping_ground;
      frame_counter = 6;
      message_buffer = &message[frame_counter];
      return;
    }
    if (!dispalyData(data)) {
      HAL_USART_Transmit(&USART2_H, hw, 4, 1);
    }
    // bit_counter = 0;
    frame_counter = 0;
    // message_buffer = message;
    // return;
  }
  message_buffer = &message[frame_counter];
  // debugToggle;
}
int decodeFrames(uint32_t *message) {
  if ((message[0] & HEADER_FILTER) != HEADER_1) {
    return 1;
  }
  if ((message[1] & HEADER_FILTER) != HEADER_2) {
    return 2;
  }
  if ((message[2] & HEADER_FILTER) != HEADER_3) {
    return 3;
  }
  if ((message[3] & ADDRESS_FILTER) >> 9 != ADDRESS_1) {
    return 4;
  }
  if ((message[4] & ADDRESS_FILTER) >> 9 != ADDRESS_2) {
    return 5;
  }
  if ((message[5] & ADDRESS_FILTER) >> 9 != ADDRESS_3) {
    return 6;
  }
  if ((message[6] & ADDRESS_FILTER) >> 9 != ADDRESS_4) {
    return 7;
  }
  data[0] = getDigit((uint8_t)((message[3] & MESSAGE_FILTER) >> 1));
  data[1] = getDigit((uint8_t)((message[4] & MESSAGE_FILTER) >> 1));
  data[2] = getDigit((uint8_t)((message[5] & MESSAGE_FILTER) >> 1));
  for (int i = 0; i < 3; i++) {
    if (data[i] == 0xFF) {
      return i + 17;
    }
  }
  return 0;
}

uint8_t getDigit(uint8_t data) {
  switch (data) {
  case 0b01111101:
    return 0;
  case 0b00000101:
    return 1;
  case 0b01101011:
    return 2;
  case 0b01001111:
    return 3;
  case 0b00010111:
    return 4;
  case 0b01011110:
    return 5;
  case 0b01111110:
    return 6;
  case 0b00001101:
    return 7;
  case 0b01111111:
    return 8;
  case 0b01011111:
    return 9;
  default:
    return 0xFF;
  }
}
int dispalyData(uint8_t *data) {
  uint8_t buff[3];
  buff[0] = data[0] + 0x30;
  buff[1] = data[1] + 0x30;
  buff[2] = data[2] + 0x30;
  HAL_USART_Transmit(&USART2_H, buff, 3, 1);
  uint8_t disp_data[6] = {data[0], data[1], data[2], 0, 0xFF, 0xFF};
  if (heartbeatFlag) {
    disp_data[4] = 17;
  }
  if (disp_data[0] == 0) {
    disp_data[0] = 0xFF;
    if (disp_data[1] == 0) {
      disp_data[1] = 0xFF;
      if (disp_data[2] == 0) {
        disp_data[2] = 0xFF;
      }
    }
  }
  heartbeatFlag ^= 1;
  displayDataInt(disp_data, 6);
  return 0;
}
void transmitData(uint32_t *message) {
  uint8_t buff;
  uint32_t cbuff[3] = {HEADER_1, HEADER_2, HEADER_3};
  HAL_USART_Transmit(&USART2_H, newline, 2, 1);
  for (int i = 0; i < HEADER_FRAMES; i++) {
    for (int j = 11; j >= 0; j--) {
      buff = (message[i] >> j) & 1;
      buff += 0x30;
      HAL_USART_Transmit(&USART2_H, &buff, 1, 1);
    }
    HAL_USART_Transmit(&USART2_H, newline, 2, 1);
    for (int j = 11; j >= 0; j--) {
      buff = (cbuff[i] >> j) & 1;
      buff += 0x30;
      HAL_USART_Transmit(&USART2_H, &buff, 1, 1);
    }
    HAL_USART_Transmit(&USART2_H, newline, 2, 1);
  }
  for (int i = 0; i < DATA_FRAMES; i++) {
    for (int j = 16; j >= 0; j--) {
      buff = (message[i] >> j) & 1;
      buff += 0x30;
      HAL_USART_Transmit(&USART2_H, &buff, 1, 1);
    }
    HAL_USART_Transmit(&USART2_H, newline, 2, 1);
  }
}
