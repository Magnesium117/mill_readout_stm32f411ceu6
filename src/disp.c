#include "disp.h"
#include "Legacy/stm32_hal_legacy.h"
#include "cmsis_gcc.h"
#include "main.h"
// #include "stm32f334x8.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_exti.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_gpio_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_uart.h"
// #include "stm32f4xx_hal_uart_ex.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include <stdint.h>

#define SEND_DATA_ADDR_ADDING 0b01000000
#define SEGMRNT_ADDR_0 0b11000000
#define SEGMRNT_ADDR_5 0b11000000
#define DISPLAY_ON_FULL 0b10001111
static const uint8_t DISPLAY_ADDR[6] = {0b11000000, 0b11000001, 0b11000010,
                                        0b11000011, 0b11000100, 0b11000101};
static const uint8_t segments[18] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110111, // A
    0b01111100, // b
    0b00111001, // C
    0b01011110, // d
    0b01111001, // E
    0b01110001, // F
    0b01000000, // -
    0b10000000  // .
};
// TIM_HandleTypeDef TIM3_Handle;
TIM_HandleTypeDef htim3 = {0};
USART_HandleTypeDef *husart2 = {0};
uint8_t errormsg[4] = "err";
uint8_t initmsg[5] = "init";
uint16_t output_buffer[512] = {0};
uint32_t bufferidx = 0;
uint32_t buffermax = 0;
int DisplayTransmissionRunning = 0;
uint16_t DIOPIN_A = 0;
uint16_t CLKPIN_A = 0;
int DIOpinNumber = 0;

void initDisplay(GPIO_TypeDef *Port, uint16_t DIO,
                 USART_HandleTypeDef *HUSART2) {
  HAL_USART_Transmit(HUSART2, initmsg, 4, 1);
  GPIO_TypeDef *Portclk = GPIOA;
  uint16_t CLK = GPIO_PIN_6;
  CLKPIN_A = CLK;
  DIOPIN_A = DIO;
  husart2 = HUSART2;
  for (int i = 0; i < 16; i++) {
    if ((DIOPIN_A & (1 << i)) != 0) {
      DIOpinNumber = i;
    }
  }

  //
  // Init GPIOs
  //
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pin = DIO;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pin = CLK;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(Portclk, &GPIO_InitStruct);
  //
  // Init Timer 3
  //
  int clock = 1e3;
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Channel = TIM_CHANNEL_1;
  htim3.Init.Prescaler = 100 - 1; // 100 MHz / 100 = 1 MHz
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1e6 / clock - 1; // 100 kHz PWM
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  // oc.OCPolarity = TIM_OCPOLARITY_LOW;
  oc.OCIdleState = TIM_OCIDLESTATE_SET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  oc.Pulse = 1e6 / clock / 2 - 1; // 50%
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    HAL_USART_Transmit(HUSART2, &errormsg, 3, 1);
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK) {
    HAL_USART_Transmit(HUSART2, &errormsg, 3, 1);
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  uint8_t priority = NVIC_EncodePriority(2, 0, 1);
  NVIC_SetPriority(TIM3_IRQn, priority);
  NVIC_EnableIRQ(TIM3_IRQn);
  // __HAL_TIM_DISABLE(&htim3);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  GPIOA->MODER &= ~(0b10 << (2 * 6));
  GPIOA->MODER |= (0b01 << (2 * 6));
  GPIOA->ODR |= CLKPIN_A;
  GPIOA->ODR |= DIOPIN_A;
  // __enable_irq();
  // uint8_t data_buffer[11] = {0xFF, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  // sendMessage(data_buffer, 11);
  DisplayTransmissionRunning = 0;
  uint8_t display_init = DISPLAY_ON_FULL;
  sendMessage(&display_init, 1);
  // for (int i = 0; i < 10; i++) {
  // __NOP();
  // }
  uint8_t display_clear[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  displayDataInt(display_clear, 6);
}

void TIM3_IRQHandler() {
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1)) {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);
    if (bufferidx < buffermax) {
      GPIOA->ODR = (GPIOA->ODR & ~(DIOPIN_A)) | output_buffer[bufferidx];
      // return;
    }
    bufferidx++;
    // if (bufferidx == buffermax) {
    //   GPIOA->MODER &= ~(0b10 << (2 * 6));
    //   GPIOA->MODER |= (0b01 << (2 * 6));
    //   GPIOA->ODR |= CLKPIN_A;
    // }
    if (bufferidx > buffermax + 1) {
      // __HAL_TIM_DISABLE(&htim3);
      // HAL_GPIO_WritePin(GPIOA, CLKPIN_A, GPIO_PIN_SET);
      // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      // GPIOA->MODER &= ~(0b10 << (2 * 6));
      // GPIOA->MODER |= (0b01 << (2 * 6));
      // GPIOA->ODR |= CLKPIN_A;
      // GPIOA->ODR &= ~DIOPIN_A;
      // GPIOA->ODR |= DIOPIN_A;
      // DisplayTransmissionRunning = 0;
      HAL_GPIO_WritePin(GPIOA, DIOPIN_A, GPIO_PIN_SET);
      // bufferidx = 0;
    }
  }
  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE)) {
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    if (bufferidx == buffermax) {
      GPIOA->ODR &= ~DIOPIN_A;
    }
    if (bufferidx > buffermax) {
      GPIOA->MODER &= ~(0b10 << (2 * 6));
      GPIOA->MODER |= (0b01 << (2 * 6));
      GPIOA->ODR |= CLKPIN_A;
      GPIOA->ODR &= ~DIOPIN_A;
    }
    if (bufferidx > buffermax + 1) {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      GPIOA->ODR |= DIOPIN_A;
      DisplayTransmissionRunning = 0;
    }
  }
  // GPIOB->ODR ^= GPIO_PIN_0;
}
int displayDataInt(
    uint8_t *data,
    int size) { // TODO: Verify with scope, the function works correctly
  //
  // Input data stream in send buffer
  //

  uint8_t send_buffer_cmd1 = SEND_DATA_ADDR_ADDING;
  if (sendMessage(&send_buffer_cmd1, 1) != 0) {
    return 1;
  }
  uint8_t send_buffer[16] = {0};
  send_buffer[0] = DISPLAY_ADDR[2];
  for (int i = 0; i < size; i++) {
    if (data[size - i - 1] >= 18) {
      send_buffer[i + 1] = 0b00000000;
    } else {
      send_buffer[i + 1] = segments[data[size - i - 1]];
    }
  }
  if (sendMessage(send_buffer, size + 1) != 0) {
    return 1;
  }
  return 0;
}
int fillOutputBuffer(uint8_t *data_buffer, int size, uint16_t *odr_buffer) {
  uint8_t byte_buffer = 0;
  for (int i = 0; i < size; i++) {
    byte_buffer = data_buffer[i];
    for (int j = 0; j < 8; j++) {
      odr_buffer[i * 9 + j] = (byte_buffer & 0x01) << DIOpinNumber;
      byte_buffer = byte_buffer >> 1;
    }
    odr_buffer[i * 9 + 8] = DIOPIN_A;
    // odr_buffer[i * 9 + 8] = 0x0000;
  }
  return 0;
}
int sendMessage(uint8_t *data_buffer, int size) {
  // uint8_t usartbuffer = 0;
  while (DisplayTransmissionRunning == 1) {
    // transmissionRunning = 0;
    // usartbuffer = DisplayTransmissionRunning + 0x30;
    // HAL_USART_Transmit(husart2, &usartbuffer, 1, 1);
    __NOP();
  }
  // HAL_USART_Transmit(husart2, initmsg, 4, 1);
  DisplayTransmissionRunning = 1;
  HAL_GPIO_WritePin(GPIOA, CLKPIN_A, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DIOPIN_A, GPIO_PIN_RESET);
  if (fillOutputBuffer(data_buffer, size, output_buffer) != 0) {
    return 1;
  }
  TIM3->CNT = 0;
  bufferidx = 0;
  buffermax = size * 9;
  // HAL_GPIO_WritePin(GPIOA, DIOPIN_A, GPIO_PIN_RESET);
  GPIOA->MODER &= ~(0b01 << (2 * 6));
  GPIOA->MODER |= (0b10 << (2 * 6));

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  return 0;
}
