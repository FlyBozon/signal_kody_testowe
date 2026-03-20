/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   Konfiguracja GPIO dla projektu COMBINED_LAB01_VL53L8
 *
 * Piny:
 *  PA5  – LD2 (LED)          output
 *  PA7  – ToF RESET           output (domyślnie LOW, sekwencja w main.c)
 *  PB0  – ToF ENABLE (LPn)   output (domyślnie HIGH = enabled)
 *  PB4  – MEMS INT1           EXTI4 rising (dane gotowe z LSM6DSV16X)
 *  PC13 – B1 (przycisk)       EXTI15_10 falling (zmiana stanu LOG/INFERENCE)
 *
 * UWAGA: PA4 (oryginalny pin INT czujnika ToF) NIE jest podłączony do EXTI,
 *        ponieważ EXTI4 jest zajęty przez PB4 (MEMS). Odczyt ToF odbywa się
 *        w trybie blokującym (RS_MODE_BLOCKING_CONTINUOUS).
 ******************************************************************************
 */

#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Włączenie zegarów portów */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* --- Stany wyjść przed inicjalizacją -------------------------------------- */
  /* PA5 (LED) – zgaszony */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  /* PA7 (ToF RESET) – LOW (reset aktywny), podnoszone w ToF_Init() */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  /* PB0 (ToF ENABLE/LPn) – HIGH (czujnik odblokowany) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /* --- PA5: LED LD2 --------------------------------------------------------- */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* --- PA7: ToF RESET ------------------------------------------------------- */
  GPIO_InitStruct.Pin   = GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* --- PB0: ToF ENABLE (LPn) ------------------------------------------------ */
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* --- PB4: MEMS LSM6DSV16X INT1 – EXTI4 rising ---------------------------- */
  GPIO_InitStruct.Pin  = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* --- PC13: przycisk B1 – EXTI15_10 falling -------------------------------- */
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* --- NVIC ------------------------------------------------------------------ */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
