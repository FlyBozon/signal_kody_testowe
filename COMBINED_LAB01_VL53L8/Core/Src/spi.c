#include "spi.h"

SPI_HandleTypeDef hspi2;

void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  
  /* Initial low speed for SD card init (~400kHz bounds) */
  /* PCLK1 is 80MHz. 80MHz / 128 = ~625kHz which is fine for SD init (100-400kHz pref, 625kHz usually works, or 256 for ~312kHz) */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI2)
  {
    /* Zegar dla SPI2 i portów B */
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* PB12 -> CS (jako GPIO w main lub tutaj) */
    /* Zrobimy to w main albo tutaj */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* Zabezpieczenie na start */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

    /* SPI2 GPIO: PB13 (SCK), PB14 (MISO), PB15 (MOSI) */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  if(spiHandle->Instance==SPI2)
  {
    __HAL_RCC_SPI2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
  }
}

/* =========================================================================
 * SD IO BSP wrapper that the Adafruit SD driver relies on. 
 * ========================================================================= */

void SD_IO_Init(void)
{
  /* MX_SPI2_Init is called from main. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

void SD_IO_CSState(uint8_t state)
{
  if (state == 1)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  }
}

void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)DataIn, (uint8_t*)DataOut, DataLength, HAL_MAX_DELAY);
}

uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t rxData = 0xFF;
  HAL_SPI_TransmitReceive(&hspi2, &Data, &rxData, 1, HAL_MAX_DELAY);
  return rxData;
}
