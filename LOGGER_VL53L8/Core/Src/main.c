/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "53l8a1_ranging_sensor.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  LOG,
} States_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERSION        150126u

#define TOF_SENSOR_ODR      3u /* Output Data Rate [Hz] */
#define RESULT_VALUES_NB    2u /* Logged result values: DISTANCE, SIGNAL */
#define TOF_RESOLUTION      VL53L8CX_RESOLUTION_8X8
#define SIGNAL_SIZE         (TOF_RESOLUTION * RESULT_VALUES_NB)
#define DISTANCE_MAX        2000u  /* [mm */
#define SKIPPED_RESULTS_NB  2u  /* First ToF sensor results to be skipped: transition stage */
#define LED_LOG_OK_PERIOD   75u  /* [ms] */
#define LED_LOG_NOK_PERIOD  25u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

RANGING_SENSOR_Capabilities_t Cap;
RANGING_SENSOR_ProfileConfig_t Profile;

__IO uint32_t ToF_EventFlag = SET;
__IO States_t appState = LOG;
__IO uint32_t ld2PeriodCnt = 0;  /* LD2 toggling counter incremented by SysTick */
__IO uint32_t ld2Period = 0;  /* LD2 toggle timing to be set by user */
uint32_t resultCnt = 0;
float input_user_buffer[SIGNAL_SIZE] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ToF_Init(void);
void ToF_ProfileConfig(uint8_t resolution);
void ToF_Start(void);
int32_t FillBuffer(float *buffer);
void PrintBuffer(float *buffer);
void Log(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
 HAL_UART_Transmit(&huart2,(uint8_t*)ptr,len,100);
  return len;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("\r\nVer: %u.%u.%u\r\n",VERSION,SIGNAL_SIZE,TOF_SENSOR_ODR);

  ToF_Init();
  ToF_ProfileConfig(SIGNAL_SIZE);
  ToF_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	switch (appState)
	{
	  case LOG:
	    Log();
	  	break;
	  default:
	    appState = LOG;
	    break;
	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ToF_Init(void)
{
  int32_t status = 0;

  printf("ToF sensor init...\r\n");

  /* Sensor reset */
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
   HAL_Delay(2);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
   HAL_Delay(2);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
   HAL_Delay(2);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
   HAL_Delay(2);

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);
  if (status != BSP_ERROR_NONE)
  {
	printf("ERROR\r\n");
	Error_Handler();
  }
  else
  {
	printf("OK\r\n");
  }
}

void ToF_ProfileConfig(uint8_t resolution)
{
  uint32_t Id;

  VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
  VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);

  switch (resolution)
  {
    case VL53L8CX_RESOLUTION_8X8:
      Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
      break;
    case VL53L8CX_RESOLUTION_4X4:
      Profile.RangingProfile = RS_PROFILE_4x4_CONTINUOUS;
      break;
    default:
      Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
      break;
  }
  Profile.TimingBudget = 30; /* 5 ms < TimingBudget < 100 ms */
  Profile.Frequency = TOF_SENSOR_ODR; /* Hz */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableAmbient = 0; /* Enable: 1, Disable: 0 */

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
  printf("ToF sensor ID: 0x%X\r\n",(int)Id);
}

void ToF_Start(void)
{
  int32_t status = 0;

  status = VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS);
  if (status != BSP_ERROR_NONE)
  {
    printf("ERROR: TOF sensor start failed\r\n");
    Error_Handler();
  }
}

int32_t FillBuffer(float *buffer)
{
  uint16_t i,j;
  RANGING_SENSOR_Result_t Result;
  int32_t status = 0;

  if (ToF_EventFlag)
  {
    ToF_EventFlag = RESET;
	status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &Result);
    if (status == BSP_ERROR_NONE)
    {
      resultCnt++;
      j=0;
      for (i=0; i<TOF_RESOLUTION; i++)
	  {
    	if (Result.ZoneResult[i].Distance[0] <= DISTANCE_MAX)  /* Distance within expected range */
    	{
	      buffer[j++] = (float)Result.ZoneResult[i].Distance[0];
    	}
    	else
    	{
    	  buffer[j++] = DISTANCE_MAX;
    	}
    	buffer[j++] = (float)Result.ZoneResult[i].Signal[0];
    	//buffer[j++] = (float)Result.ZoneResult[i].Ambient[0];
	  }
	 }
  }
  return status;
}

void PrintBuffer(float *buffer)
{
  uint8_t row, col;

  printf("\r\n--- Pomiar #%lu ---\r\n", resultCnt - SKIPPED_RESULTS_NB);

  printf("Odleglosc [mm]:\r\n");
  for (row = 0; row < 8; row++)
  {
    for (col = 0; col < 8; col++)
    {
      printf("%5d", (int)buffer[(row * 8 + col) * 2]);
    }
    printf("\r\n");
  }

  printf("Sygnal:\r\n");
  for (row = 0; row < 8; row++)
  {
    for (col = 0; col < 8; col++)
    {
      printf("%5d", (int)buffer[(row * 8 + col) * 2 + 1]);
    }
    printf("\r\n");
  }
}

void Log(void)
{
  int32_t status = 0;

  status = FillBuffer(input_user_buffer);
  if (resultCnt>SKIPPED_RESULTS_NB)
  {
    if (status == 0)
    {
	  ld2Period = LED_LOG_OK_PERIOD;
      PrintBuffer(input_user_buffer);
    }
    else
    {
	  ld2Period = LED_LOG_NOK_PERIOD;
    }
  }
  else
  {
	printf("Waiting for valid data...\r\n");
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin==GPIO_PIN_4)
  {
	ToF_EventFlag = SET;
  }
}

void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;
  if(ld2PeriodCnt>0)
  {
    ld2PeriodCnt--;
  }
  else
  {
    ld2PeriodCnt = ld2Period;
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
