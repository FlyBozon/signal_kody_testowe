/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

#include <stdio.h>
#include <stdbool.h>
#include "iks4a1_motion_sensors.h"
#include "iks4a1_motion_sensors_ex.h"
#include "NanoEdgeAI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  LOG,
  LEARN,
  INFERENCE,
} States_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SIGNAL_SIZE             (uint32_t)(NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER)  /* Runtime signal size */
#define LEARNING_ITERATIONS     (uint32_t)(20)
#define LED_LOG_PERIOD          (uint32_t)(75)  /* LED toggling half period [ms] following the app state */
#define LED_LEARN_PERIOD        (uint32_t)(1000)
#define LED_INFERENCE_PERIOD    (uint32_t)(500)
#define LED_ANOMALY_PERIOD      (uint32_t)(20)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile States_t appState = LOG;  /* FSM state */
volatile uint32_t sensorDataReady = RESET;  /* Sensor data ready flag */
float input_user_buffer[SIGNAL_SIZE];  /* Input data buffer */
bool use_pretrained = false; /* true to use the pretrained model, false to start learning from scratch */
volatile uint32_t ledPeriodCnt = 0;  /* LED toggling counter incremented by SysTick */
volatile uint32_t ledPeriod = 0;  /* LED toggle timing to be set by user */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void MEMS_Init(void);
void FillBuffer (float *buffer, uint32_t size);
void PrintBuffer(float *buffer, uint32_t size);
void Log(void);
void Learn(void);
void Inference(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,HAL_MAX_DELAY);
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

  MEMS_Init();
  enum neai_state error_code = neai_anomalydetection_init(use_pretrained);
  if (error_code!=NEAI_OK)
  {
    printf("Knowledge initialization ERROR: %d\r\n",error_code);
    Error_Handler();
  }
  else
  {
    printf("Knowledge initialization done\r\n");
  }


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
	    case LEARN:
	      Learn();
	      appState = INFERENCE;
	      break;
	    case INFERENCE:
	      Inference();
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

void MEMS_Init(void)
{
  IKS4A1_MOTION_SENSOR_AxesRaw_t axes;

  IKS4A1_MOTION_SENSOR_Init(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO|MOTION_GYRO);
  IKS4A1_MOTION_SENSOR_DRDY_Enable_Interrupt(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO|MOTION_GYRO,IKS4A1_MOTION_SENSOR_INT1_PIN);
  IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO|MOTION_GYRO,30.0f);  /* 30 Hz */
  IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO,4);  /* [-4g; +4g] */
  IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0,MOTION_GYRO,250);  /* [-250dps; +250dps] */
  IKS4A1_MOTION_SENSOR_GetAxesRaw(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO,&axes);
  IKS4A1_MOTION_SENSOR_GetAxesRaw(IKS4A1_LSM6DSV16X_0,MOTION_GYRO,&axes);
}

void FillBuffer (float *buffer, uint32_t size)
{
  uint32_t i=0;
  IKS4A1_MOTION_SENSOR_Axes_t axes;

  while (i < size)
  {
    if (sensorDataReady!=RESET)
	{
	  sensorDataReady = RESET;
	  IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0,MOTION_ACCELERO,&axes);
	  buffer[i] = (float) axes.x;
	  buffer[i+1] = (float) axes.y;
	  buffer[i+2] = (float) axes.z;
	  IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0,MOTION_GYRO,&axes);
	  buffer[i+3] = (float) axes.x / 1000;  // [mdps] -> [dps]
	  buffer[i+4] = (float) axes.y / 1000;
	  buffer[i+5] = (float) axes.z / 1000;
	  i += NEAI_INPUT_AXIS_NUMBER;
	}
  }
}

void PrintBuffer(float *buffer, uint32_t size)
{
  static const char *labels[] = { "Ax", "Ay", "Az", "Gx", "Gy", "Gz" };
  uint32_t i = 0;
  uint32_t sample = 0;

  while (i < size)
  {
    printf("-- Sample %lu --\r\n", (unsigned long)sample);
    for (uint32_t j = 0; j < NEAI_INPUT_AXIS_NUMBER; j++)
    {
      printf("  %s = %d\r\n", labels[j], (int)buffer[i + j]);
    }
    i += NEAI_INPUT_AXIS_NUMBER;
    sample++;
  }
  printf("\r\n");
}

void Log(void)
{
  ledPeriod = LED_LOG_PERIOD;
  FillBuffer(input_user_buffer,SIGNAL_SIZE);
  PrintBuffer(input_user_buffer,SIGNAL_SIZE);
}

void Learn(void)
{
  enum neai_state learnStatus;
  uint32_t learnIteration = 0;

  ledPeriod = LED_LEARN_PERIOD;
  printf("\r\nStart of learning\r\n");
  HAL_Delay(2000);  /* Avoid parasitic vibrations of the mechanical setup*/
  while (learnIteration < LEARNING_ITERATIONS)
  {
    printf("Learning... %d/%d",(int)learnIteration+1,(int)LEARNING_ITERATIONS);
    FillBuffer(input_user_buffer,SIGNAL_SIZE);
    learnStatus = neai_anomalydetection_learn(input_user_buffer);
    if ((learnStatus==NEAI_OK) || (learnStatus==NEAI_LEARNING_DONE)) { printf(" -> OK\r\n"); }
    else if (learnStatus==NEAI_LEARNING_IN_PROGRESS) { printf(" -> .\r\n"); }
    else { printf(" -> ERR: %d\r\n",(int)learnStatus); }
    learnIteration++;
  }
}

void Inference(void)
{
  enum neai_state detectionStatus;
  uint8_t similarity = 0;

  FillBuffer(input_user_buffer,SIGNAL_SIZE);
  detectionStatus = neai_anomalydetection_detect(input_user_buffer,&similarity);
  if (detectionStatus==NEAI_OK)
  {
    printf("Similarity = %d%%",(int)similarity);
    if (similarity<90)
    {
      ledPeriod = LED_ANOMALY_PERIOD;
      printf(" -> ANOMALY\r\n");
    }
    else
    {
      ledPeriod = LED_INFERENCE_PERIOD;
      printf(" -> OK\r\n");
    }
  }
  else
  {
	printf("Detection error\r\n");
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin)
  {
    case GPIO_PIN_4:
	  sensorDataReady = SET;
	  break;
	case GPIO_PIN_13:
	  if (appState==LOG) { appState = LEARN; }
	  else if (appState==INFERENCE) { appState = LOG; }
	  break;
	default:
	  break;
  }

}

void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;
  if(ledPeriodCnt>0)
  {
    ledPeriodCnt--;
  }
  else
  {
    ledPeriodCnt = ledPeriod;
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
