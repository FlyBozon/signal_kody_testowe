/**
 ******************************************************************************
 * @file    main.c
 * @brief   Połączony projekt: IKS4A1 (LSM6DSV16X) + VL53L8CX
 *
 * Format UART (dla wizualizatora):
 *   MEMS Ax=123 Ay=-45 Az=980 Gx=0 Gy=1 Gz=-2
 *   TDIST 500 520 490 ... (64 wartości, mm)
 *   TSIG  3000 3100 ...   (64 wartości, kcps)
 *   NEAI sim=95 ok=1      (tylko w trybie INFERENCE)
 *
 * Przycisk B1 (PC13): LOG -> LEARN -> INFERENCE -> LOG
 ******************************************************************************
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"

#include <stdbool.h>
#include <stdio.h>

#include "53l8a1_ranging_sensor.h"
#include "NanoEdgeAI.h"
#include "iks4a1_motion_sensors.h"
#include "iks4a1_motion_sensors_ex.h"

/* ===========================================================================
 * Stany automatu
 * ===========================================================================*/
typedef enum { LOG, LEARN, INFERENCE } States_t;

/* ===========================================================================
 * Stałe
 * ===========================================================================*/
#define MEMS_SIGNAL_SIZE                                                       \
  (uint32_t)(NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER)
#define LEARNING_ITERATIONS (uint32_t)(20)

#define LED_LOG_PERIOD (uint32_t)(75)
#define LED_LEARN_PERIOD (uint32_t)(1000)
#define LED_INFERENCE_PERIOD (uint32_t)(500)
#define LED_ANOMALY_PERIOD (uint32_t)(20)

#define TOF_ODR 3u
#define TOF_RESOLUTION 64u  /* 8×8 stref */
#define DISTANCE_MAX 2000u  /* mm */
#define TOF_SKIP_RESULTS 2u /* pierwsze klatki do pominięcia */
#define MEMS_PER_TOF 10u    /* co ile próbek MEMS odczytać ToF */

/* ===========================================================================
 * Zmienne globalne
 * ===========================================================================*/
volatile States_t appState = LOG;
volatile uint32_t memsDataReady = RESET;
volatile uint32_t ledPeriodCnt = 0;
volatile uint32_t ledPeriod = 0;

float mems_buffer[MEMS_SIGNAL_SIZE];

bool use_pretrained = false;

RANGING_SENSOR_Capabilities_t Cap;
RANGING_SENSOR_ProfileConfig_t Profile;
uint32_t tofResultCnt = 0;

/* ===========================================================================
 * Prototypy
 * ===========================================================================*/
void SystemClock_Config(void);
void MEMS_Init(void);
void ToF_Init(void);
void ToF_ProfileConfig(void);
void ToF_Start(void);
void FillMEMSBuffer(float *buf, uint32_t size);
void PrintToFLine(void);
void Log(void);
void Learn(void);
void Inference(void);

/* ===========================================================================
 * printf -> UART2
 * ===========================================================================*/
int _write(int fd, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* ===========================================================================
 * main()
 * ===========================================================================*/
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Wyłączenie buforowania stdout, aby minicom od razu wyrzucał dane */
  setbuf(stdout, NULL);

  /* Czekamy 3 sekundy na pełną enumerację USB (ST-Linka) aby port dał nam 500mA
   */
  printf("Waiting 1s for USB enumeration...\r\n");
  HAL_Delay(1000);

  ToF_Init();
  printf("ToF_Init done\r\n");
  ToF_ProfileConfig();
  printf("ToF_ProfileConfig done\r\n");
  ToF_Start();
  printf("ToF_Start done\r\n");

  MEMS_Init();
  printf("MEMS_Init done\r\n");

  enum neai_state neai_err = neai_anomalydetection_init(use_pretrained);
  if (neai_err != NEAI_OK) {
    printf("NEAI Init Error: %d\r\n", (int)neai_err);
    Error_Handler();
  }
  printf("NEAI Init done\r\n");

  while (1) {
    switch (appState) {
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
}

/* ===========================================================================
 * Zegar (80 MHz, HSI + PLL)
 * ===========================================================================*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

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
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

/* ===========================================================================
 * MEMS: LSM6DSV16X @ 30 Hz, ±4g, ±250 dps, INT1 = PB4
 * ===========================================================================*/
void MEMS_Init(void) {
  IKS4A1_MOTION_SENSOR_AxesRaw_t axes;

  IKS4A1_MOTION_SENSOR_Init(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO | MOTION_GYRO);
  IKS4A1_MOTION_SENSOR_DRDY_Enable_Interrupt(IKS4A1_LSM6DSV16X_0,
                                             MOTION_ACCELERO | MOTION_GYRO,
                                             IKS4A1_MOTION_SENSOR_INT1_PIN);
  IKS4A1_MOTION_SENSOR_SetOutputDataRate(IKS4A1_LSM6DSV16X_0,
                                         MOTION_ACCELERO | MOTION_GYRO, 30.0f);
  IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, 4);
  IKS4A1_MOTION_SENSOR_SetFullScale(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, 250);
  IKS4A1_MOTION_SENSOR_GetAxesRaw(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &axes);
  IKS4A1_MOTION_SENSOR_GetAxesRaw(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &axes);
}

/* ===========================================================================
 * ToF: VL53L8CX – bez przerwania PA4 (konflikt EXTI4/PB4), polling
 * ===========================================================================*/
void ToF_Init(void) {
  int32_t status;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(2);

  status = VL53L8A1_RANGING_SENSOR_Init(VL53L8A1_DEV_CENTER);
  if (status != BSP_ERROR_NONE) {
    printf("ToF Init Error: %d\r\n", (int)status);
    Error_Handler();
  }
}

void ToF_ProfileConfig(void) {
  uint32_t Id;
  VL53L8A1_RANGING_SENSOR_ReadID(VL53L8A1_DEV_CENTER, &Id);
  VL53L8A1_RANGING_SENSOR_GetCapabilities(VL53L8A1_DEV_CENTER, &Cap);

  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = 30;
  Profile.Frequency = TOF_ODR;
  Profile.EnableSignal = 1;
  Profile.EnableAmbient = 0;

  VL53L8A1_RANGING_SENSOR_ConfigProfile(VL53L8A1_DEV_CENTER, &Profile);
}

void ToF_Start(void) {
  int32_t status = VL53L8A1_RANGING_SENSOR_Start(VL53L8A1_DEV_CENTER,
                                                 RS_MODE_BLOCKING_CONTINUOUS);
  if (status != BSP_ERROR_NONE) {
    printf("ToF Start Error: %d\r\n", (int)status);
    Error_Handler();
  }
}

/* ===========================================================================
 * FillMEMSBuffer – potrzebne przez Learn()
 * ===========================================================================*/
void FillMEMSBuffer(float *buf, uint32_t size) {
  uint32_t i = 0;
  IKS4A1_MOTION_SENSOR_Axes_t axes;

  while (i < size) {
    if (memsDataReady != RESET) {
      memsDataReady = RESET;
      IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &axes);
      buf[i] = (float)axes.x;
      buf[i + 1] = (float)axes.y;
      buf[i + 2] = (float)axes.z;
      IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &axes);
      buf[i + 3] = (float)axes.x / 1000.0f;
      buf[i + 4] = (float)axes.y / 1000.0f;
      buf[i + 5] = (float)axes.z / 1000.0f;
      i += NEAI_INPUT_AXIS_NUMBER;
    }
  }
}

/* ===========================================================================
 * PrintToFLine – pobiera jedną klatkę 8×8 i wysyła dwie linie:
 *   TDIST <64 wartości odległości w mm>
 *   TSIG  <64 wartości sygnału w kcps>
 * ===========================================================================*/
void PrintToFLine(void) {
  RANGING_SENSOR_Result_t Result;
  int32_t status;
  uint16_t i;

  status = VL53L8A1_RANGING_SENSOR_GetDistance(VL53L8A1_DEV_CENTER, &Result);
  if (status != BSP_ERROR_NONE)
    return;

  tofResultCnt++;
  if (tofResultCnt <= TOF_SKIP_RESULTS)
    return; /* poczekaj na stabilne dane */

  printf("TDIST");
  for (i = 0; i < TOF_RESOLUTION; i++) {
    int d = (int)Result.ZoneResult[i].Distance[0];
    if (d < 0 || d > (int)DISTANCE_MAX)
      d = (int)DISTANCE_MAX;
    printf(" %d", d);
  }
  printf("\r\n");

  printf("TSIG");
  for (i = 0; i < TOF_RESOLUTION; i++)
    printf(" %d", (int)Result.ZoneResult[i].Signal[0]);
  printf("\r\n");
}

/* ===========================================================================
 * LOG – ciągłe dane z MEMS (30 Hz) i ToF (co 10 próbek, ~3 Hz)
 * ===========================================================================*/
void Log(void) {
  static uint32_t tof_cnt = 0;
  IKS4A1_MOTION_SENSOR_Axes_t acc, gyr;

  ledPeriod = LED_LOG_PERIOD;

  if (memsDataReady != RESET) {
    memsDataReady = RESET;

    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &acc);
    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &gyr);

    /* Akcelerometr [mg], Żyroskop [dps] */
    printf("MEMS Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\r\n", (int)acc.x,
           (int)acc.y, (int)acc.z, (int)(gyr.x / 1000), (int)(gyr.y / 1000),
           (int)(gyr.z / 1000));

    tof_cnt++;
    if (tof_cnt >= MEMS_PER_TOF) {
      tof_cnt = 0;
      PrintToFLine();
    }
  }
}

/* ===========================================================================
 * LEARN – uczenie modelu NEAI (blokujące)
 * ===========================================================================*/
void Learn(void) {
  enum neai_state st;
  uint32_t iter = 0;

  ledPeriod = LED_LEARN_PERIOD;
  printf("LEARN start %lu iter\r\n", (unsigned long)LEARNING_ITERATIONS);
  HAL_Delay(2000);

  while (iter < LEARNING_ITERATIONS) {
    FillMEMSBuffer(mems_buffer, MEMS_SIGNAL_SIZE);
    st = neai_anomalydetection_learn(mems_buffer);
    printf("LEARN %lu/%lu st=%d\r\n", (unsigned long)(iter + 1),
           (unsigned long)LEARNING_ITERATIONS, (int)st);
    iter++;
  }
  printf("LEARN done\r\n");
}

/* ===========================================================================
 * INFERENCE – ciągłe dane + wykrywanie anomalii NEAI (co pełny bufor)
 *
 * Co próbkę MEMS:
 *   - drukuje MEMS ...
 *   - akumuluje w buforze NEAI
 *   - gdy bufor pełny: uruchamia detekcję, drukuje NEAI sim=X ok=Y
 * Co MEMS_PER_TOF próbek: drukuje TDIST/TSIG
 * ===========================================================================*/
void Inference(void) {
  static uint32_t buf_cnt = 0;
  static uint32_t tof_cnt = 0;
  IKS4A1_MOTION_SENSOR_Axes_t acc, gyr;

  if (memsDataReady != RESET) {
    memsDataReady = RESET;

    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &acc);
    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &gyr);

    /* Drukuj bieżącą próbkę */
    printf("MEMS Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\r\n", (int)acc.x,
           (int)acc.y, (int)acc.z, (int)(gyr.x / 1000), (int)(gyr.y / 1000),
           (int)(gyr.z / 1000));

    /* Akumuluj w buforze NEAI */
    uint32_t idx = buf_cnt * NEAI_INPUT_AXIS_NUMBER;
    mems_buffer[idx] = (float)acc.x;
    mems_buffer[idx + 1] = (float)acc.y;
    mems_buffer[idx + 2] = (float)acc.z;
    mems_buffer[idx + 3] = (float)gyr.x / 1000.0f;
    mems_buffer[idx + 4] = (float)gyr.y / 1000.0f;
    mems_buffer[idx + 5] = (float)gyr.z / 1000.0f;

    buf_cnt++;
    if (buf_cnt >= NEAI_INPUT_SIGNAL_LENGTH) {
      buf_cnt = 0;
      uint8_t sim = 0;
      enum neai_state st = neai_anomalydetection_detect(mems_buffer, &sim);
      if (st == NEAI_OK) {
        int ok = (sim >= 90) ? 1 : 0;
        ledPeriod = ok ? LED_INFERENCE_PERIOD : LED_ANOMALY_PERIOD;
        printf("NEAI sim=%d ok=%d\r\n", (int)sim, ok);
      }
    }

    /* ToF co MEMS_PER_TOF próbek */
    tof_cnt++;
    if (tof_cnt >= MEMS_PER_TOF) {
      tof_cnt = 0;
      PrintToFLine();
    }
  }
}

/* ===========================================================================
 * Przerwania GPIO
 * ===========================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case GPIO_PIN_4:
    memsDataReady = SET;
    break;          /* PB4: MEMS INT1 */
  case GPIO_PIN_13: /* PC13: przycisk B1 */
    if (appState == LOG)
      appState = LEARN;
    else if (appState == INFERENCE)
      appState = LOG;
    break;
  default:
    break;
  }
}

/* ===========================================================================
 * SysTick – mruganie LED
 * ===========================================================================*/
void HAL_IncTick(void) {
  uwTick += (uint32_t)uwTickFreq;
  if (ledPeriodCnt > 0)
    ledPeriodCnt--;
  else {
    ledPeriodCnt = ledPeriod;
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}

/* ===========================================================================
 * Obsługa błędów
 * ===========================================================================*/
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  (void)file;
  (void)line;
}
#endif
