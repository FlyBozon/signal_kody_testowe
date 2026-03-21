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
#include "spi.h"
#include "gps.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "user_diskio.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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
#define SENSOR_LOGGING_ENABLED 0
#define UART_CSV_LOGGING_ENABLED 1   /* 1 = wysyłaj surowy log CSV przez UART */

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

FATFS SDFatFs;
FIL MyFile;
char SDPath[4];
uint8_t sd_mounted = 0;
uint32_t last_gps_print = 0;

/* Kalibracja czasu GPS */
uint8_t  gps_time_synced = 0;
uint32_t sync_tick = 0;
uint16_t sync_year = 0;
uint8_t  sync_month = 0, sync_day = 0;
uint8_t  sync_hour = 0, sync_min = 0, sync_sec = 0;

#define CSV_LOG_FILE "log.csv"

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
void GPS_PrintAndLog(void);

/* CSV helpery */
void GPS_TrySync(void);
void get_timestamp(uint32_t tick, char *buf, int bufsize);
void csv_pad(FIL *f, int n, uint8_t to_uart);
void csv_write_header(void);

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
  MX_USART1_UART_Init();
  MX_SPI2_Init();

  /* Wyłączenie buforowania stdout, aby minicom od razu wyrzucał dane */
  setbuf(stdout, NULL);

  /* Link the SD disk I/O driver */
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 1) == FR_OK)
    {
      sd_mounted = 1;
      printf("SD Card mounted successfully.\r\n");
    }
    else
    {
      printf("[ERROR] SD Card mount failed!\r\n");
    }
  }

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

  GPS_Init();
  printf("GPS_Init done (USART1 PA10=RX, 9600 baud)\r\n");

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
 * PrintToFLine – pobiera jedną klatkę 8×8:
 *   UART: TDIST + TSIG (dwie linie, jak dotychczas)
 *   SD:   jeden wiersz CSV typu TOF (128 wartości: D00–D63, S00–S63)
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
    return;

  /* Clamp distances */
  int dist[TOF_RESOLUTION];
  for (i = 0; i < TOF_RESOLUTION; i++) {
    dist[i] = (int)Result.ZoneResult[i].Distance[0];
    if (dist[i] < 0 || dist[i] > (int)DISTANCE_MAX)
      dist[i] = (int)DISTANCE_MAX;
  }

#if SENSOR_LOGGING_ENABLED
  printf("TDIST");
  for (i = 0; i < TOF_RESOLUTION; i++)
    printf(" %d", dist[i]);
  printf("\r\n");

  printf("TSIG");
  for (i = 0; i < TOF_RESOLUTION; i++)
    printf(" %d", (int)Result.ZoneResult[i].Signal[0]);
  printf("\r\n");
#endif

  /* CSV na SD (i opcjonalnie UART) – typ TOF: 14 pustych (MEMS+NEAI+GPS) + 128 wartości */
  if (gps_time_synced) {
#if UART_CSV_LOGGING_ENABLED
    char ts[32];
    get_timestamp(HAL_GetTick(), ts, sizeof(ts));
    printf("%s;TOF", ts);
    csv_pad(NULL, 14, 1);
    for (i = 0; i < TOF_RESOLUTION; i++) printf(";%d", dist[i]);
    for (i = 0; i < TOF_RESOLUTION; i++) printf(";%d", (int)Result.ZoneResult[i].Signal[0]);
    printf("\r\n");
#endif

    if (sd_mounted) {
      if (f_open(&MyFile, CSV_LOG_FILE, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        char ts[32];
        get_timestamp(HAL_GetTick(), ts, sizeof(ts));
        f_printf(&MyFile, "%s;TOF", ts);
        csv_pad(&MyFile, 14, 0);  /* kol 2–15 puste */
        for (i = 0; i < TOF_RESOLUTION; i++)
          f_printf(&MyFile, ";%d", dist[i]);
        for (i = 0; i < TOF_RESOLUTION; i++)
          f_printf(&MyFile, ";%d", (int)Result.ZoneResult[i].Signal[0]);
        f_printf(&MyFile, "\n");
        f_close(&MyFile);
      }
    }
  }
}

/* ===========================================================================
 * LOG – ciągłe dane z MEMS (30 Hz) i ToF (co 10 próbek, ~3 Hz)
 * ===========================================================================*/
void Log(void) {
  static uint32_t tof_cnt = 0;
  IKS4A1_MOTION_SENSOR_Axes_t acc, gyr;

  ledPeriod = LED_LOG_PERIOD;

  /* Przetwarzaj dane GPS z bufora UART */
  GPS_Process();

  /* Logowanie GPS co 1 sekundę */
  if (HAL_GetTick() - last_gps_print >= 1000) {
    last_gps_print = HAL_GetTick();
    GPS_PrintAndLog();
  }

  if (memsDataReady != RESET) {
    memsDataReady = RESET;

    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &acc);
    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &gyr);

    int ax = (int)acc.x, ay = (int)acc.y, az = (int)acc.z;
    int gx = (int)(gyr.x / 1000), gy = (int)(gyr.y / 1000), gz = (int)(gyr.z / 1000);

#if SENSOR_LOGGING_ENABLED
    printf("MEMS Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\r\n", ax, ay, az, gx, gy, gz);
#endif

    /* CSV na SD (i UART) – typ MEMS: 6 wartości + 136 pustych */
  if (gps_time_synced) {
#if UART_CSV_LOGGING_ENABLED
      char ts[32];
      get_timestamp(HAL_GetTick(), ts, sizeof(ts));
      printf("%s;MEMS;%d;%d;%d;%d;%d;%d", ts, ax, ay, az, gx, gy, gz);
      csv_pad(NULL, 136, 1);
      printf("\r\n");
#endif

      if (sd_mounted) {
        if (f_open(&MyFile, CSV_LOG_FILE, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
          char ts[32];
          get_timestamp(HAL_GetTick(), ts, sizeof(ts));
          f_printf(&MyFile, "%s;MEMS;%d;%d;%d;%d;%d;%d", ts, ax, ay, az, gx, gy, gz);
          csv_pad(&MyFile, 136, 0);  /* kol 8–143 puste */
          f_printf(&MyFile, "\n");
          f_close(&MyFile);
        }
      }
    }

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

  /* Przetwarzaj dane GPS z bufora UART */
  GPS_Process();

  /* Logowanie GPS co 1 sekundę */
  if (HAL_GetTick() - last_gps_print >= 1000) {
    last_gps_print = HAL_GetTick();
    GPS_PrintAndLog();
  }

  if (memsDataReady != RESET) {
    memsDataReady = RESET;

    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_ACCELERO, &acc);
    IKS4A1_MOTION_SENSOR_GetAxes(IKS4A1_LSM6DSV16X_0, MOTION_GYRO, &gyr);

    int ax = (int)acc.x, ay = (int)acc.y, az = (int)acc.z;
    int gx = (int)(gyr.x / 1000), gy = (int)(gyr.y / 1000), gz = (int)(gyr.z / 1000);

#if SENSOR_LOGGING_ENABLED
    printf("MEMS Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\r\n", ax, ay, az, gx, gy, gz);
#endif

    /* CSV na SD – typ MEMS */
    if (gps_time_synced) {
#if UART_CSV_LOGGING_ENABLED
      char ts[32];
      get_timestamp(HAL_GetTick(), ts, sizeof(ts));
      printf("%s;MEMS;%d;%d;%d;%d;%d;%d", ts, ax, ay, az, gx, gy, gz);
      csv_pad(NULL, 136, 1);
      printf("\r\n");
#endif

      if (sd_mounted) {
        if (f_open(&MyFile, CSV_LOG_FILE, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
          char ts[32];
          get_timestamp(HAL_GetTick(), ts, sizeof(ts));
          f_printf(&MyFile, "%s;MEMS;%d;%d;%d;%d;%d;%d", ts, ax, ay, az, gx, gy, gz);
          csv_pad(&MyFile, 136, 0);
          f_printf(&MyFile, "\n");
          f_close(&MyFile);
        }
      }
    }

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
#if SENSOR_LOGGING_ENABLED
        printf("NEAI sim=%d ok=%d\r\n", (int)sim, ok);
#endif
        /* CSV na SD – typ NEAI: 6 pustych (MEMS) + 2 wartości + 134 pustych */
        if (gps_time_synced) {
#if UART_CSV_LOGGING_ENABLED
          char ts2[32];
          get_timestamp(HAL_GetTick(), ts2, sizeof(ts2));
          printf("%s;NEAI", ts2);
          csv_pad(NULL, 6, 1);
          printf(";%d;%d", (int)sim, ok);
          csv_pad(NULL, 134, 1);
          printf("\r\n");
#endif
          if (sd_mounted) {
            if (f_open(&MyFile, CSV_LOG_FILE, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
              char ts2[32];
              get_timestamp(HAL_GetTick(), ts2, sizeof(ts2));
              f_printf(&MyFile, "%s;NEAI", ts2);
              csv_pad(&MyFile, 6, 0);  /* kol 2–7 puste */
              f_printf(&MyFile, ";%d;%d", (int)sim, ok);
              csv_pad(&MyFile, 134, 0); /* kol 10–143 puste */
              f_printf(&MyFile, "\n");
              f_close(&MyFile);
            }
          }
        }
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
 * Callback UART – przerwanie po odebraniu bajtu (GPS na UART4)
 * ===========================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1)
    GPS_UART_RxCallback();
}

/* ===========================================================================
 * ErrorCallback UART – wznowienie odbioru po błędzie (overrun, framing itp.)
 * Bez tego HAL zatrzymuje Receive_IT na zawsze po pierwszym błędzie.
 * ===========================================================================*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    /* Wyczyść flagi błędów i wznów nasłuchiwanie */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    extern uint8_t rx_byte;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}

/* ===========================================================================
 * GPS_TrySync – przy pierwszym ważnym fixie GPS zapamiętaj czas i zapisz header
 * ===========================================================================*/
void GPS_TrySync(void) {
  if (gps_time_synced) return;
  const GPS_Data_t *g = GPS_GetData();
  if (!g->valid || g->year < 2020) return;

  sync_tick  = g->last_fix_tick;
  sync_year  = g->year;
  sync_month = g->month;
  sync_day   = g->day;
  sync_hour  = g->hour;
  sync_min   = g->minute;
  sync_sec   = g->second;
  gps_time_synced = 1;

  printf("[GPS SYNC] %04d-%02d-%02d %02d:%02d:%02d UTC  -> SD logging started\r\n",
         sync_year, sync_month, sync_day, sync_hour, sync_min, sync_sec);

  csv_write_header();
}

/* ===========================================================================
 * get_timestamp – oblicza czas ISO 8601 z tiku MCU skalibrowanego o GPS UTC
 * ===========================================================================*/
void get_timestamp(uint32_t tick, char *buf, int bufsize) {
  uint32_t elapsed_ms = tick - sync_tick;
  uint32_t elapsed_s  = elapsed_ms / 1000;
  uint32_t ms         = elapsed_ms % 1000;

  uint32_t base_s = (uint32_t)sync_hour * 3600u
                  + (uint32_t)sync_min  * 60u
                  + (uint32_t)sync_sec;
  uint32_t total_s = base_s + elapsed_s;

  uint8_t h = (uint8_t)((total_s / 3600) % 24);
  uint8_t m = (uint8_t)((total_s % 3600) / 60);
  uint8_t s = (uint8_t)(total_s % 60);

  snprintf(buf, bufsize, "%04d-%02d-%02dT%02d:%02d:%02d.%03lu",
           (int)sync_year, (int)sync_month, (int)sync_day,
           (int)h, (int)m, (int)s, (unsigned long)ms);
}

/* ===========================================================================
 * csv_pad – zapisuje n separatorów ';' (puste kolumny CSV)
 * ===========================================================================*/
void csv_pad(FIL *f, int n, uint8_t to_uart) {
  static const char s16[] = ";;;;;;;;;;;;;;;;";
  while (n >= 16) { 
    if (f) f_puts(s16, f); 
    if (to_uart) printf("%s", s16);
    n -= 16; 
  }
  if (n > 0) {
    char tmp[17];
    memset(tmp, ';', n);
    tmp[n] = '\0';
    if (f) f_puts(tmp, f);
    if (to_uart) printf("%s", tmp);
  }
}

/* ===========================================================================
 * csv_write_header – nagłówek CSV (wywoływany raz przy GPS sync)
 * ===========================================================================*/
void csv_write_header(void) {
#if UART_CSV_LOGGING_ENABLED
  printf("timestamp;type;Ax;Ay;Az;Gx;Gy;Gz;sim;ok;fix;sat;lat;lon;alt;spd");
  for (int i = 0; i < 64; i++) printf(";D%02d", i);
  for (int i = 0; i < 64; i++) printf(";S%02d", i);
  printf("\r\n");
#endif

  if (!sd_mounted) return;
  if (f_open(&MyFile, CSV_LOG_FILE, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return;

  f_puts("timestamp;type"
         ";Ax;Ay;Az;Gx;Gy;Gz"
         ";sim;ok"
         ";fix;sat;lat;lon;alt;spd", &MyFile);

  char col[8];
  int i;
  for (i = 0; i < 64; i++) { snprintf(col, sizeof(col), ";D%02d", i); f_puts(col, &MyFile); }
  for (i = 0; i < 64; i++) { snprintf(col, sizeof(col), ";S%02d", i); f_puts(col, &MyFile); }
  f_puts("\n", &MyFile);
  f_close(&MyFile);
}

/* ===========================================================================
 * GPS_PrintAndLog – co 1s drukuje dane GPS na UART i zapisuje CSV na SD
 * ===========================================================================*/
void GPS_PrintAndLog(void) {
  const GPS_Data_t *g = GPS_GetData();

  /* Spróbuj zsynchronizować czas (jednorazowo) */
  GPS_TrySync();

  if (g->sentences_ok == 0 && g->sentences_err == 0) {
    if (!gps_time_synced)
      printf("[STATUS] GPS brak danych – sprawdz PA10\r\n");
    return;
  }

  /* Przed synchronizacją – pokaż status GPS na UART (nie-CSV) */
  if (!gps_time_synced) {
    printf("[STATUS] GPS fix=%d sat=%d ok=%lu err=%lu – czekam na date...\r\n",
           (int)g->fix_quality, (int)g->satellites,
           (unsigned long)g->sentences_ok,
           (unsigned long)g->sentences_err);
  }

  int lat_i = (int)(g->latitude * 1000000);
  int lon_i = (int)(g->longitude * 1000000);
  int alt_i = (int)(g->altitude * 10);
  int spd_i = (int)(g->speed_kmh * 10);

#if SENSOR_LOGGING_ENABLED
  if (gps_time_synced) {
    printf("GPS fix=%d sat=%d lat=%d lon=%d alt=%d spd=%d ok=%lu err=%lu\r\n",
           (int)g->fix_quality, (int)g->satellites,
           lat_i, lon_i, alt_i, spd_i,
           (unsigned long)g->sentences_ok,
           (unsigned long)g->sentences_err);
  }
#endif

  /* CSV na SD – typ GPS: 8 pustych (MEMS+NEAI) + 6 wartości + 128 pustych */
  if (gps_time_synced) {
#if UART_CSV_LOGGING_ENABLED
    char ts[32];
    get_timestamp(HAL_GetTick(), ts, sizeof(ts));
    printf("%s;GPS", ts);
    csv_pad(NULL, 8, 1);
    printf(";%d;%d;%d;%d;%d;%d",
            (int)g->fix_quality, (int)g->satellites,
            lat_i, lon_i, alt_i, spd_i);
    csv_pad(NULL, 128, 1);
    printf("\r\n");
#endif

    if (sd_mounted) {
      if (f_open(&MyFile, CSV_LOG_FILE, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        char ts[32];
        get_timestamp(HAL_GetTick(), ts, sizeof(ts));
        f_printf(&MyFile, "%s;GPS", ts);
        csv_pad(&MyFile, 8, 0);  /* kol 2–9 puste (MEMS + NEAI) */
        f_printf(&MyFile, ";%d;%d;%d;%d;%d;%d",
                 (int)g->fix_quality, (int)g->satellites,
                 lat_i, lon_i, alt_i, spd_i);
        csv_pad(&MyFile, 128, 0); /* kol 16–143 puste (TOF) */
        f_printf(&MyFile, "\n");
        f_close(&MyFile);
      }
    }
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
