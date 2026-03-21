/**
 * @file    main.c
 * @brief   GPS NEO-6M na STM32L476RG (NUCLEO-L476RG)
 *
 * Podłączenie:
 *   NEO-6M TX  →  PA10  (USART1_RX, CN10 pin 33)
 *   NEO-6M RX  →  PA9   (USART1_TX, CN10 pin 21) [opcjonalnie]
 *   NEO-6M VCC →  3V3 lub 5V (moduł akceptuje oba)
 *   NEO-6M GND →  GND
 *
 * UART:
 *   USART1 (PA9/PA10) @ 9600 baud  – GPS
 *   USART2 (PA2/PA3)  @ 115200 baud – printf/debug przez ST-Link
 *
 * CubeMX / STM32CubeIDE:
 *   1. USART1: Asynchronous, 9600 8N1, Global Interrupt ON
 *   2. USART2: Asynchronous, 115200 8N1
 *   3. Zegar: 80 MHz (HSI + PLL, jak w innych projektach)
 *   4. Dodaj gps.c do projektu, gps.h do include path
 */

#include "main.h"
#include "gps.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 * Uchwyty peryferyjne
 * =========================================================================*/
UART_HandleTypeDef huart1;   /* GPS  @ 9600  */
UART_HandleTypeDef huart2;   /* Debug @ 115200 */

/* =========================================================================
 * Prototypy funkcji pomocniczych
 * =========================================================================*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void GPS_PrintData(void);

/* =========================================================================
 * printf → USART2 (ST-Link)
 * =========================================================================*/
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* =========================================================================
 * main()
 * =========================================================================*/
int main(void)
{
    /* 1. Inicjalizacja HAL (SysTick, rejestry) */
    HAL_Init();

    /* 2. Konfiguracja zegara 80 MHz */
    SystemClock_Config();

    /* 3. GPIO (LED PA5) */
    MX_GPIO_Init();

    /* 4. USART2 – debug przez ST-Link (musi być przed pierwszym printf) */
    MX_USART2_UART_Init();

    printf("\r\n");
    printf("  GPS NEO-6M + STM32L476RG\r\n");
    printf("[1/4] HAL init        OK\r\n");
    printf("[2/4] Zegar 80 MHz    OK\r\n");
    printf("[3/4] GPIO (LED PA5)  OK\r\n");
    printf("[4/4] USART2 debug    OK  (115200 baud)\r\n");

    /* 5. USART1 – odbiór danych GPS (przerwanie) */
    MX_USART1_UART_Init();
    printf("[5/5] USART1 GPS      OK  (PA10=RX, 9600 baud)\r\n");

    /* 6. Start parsera GPS – uruchamia HAL_UART_Receive_IT */
    GPS_Init();
    printf("[GPS] Parser uruchomiony, czekam na dane...\r\n");
    printf("[GPS] Zimny start: do 30s na zewnątrz, kilka minut w środku\r\n");
    printf("----------------------------------------\r\n\r\n");

    uint32_t last_print = 0;
    uint32_t last_tick  = 0;
    uint8_t  led_state  = 0;

    while (1)
    {
        /* Przetwarzaj dane z bufora UART (nie blokuje) */
        GPS_Process();

        /* LED miga: wolno (1 Hz) gdy brak fixa, szybko (4 Hz) gdy fix */
        uint32_t blink = GPS_IsValid() ? 125 : 500;
        if (HAL_GetTick() - last_tick >= blink)
        {
            last_tick  = HAL_GetTick();
            led_state ^= 1;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state);
        }

        /* Drukuj dane co 1 sekundę */
        if (HAL_GetTick() - last_print >= 1000)
        {
            last_print = HAL_GetTick();
            GPS_PrintData();
        }
    }
}

/* =========================================================================
 * Wyświetlanie danych GPS na USART2
 * =========================================================================*/
static void GPS_PrintData(void)
{
    const GPS_Data_t *g = GPS_GetData();
    uint32_t t = HAL_GetTick();

    printf("[%6lu ms] ", (unsigned long)t);

    if (g->sentences_ok == 0 && g->sentences_err == 0)
    {
        printf("BRAK DANYCH z modulu GPS – sprawdz polaczenie PA10\r\n");
        return;
    }

    printf("Zdania: %lu OK / %lu ERR  |  ",
           (unsigned long)g->sentences_ok,
           (unsigned long)g->sentences_err);

    if (g->sentences_err > 0)
        printf("[!] Bledy sumy kontrolnej – zly kabel lub baudrate\r\n");

    if (g->fix_quality == 0)
    {
        printf("FIX: BRAK  |  Satelity widoczne: %d\r\n", g->satellites);
        return;
    }

    printf("FIX: %s  |  Sat: %d  |  HDOP: %.1f\r\n",
           g->fix_quality == 2 ? "DGPS" : "GPS",
           g->satellites, g->hdop);

    if (!g->valid)
    {
        printf("             Status: VOID (pozycja niedostepna)\r\n");
        return;
    }

    int lat_int = (int)(g->latitude * 1000000);
    int lon_int = (int)(g->longitude * 1000000);

    printf("TEST RAW: Lat_int: %d, Lon_int: %d\r\n", lat_int, lon_int);

    printf("             Czas:  %02d:%02d:%02d UTC   Data: %02d.%02d.%04d\r\n",
           g->hour, g->minute, g->second,
           g->day, g->month, g->year);

    printf("             Lat:   %.6f %c\r\n",
           g->latitude  >= 0 ?  g->latitude  : -g->latitude,
           g->latitude  >= 0 ? 'N' : 'S');

    printf("             Lon:   %.6f %c\r\n",
           g->longitude >= 0 ?  g->longitude : -g->longitude,
           g->longitude >= 0 ? 'E' : 'W');

    printf("             Alt:   %.1f m    Predkosc: %.1f km/h    Kurs: %.1f deg\r\n",
           g->altitude, g->speed_kmh, g->course);
}

/* =========================================================================
 * Callback UART – wywoływany przez HAL po odebraniu bajtu
 * =========================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        GPS_UART_RxCallback();
}

/* =========================================================================
 * Inicjalizacja zegara: 80 MHz (HSI 16 MHz → PLL × 10 / 2)
 * =========================================================================*/
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
        Error_Handler();

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 10;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
        Error_Handler();
}

/* =========================================================================
 * GPIO: PA5 = LED2 (LD2) na NUCLEO
 * =========================================================================*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5: LED2 */
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* =========================================================================
 * USART1: GPS, 9600 baud, PA9=TX, PA10=RX, przerwanie
 * =========================================================================*/
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 9600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* HAL_UART_MspInit – konfiguracja GPIO i NVIC dla USART1 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9  = USART1_TX (AF7)
         * PA10 = USART1_RX (AF7) */
        GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (huart->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA2 = USART2_TX (AF7)
         * PA3 = USART2_RX (AF7) – połączone ze ST-Link */
        GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        /* USART2 bez przerwania – tylko TX (printf) */
    }
}

/* =========================================================================
 * USART2: debug/printf przez ST-Link, 115200 baud
 * =========================================================================*/
static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

/* =========================================================================
 * Obsługa przerwania USART1
 * =========================================================================*/
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* =========================================================================
 * Obsługa błędów
 * =========================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif