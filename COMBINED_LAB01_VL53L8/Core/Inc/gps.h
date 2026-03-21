/**
 * @file    gps.h
 * @brief   Parser NMEA dla modułu NEO-6M na STM32L476RG
 *
 * Obsługiwane zdania NMEA:
 *   $GPRMC / $GNRMC – czas, data, pozycja, prędkość
 *   $GPGGA / $GNGGA – pozycja, fix, liczba satelitów, wysokość
 *
 * Użycie:
 *   1. Wywołaj GPS_Init() w main() po inicjalizacji HAL i GPIO
 *   2. Wołaj GPS_Process() w pętli głównej
 *   3. Dane dostępne przez gps_data (globalny) lub GPS_GetData()
 */

#ifndef GPS_H
#define GPS_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Rozmiary buforów
 * =========================================================================*/
#define GPS_RX_BUF_SIZE   256   /* kołowy bufor wejściowy UART */
#define GPS_LINE_BUF_SIZE 128   /* bufor jednego zdania NMEA   */

/* =========================================================================
 * Dane GPS
 * =========================================================================*/
typedef struct {
    /* Czas UTC */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;

    /* Data */
    uint8_t  day;
    uint8_t  month;
    uint16_t year;

    /* Pozycja (stopnie dziesiętne, dodatnie = N/E, ujemne = S/W) */
    double   latitude;   /* -90.0 … +90.0  */
    double   longitude;  /* -180.0 … +180.0 */
    float    altitude;   /* metry (z GPGGA)  */

    /* Prędkość i kurs */
    float    speed_kmh;  /* km/h             */
    float    course;     /* stopnie (0–360)  */

    /* Jakość sygnału */
    uint8_t  satellites; /* liczba satelitów widzianych */
    uint8_t  fix_quality;/* 0=brak, 1=GPS, 2=DGPS       */
    float    hdop;       /* Horizontal Dilution of Precision */

    /* Flaga ważności danych */
    bool     valid;      /* true jeśli ostatni $GPRMC miał status A */
    uint32_t last_fix_tick; /* HAL_GetTick() ostatniego poprawnego pomiaru */

    /* Liczniki */
    uint32_t sentences_ok;  /* zdania sparsowane poprawnie */
    uint32_t sentences_err; /* zdania z błędem sumy kontrolnej */
} GPS_Data_t;

/* =========================================================================
 * API
 * =========================================================================*/

/**
 * @brief Inicjalizacja parsera (wywołaj raz po MX_USART1_UART_Init).
 */
void GPS_Init(void);

/**
 * @brief Przetwarzanie danych UART – wywołuj w pętli głównej.
 *        Nie blokuje.
 */
void GPS_Process(void);

/**
 * @brief Zwraca wskaźnik do aktualnych danych GPS (tylko do odczytu).
 */
const GPS_Data_t *GPS_GetData(void);

/**
 * @brief Sprawdza czy dane GPS są aktualne (fix w ciągu ostatnich 2 sekund).
 */
bool GPS_IsValid(void);

/**
 * @brief Callback wywoływany przez HAL_UART_RxCpltCallback.
 *        Podłącz go do głównego callbacku UART.
 */
void GPS_UART_RxCallback(void);

/* Eksportowany uchwyt UART – do użycia w main.c */
extern UART_HandleTypeDef huart4;

#endif /* GPS_H */
