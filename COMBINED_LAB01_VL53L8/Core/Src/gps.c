/**
 * @file    gps.c
 * @brief   Parser NMEA dla NEO-6M / STM32L476RG
 *
 * Architektura:
 *   [NEO-6M TX] → [PA10 USART1_RX] → przerwanie UART
 *        ↓
 *   rx_buf[GPS_RX_BUF_SIZE]  – kołowy bufor przerwania (ISR pisze, GPS_Process czyta)
 *        ↓
 *   line_buf[GPS_LINE_BUF_SIZE] – jedno kompletne zdanie NMEA ($…\n)
 *        ↓
 *   gps_parse_line()          – sprawdza checksum, rozpoznaje typ, parsuje
 *        ↓
 *   gps_data                  – zaktualizowane dane GPS
 */

#include "gps.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* =========================================================================
 * Zmienne wewnętrzne
 * =========================================================================*/

/* Bufor kołowy wypełniany przez ISR */
static volatile uint8_t  rx_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t rx_write = 0;  /* ISR pisze tutaj  */
static volatile uint16_t rx_read  = 0;  /* Process czyta stąd */

/* Bufor jednego zdania NMEA */
static char    line_buf[GPS_LINE_BUF_SIZE];
static uint8_t line_pos = 0;
static bool    in_sentence = false;

/* Bajt odbierany przez przerwanie */
uint8_t rx_byte;  /* nie-static: potrzebny w HAL_UART_ErrorCallback (main.c) */

/* Dane GPS – dostępne dla użytkownika */
static GPS_Data_t gps_data;

/* =========================================================================
 * Helpers
 * =========================================================================*/

/**
 * @brief Konwersja NMEA DDMM.mmmm → stopnie dziesiętne
 */
static double nmea_to_deg(const char *s)
{
    if (!s || s[0] == '\0') return 0.0;
    double raw = atof(s);
    int    deg = (int)(raw / 100);
    double min = raw - deg * 100.0;
    return deg + min / 60.0;
}

/**
 * @brief Sprawdzenie sumy kontrolnej XOR zdania NMEA.
 *        Zdanie musi zaczynać się od '$' i zawierać '*HH'.
 * @return true jeśli checksum poprawny
 */
static bool nmea_checksum_ok(const char *sentence)
{
    /* Szukaj '*' */
    const char *star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) return false;

    /* Oblicz XOR między '$' a '*' */
    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++)
        calc ^= (uint8_t)*p;

    /* Odczytaj podany checksum */
    uint8_t given = (uint8_t)strtol(star + 1, NULL, 16);
    return calc == given;
}

/**
 * @brief Bezpieczna wersja strtok obsługująca puste pola (,,)
 *        Standard strtok() pomija kolejne separatory – ta nie.
 */
static char *strtok_nmea(char *str, char delim)
{
    static char *ptr = NULL;
    if (str) ptr = str;
    if (!ptr) return NULL;

    char *start = ptr;
    char *p = ptr;
    while (*p && *p != delim) p++;

    if (*p == delim) { *p = '\0'; ptr = p + 1; }
    else              ptr = NULL;

    return start;
}

/* =========================================================================
 * Parsowanie konkretnych zdań
 * =========================================================================*/

/**
 * @brief Parsuje $GPRMC / $GNRMC
 *
 * Format:
 *   $GPRMC,HHMMSS.ss,A,DDMM.mm,N,DDDMM.mm,E,spd,crs,DDMMYY,mag,dir[,mode]*hh
 *   pola:    0       1  2       3 4        5 6   7   8       9   10
 */
static void parse_gprmc(char *fields[])
{
    /* Pole 1: czas HHMMSS.ss */
    const char *t = fields[1];
    if (t && strlen(t) >= 6) {
        gps_data.hour   = (t[0]-'0')*10 + (t[1]-'0');
        gps_data.minute = (t[2]-'0')*10 + (t[3]-'0');
        gps_data.second = (t[4]-'0')*10 + (t[5]-'0');
    }

    /* Pole 2: status A=active (fix), V=void */
    bool active = (fields[2] && fields[2][0] == 'A');
    gps_data.valid = active;
    if (active) gps_data.last_fix_tick = HAL_GetTick();

    /* Pole 3+4: szerokość geograficzna */
    if (fields[3] && fields[3][0]) {
        gps_data.latitude = nmea_to_deg(fields[3]);
        if (fields[4] && fields[4][0] == 'S')
            gps_data.latitude = -gps_data.latitude;
    }

    /* Pole 5+6: długość geograficzna */
    if (fields[5] && fields[5][0]) {
        gps_data.longitude = nmea_to_deg(fields[5]);
        if (fields[6] && fields[6][0] == 'W')
            gps_data.longitude = -gps_data.longitude;
    }

    /* Pole 7: prędkość w węzłach → km/h */
    if (fields[7] && fields[7][0])
        gps_data.speed_kmh = atof(fields[7]) * 1.852f;

    /* Pole 8: kurs */
    if (fields[8] && fields[8][0])
        gps_data.course = atof(fields[8]);

    /* Pole 9: data DDMMYY */
    const char *d = fields[9];
    if (d && strlen(d) >= 6) {
        gps_data.day   = (d[0]-'0')*10 + (d[1]-'0');
        gps_data.month = (d[2]-'0')*10 + (d[3]-'0');
        gps_data.year  = 2000 + (d[4]-'0')*10 + (d[5]-'0');
    }
}

/**
 * @brief Parsuje $GPGGA / $GNGGA
 *
 * Format:
 *   $GPGGA,HHMMSS.ss,DDMM.mm,N,DDDMM.mm,E,q,nn,hdop,alt,M,geoid,M,,*hh
 *   pola:   0        1       2 3        4 5 6 7  8    9   10
 */
static void parse_gpgga(char *fields[])
{
    /* Pole 6: jakość fixa (0=brak, 1=GPS, 2=DGPS) */
    if (fields[6] && fields[6][0])
        gps_data.fix_quality = (uint8_t)atoi(fields[6]);

    /* Pole 7: liczba satelitów */
    if (fields[7] && fields[7][0])
        gps_data.satellites = (uint8_t)atoi(fields[7]);

    /* Pole 8: HDOP */
    if (fields[8] && fields[8][0])
        gps_data.hdop = atof(fields[8]);

    /* Pole 9: wysokość n.p.m. w metrach */
    if (fields[9] && fields[9][0])
        gps_data.altitude = atof(fields[9]);
}

/* =========================================================================
 * Główny parser linii
 * =========================================================================*/

/**
 * @brief Przetwarza jedno kompletne zdanie NMEA z line_buf.
 */
static void gps_parse_line(void)
{
    if (line_buf[0] != '$') return;

    /* Pokaż surowe zdanie NMEA w terminalu */
    // printf("[NMEA] %s\r\n", line_buf);

    if (!nmea_checksum_ok(line_buf)) {
        gps_data.sentences_err++;
        // printf("[NMEA] !!! Zla suma kontrolna: %s\r\n", line_buf);
        return;
    }

    /* Usuń '*HH\r\n' z końca (przed tokenizacją) */
    char *star = strchr(line_buf, '*');
    if (star) *star = '\0';

    /* Tokenizacja – maksymalnie 20 pól */
    char *fields[20];
    int   n = 0;
    char *tok = strtok_nmea(line_buf, ',');
    while (tok && n < 20) { fields[n++] = tok; tok = strtok_nmea(NULL, ','); }
    while (n < 20) fields[n++] = NULL;

    /* Rozpoznanie typu zdania (ignoruj prefix GP/GN/GL/BD) */
    const char *type = fields[0] + 1;  /* pomiń '$' */
    if (strlen(type) >= 5) type += 2;  /* pomiń GP/GN/etc. */

    if      (!strncmp(type, "RMC", 3)) parse_gprmc(fields);
    else if (!strncmp(type, "GGA", 3)) parse_gpgga(fields);
    /* Inne zdania ignorowane */

    gps_data.sentences_ok++;
}

/* =========================================================================
 * Interfejs publiczny
 * =========================================================================*/

void GPS_Init(void)
{
    memset(&gps_data, 0, sizeof(gps_data));
    gps_data.valid = false;
    rx_write = rx_read = 0;
    line_pos = 0;
    in_sentence = false;

    /* Start odbioru pierwszego bajtu przez przerwanie */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/**
 * @brief Wywoływany z HAL_UART_RxCpltCallback – wpisuje bajt do bufora kołowego.
 */
void GPS_UART_RxCallback(void)
{
    uint16_t next = (rx_write + 1) % GPS_RX_BUF_SIZE;
    if (next != rx_read) {          /* bufor nie pełny */
        rx_buf[rx_write] = rx_byte;
        rx_write = next;
    }
    /* Uruchom następny odbiór */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void GPS_Process(void)
{
    /* Przetwórz wszystkie bajty dostępne w buforze kołowym */
    while (rx_read != rx_write) {
        uint8_t c = rx_buf[rx_read];
        rx_read = (rx_read + 1) % GPS_RX_BUF_SIZE;

        if (c == '$') {
            /* Nowe zdanie */
            in_sentence = true;
            line_pos = 0;
            line_buf[line_pos++] = '$';
        } else if (in_sentence) {
            if (c == '\n' || c == '\r') {
                /* Koniec zdania */
                line_buf[line_pos] = '\0';
                if (line_pos > 5)   /* minimalna długość sensownego zdania */
                    gps_parse_line();
                in_sentence = false;
            } else {
                if (line_pos < GPS_LINE_BUF_SIZE - 1)
                    line_buf[line_pos++] = c;
                else
                    in_sentence = false;  /* przepełnienie – porzuć */
            }
        }
    }
}

const GPS_Data_t *GPS_GetData(void)
{
    return &gps_data;
}

bool GPS_IsValid(void)
{
    return gps_data.valid && (HAL_GetTick() - gps_data.last_fix_tick < 2000);
}
