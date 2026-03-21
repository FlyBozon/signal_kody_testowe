# GPS NEO-6M na STM32L476RG

## Schemat podłączenia

```
NEO-6M          NUCLEO-L476RG
──────          ─────────────
TX      ──────► PA10  (USART1_RX, CN10 pin 33)
RX      ◄──────  PA9  (USART1_TX, CN10 pin 21)  ← opcjonalne
VCC     ──────► 3V3 lub 5V
GND     ──────► GND
```

> NEO-6M działa z 3.3V logiką. Zasilanie z 3V3 pinu NUCLEO jest wystarczające.

## Pliki projektu

| Plik | Opis |
|------|------|
| `Core/Inc/gps.h` | Struktura danych GPS + API |
| `Core/Src/gps.c` | Parser NMEA (GPRMC + GPGGA), obsługa UART IT |
| `Core/Src/main.c` | Główny program – init, pętla, wyświetlanie |
| `Core/Inc/main.h` | Minimalny nagłówek |

## Konfiguracja STM32CubeIDE

### Nowy projekt z CubeMX:
1. Board: NUCLEO-L476RG
2. USART1: Mode = Asynchronous, BaudRate = 9600, **Global Interrupt = ON**
3. USART2: Mode = Asynchronous, BaudRate = 115200 (ST-Link debug)
4. Clock: 80 MHz (HSI + PLL)
5. Generuj kod → zastąp `Core/Src/main.c` naszym plikiem
6. Dodaj `gps.c` do projektu, `gps.h` do include path

### Bez CubeMX (gotowy Makefile):
Skopiuj `Drivers/` i `Core/Startup/` z `LAB01_full_app/`.
Potrzebne pliki HAL (wystarczą):
- `stm32l4xx_hal.c`, `stm32l4xx_hal_uart.c`, `stm32l4xx_hal_gpio.c`
- `stm32l4xx_hal_rcc.c`, `stm32l4xx_hal_rcc_ex.c`
- `stm32l4xx_hal_cortex.c`, `stm32l4xx_hal_pwr.c`, `stm32l4xx_hal_pwr_ex.c`
- `stm32l4xx_hal_flash.c`, `stm32l4xx_hal_flash_ex.c`, `stm32l4xx_hal_flash_ramfunc.c`
- `startup_stm32l476rgtx.s`, `system_stm32l4xx.c`
- `syscalls.c`, `sysmem.c`
- linker script: `STM32L476RGTX_FLASH.ld`

## Wyjście na terminal (115200 baud, USART2)

```
=== GPS NEO-6M + STM32L476RG ===
Czekam na sygnał GPS...

----------------------------------------
  Status:   AKTYWNY  |  Fix: GPS
  Czas UTC: 13:45:22  Data: 20.03.2026
  Szerokość:  52.237890 N
  Długość:    21.017453 E
  Wysokość:   112.3 m n.p.m.
  Prędkość:   2.1 km/h  Kurs: 45.0 stopni
  Satelity:   8  HDOP: 1.2
  Zdania OK/ERR: 156 / 0
```

## Obsługiwane zdania NMEA

| Zdanie | Dane |
|--------|------|
| `$GPRMC` / `$GNRMC` | czas, data, pozycja, prędkość, kurs, status |
| `$GPGGA` / `$GNGGA` | pozycja, wysokość, jakość fixa, satelity, HDOP |

Pozostałe (GPGSV, GPGSA, GPVTG) ignorowane.

## Uwagi

- Czas do pierwszego fixa: ~30s na zewnątrz (cold start), w środku kilka minut
- `sentences_err` powinno być 0 – błędy sumy kontrolnej = problem z przewodem
- NEO-6M domyślnie wysyła dane co 1 sekundę przy 9600 baud
