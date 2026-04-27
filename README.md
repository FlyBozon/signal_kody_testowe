# signal_kody_testowe

## kod do testów przed hackathonem SignAI

do odpalenia kodu potrzebujecie zainstalować kompilator (instrukcja się znajduje na tym repo https://github.com/MrQlek/meteo-station)

potem wejść do odpowiedniego foldreru
`make`
lub `make clean & make`

by załadować program do stmki odpalcie `make flash`

### visualizer
#### LAB01
```
# Instalacja raz
sudo apt install libncurses-dev

# Kompilacja
cd LAB01_full_app
gcc -O2 -o visualizer visualizer.c -lncursesw -lpthread -lm

# Uruchomienie
./visualizer              # STM32 na /dev/ttyACM1
./visualizer /dev/ttyACM0
./visualizer --demo       # bez sprzętu – podgląd interfejsu
```

#### LOGGER
```cd LOGGER_VL53L8
gcc -O2 -o visualizer_bin visualizer.c -lncursesw -lpthread -lm
TERM=xterm-256color ./visualizer_bin --port /dev/ttyACM0
```
