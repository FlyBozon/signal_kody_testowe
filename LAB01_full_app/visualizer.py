#!/usr/bin/env python3
"""
STM32 NUCLEO-L476RG + NanoEdgeAI - Live Orientation Visualizer
================================================================
Odczytuje dane IMU przez UART i wyświetla w czasie rzeczywistym:
  - Wizualizacja 3D orientacji płytki (sześcian)
  - Wykresy czasowe: akcelerometr, żyroskop, kąty
  - Odczyt kątów Roll / Pitch / Yaw

Dane wejściowe (format z STM32 PrintBuffer):
  -- Sample N --
    Ax = <mg>
    Ay = <mg>
    Az = <mg>
    Gx = <dps>
    Gy = <dps>
    Gz = <dps>

Filtr orientacji: komplementarny (gyro 98% + accel 2%)
Yaw: tylko integracja żyroskopu (brak magnetometru → dryfuje)

Użycie:
    python3 visualizer.py                         # /dev/ttyACM0, 115200
    python3 visualizer.py --port /dev/ttyUSB0
    python3 visualizer.py --demo                  # tryb demo bez sprzętu

Wymagania:
    pip install pyserial numpy matplotlib
"""

import argparse
import collections
import math
import threading
import time
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------------------
# Konfiguracja
# ---------------------------------------------------------------------------
DEFAULT_PORT   = "/dev/ttyACM0"
DEFAULT_BAUD   = 115200
SAMPLE_RATE_HZ = 30.0          # Musi zgadzać się z ODR w STM32
ALPHA          = 0.98           # Waga filtra komplementarnego (gyro vs accel)
HISTORY        = 300            # Liczba próbek w wykresach czasowych

# ---------------------------------------------------------------------------
# Współdzielony stan (chroni lock)
# ---------------------------------------------------------------------------
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        empty = lambda: collections.deque([0.0] * HISTORY, maxlen=HISTORY)
        self.ax = empty(); self.ay = empty(); self.az = empty()
        self.gx = empty(); self.gy = empty(); self.gz = empty()
        self.rolls  = empty()
        self.pitches = empty()
        self.yaws   = empty()
        self.connected = False
        self.sample_count = 0

state = State()

# ---------------------------------------------------------------------------
# Filtr komplementarny
# ---------------------------------------------------------------------------
class ComplementaryFilter:
    def __init__(self, alpha=ALPHA, dt=1.0 / SAMPLE_RATE_HZ):
        self.alpha = alpha
        self.dt    = dt
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

    def update(self, ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps):
        # mg → g
        ax = ax_mg / 1000.0
        ay = ay_mg / 1000.0
        az = az_mg / 1000.0

        # Kąty z akcelerometru (stabilne, ale szumne)
        accel_roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        # Filtr: łączy integrację żyroskopu z korekcją akcelerometru
        self.roll  = self.alpha * (self.roll  + gx_dps * self.dt) \
                   + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy_dps * self.dt) \
                   + (1 - self.alpha) * accel_pitch
        self.yaw  += gz_dps * self.dt   # brak korekty – dryfuje z czasem

        return self.roll, self.pitch, self.yaw

filt = ComplementaryFilter()

# ---------------------------------------------------------------------------
# Macierz obrotu ZYX (yaw→pitch→roll)
# ---------------------------------------------------------------------------
def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    Rz = np.array([[ math.cos(y), -math.sin(y), 0],
                   [ math.sin(y),  math.cos(y), 0],
                   [ 0,            0,            1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [ 0,           1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,            0           ],
                   [0, math.cos(r), -math.sin(r) ],
                   [0, math.sin(r),  math.cos(r) ]])
    return Rz @ Ry @ Rx

# ---------------------------------------------------------------------------
# Geometria sześcianu (proporcje zbliżone do płytki NUCLEO)
# ---------------------------------------------------------------------------
W, H, D = 1.2, 0.8, 0.15   # szerokość, wysokość, grubość

_v = np.array([
    [-W, -H, -D], [ W, -H, -D], [ W,  H, -D], [-W,  H, -D],
    [-W, -H,  D], [ W, -H,  D], [ W,  H,  D], [-W,  H,  D],
])
CUBE_FACES  = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[0,3,7,4],[1,2,6,5]]
FACE_COLORS = ["#1565C0","#0D47A1","#1976D2","#1E88E5","#42A5F5","#90CAF9"]

def rotated_cube(roll, pitch, yaw):
    R = rotation_matrix(roll, pitch, yaw)
    return _v @ R.T

# ---------------------------------------------------------------------------
# Wątek odczytu UART
# ---------------------------------------------------------------------------
def serial_reader(port, baud):
    import serial

    while True:
        try:
            ser = serial.Serial(port, baud, timeout=2)
            print(f"[UART] Połączono: {port} @ {baud} baud")
            with state.lock:
                state.connected = True
            break
        except Exception as e:
            print(f"[UART] Oczekiwanie na port {port}... ({e})")
            time.sleep(2)

    sample = {}
    KEYS = {"Ax", "Ay", "Az", "Gx", "Gy", "Gz"}

    while True:
        try:
            raw = ser.readline()
            line = raw.decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        if line.startswith("-- Sample"):
            sample = {}
        elif "=" in line:
            try:
                key, val = line.split("=", 1)
                key = key.strip()
                val = int(val.strip())
                if key in KEYS:
                    sample[key] = val
            except ValueError:
                pass

            if KEYS <= sample.keys():
                roll, pitch, yaw = filt.update(
                    sample["Ax"], sample["Ay"], sample["Az"],
                    sample["Gx"], sample["Gy"], sample["Gz"],
                )
                with state.lock:
                    state.roll  = roll
                    state.pitch = pitch
                    state.yaw   = yaw
                    state.ax.append(sample["Ax"])
                    state.ay.append(sample["Ay"])
                    state.az.append(sample["Az"])
                    state.gx.append(sample["Gx"])
                    state.gy.append(sample["Gy"])
                    state.gz.append(sample["Gz"])
                    state.rolls.append(roll)
                    state.pitches.append(pitch)
                    state.yaws.append(yaw)
                    state.sample_count += 1
                sample = {}

# ---------------------------------------------------------------------------
# Wątek symulatora (tryb demo bez sprzętu)
# ---------------------------------------------------------------------------
def demo_reader():
    print("[DEMO] Tryb demonstracyjny – symulowane dane IMU")
    t = 0.0
    dt = 1.0 / SAMPLE_RATE_HZ
    with state.lock:
        state.connected = True

    while True:
        ax_mg = int(math.sin(t * 0.3) * 500)
        ay_mg = int(math.cos(t * 0.2) * 300)
        az_mg = int(980 + math.sin(t * 0.1) * 50)
        gx_dps = math.sin(t * 0.5) * 20
        gy_dps = math.cos(t * 0.4) * 15
        gz_dps = math.sin(t * 0.15) * 10

        roll, pitch, yaw = filt.update(ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps)

        with state.lock:
            state.roll  = roll
            state.pitch = pitch
            state.yaw   = yaw
            state.ax.append(ax_mg); state.ay.append(ay_mg); state.az.append(az_mg)
            state.gx.append(gx_dps); state.gy.append(gy_dps); state.gz.append(gz_dps)
            state.rolls.append(roll)
            state.pitches.append(pitch)
            state.yaws.append(yaw)
            state.sample_count += 1

        t += dt
        time.sleep(dt)

# ---------------------------------------------------------------------------
# Styl wykresów
# ---------------------------------------------------------------------------
BG_DARK  = "#1a1a2e"
BG_PANEL = "#16213e"
BG_PLOT  = "#0f3460"
C_TEXT   = "#e0e0e0"
C_GRID   = "#1f4068"

def style_axes(ax, title=""):
    ax.set_facecolor(BG_PLOT)
    ax.tick_params(colors=C_TEXT, labelsize=7)
    ax.set_title(title, color=C_TEXT, fontsize=8, pad=4)
    ax.grid(True, color=C_GRID, linewidth=0.5, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_edgecolor("#1f4068")

# ---------------------------------------------------------------------------
# Animacja
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="STM32 IMU Visualizer")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help=f"Port szeregowy (domyślnie: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"Prędkość transmisji (domyślnie: {DEFAULT_BAUD})")
    parser.add_argument("--demo", action="store_true",
                        help="Tryb demo – symulowane dane bez podłączonego sprzętu")
    args = parser.parse_args()

    # Uruchom wątek źródła danych
    target = demo_reader if args.demo else lambda: serial_reader(args.port, args.baud)
    threading.Thread(target=target, daemon=True).start()

    # --- Layout ----------------------------------------------------------------
    fig = plt.figure(figsize=(15, 8), facecolor=BG_DARK)
    fig.suptitle("STM32 NUCLEO-L476RG  |  NanoEdgeAI IMU Visualizer",
                 color=C_TEXT, fontsize=12, fontweight="bold", y=0.98)

    gs = fig.add_gridspec(3, 3, left=0.04, right=0.97, top=0.93, bottom=0.06,
                          hspace=0.45, wspace=0.35)

    ax3d    = fig.add_subplot(gs[:, 0], projection="3d")   # cała lewa kolumna
    ax_info = fig.add_subplot(gs[0, 1])                     # środek górny
    ax_acc  = fig.add_subplot(gs[0, 2])                     # prawy górny
    ax_gyro = fig.add_subplot(gs[1, 2])                     # prawy środkowy
    ax_ang  = fig.add_subplot(gs[2, 2])                     # prawy dolny
    ax_rpy  = fig.add_subplot(gs[1:, 1])                    # środek dolny (większy)

    # ---- Panel info (środek górny) ------------------------------------------
    ax_info.set_facecolor(BG_PANEL)
    ax_info.axis("off")
    info_lines = [
        ("Port:",    lambda: args.port if not args.demo else "DEMO"),
        ("Baud:",    lambda: str(args.baud)),
        ("ODR:",     lambda: f"{SAMPLE_RATE_HZ:.0f} Hz"),
        ("Filtr:",   lambda: f"Komplementarny α={ALPHA}"),
        ("Próbki:",  lambda: str(state.sample_count)),
    ]
    info_texts = []
    for idx, (label, _) in enumerate(info_lines):
        y = 0.82 - idx * 0.18
        ax_info.text(0.05, y, label, transform=ax_info.transAxes,
                     color="#90CAF9", fontsize=8, va="center", fontfamily="monospace")
        t = ax_info.text(0.45, y, "–", transform=ax_info.transAxes,
                         color=C_TEXT, fontsize=8, va="center", fontfamily="monospace")
        info_texts.append((t, info_lines[idx][1]))
    ax_info.set_title("Info", color=C_TEXT, fontsize=8, pad=4)

    # ---- Panel Roll/Pitch/Yaw (środek dolny) --------------------------------
    ax_rpy.set_facecolor(BG_PANEL)
    ax_rpy.axis("off")
    ax_rpy.set_title("Orientacja", color=C_TEXT, fontsize=9, pad=6)

    def angle_text(ax, y, label, color):
        ax.text(0.08, y + 0.07, label, transform=ax.transAxes,
                color=color, fontsize=10, va="center", fontfamily="monospace")
        t = ax.text(0.08, y - 0.05, "0.0°", transform=ax.transAxes,
                    color=color, fontsize=26, va="center", fontfamily="monospace",
                    fontweight="bold")
        return t

    roll_t  = angle_text(ax_rpy, 0.75, "Roll",  "#FF7043")
    pitch_t = angle_text(ax_rpy, 0.42, "Pitch", "#66BB6A")
    yaw_t   = angle_text(ax_rpy, 0.09, "Yaw",   "#42A5F5")

    # Wskaźniki poziomicy
    bar_ax = [ax_rpy.text(0.60, y, "●", transform=ax_rpy.transAxes,
                           fontsize=14, va="center", color=c)
              for y, c in [(0.75, "#FF7043"), (0.42, "#66BB6A"), (0.09, "#42A5F5")]]

    # Styl wykresów czasowych
    style_axes(ax_acc,  "Akcelerometr [mg]")
    style_axes(ax_gyro, "Żyroskop [dps]")
    style_axes(ax_ang,  "Kąty orientacji [°]")

    x_ax = np.arange(HISTORY)

    # Linie wykresów (inicjalizacja)
    def make_lines(ax, labels, colors):
        return [ax.plot(x_ax, [0]*HISTORY, lw=1, color=c, label=l)[0]
                for l, c in zip(labels, colors)]

    lines_acc  = make_lines(ax_acc,  ["Ax","Ay","Az"], ["#FF7043","#66BB6A","#42A5F5"])
    lines_gyro = make_lines(ax_gyro, ["Gx","Gy","Gz"], ["#FF7043","#66BB6A","#42A5F5"])
    lines_ang  = make_lines(ax_ang,  ["Roll","Pitch","Yaw"], ["#FF7043","#66BB6A","#42A5F5"])

    for ax in [ax_acc, ax_gyro, ax_ang]:
        ax.legend(fontsize=6, loc="upper right",
                  facecolor=BG_DARK, labelcolor=C_TEXT, framealpha=0.7)

    # 3D – wstępna konfiguracja
    ax3d.set_facecolor(BG_PLOT)
    ax3d.set_title("3D orientacja płytki", color=C_TEXT, fontsize=9, pad=2)
    ax3d.tick_params(colors=C_TEXT, labelsize=5)

    def update(_frame):
        with state.lock:
            roll  = state.roll
            pitch = state.pitch
            yaw   = state.yaw
            d_ax  = list(state.ax);  d_ay = list(state.ay);  d_az = list(state.az)
            d_gx  = list(state.gx);  d_gy = list(state.gy);  d_gz = list(state.gz)
            d_r   = list(state.rolls)
            d_p   = list(state.pitches)
            d_y   = list(state.yaws)
            cnt   = state.sample_count

        # ---- 3D sześcian ----------------------------------------------------
        ax3d.cla()
        ax3d.set_facecolor(BG_PLOT)
        verts = rotated_cube(roll, pitch, yaw)
        faces = [[verts[i] for i in f] for f in CUBE_FACES]
        ax3d.add_collection3d(
            Poly3DCollection(faces, facecolors=FACE_COLORS,
                             edgecolors="#ffffff22", linewidths=0.4, alpha=0.85)
        )
        # Osie układu współrzędnych
        R = rotation_matrix(roll, pitch, yaw)
        for vec, col, lbl in [([1.6,0,0],"#FF5252","X"),
                               ([0,1.6,0],"#69F0AE","Y"),
                               ([0,0,1.6],"#40C4FF","Z")]:
            rv = R @ np.array(vec)
            ax3d.quiver(0,0,0, rv[0],rv[1],rv[2],
                        color=col, linewidth=1.8, arrow_length_ratio=0.15)
            ax3d.text(rv[0]*1.05, rv[1]*1.05, rv[2]*1.05, lbl,
                      color=col, fontsize=8, fontweight="bold")

        lim = 1.8
        ax3d.set_xlim(-lim, lim); ax3d.set_ylim(-lim, lim); ax3d.set_zlim(-lim, lim)
        ax3d.set_xlabel("X", color=C_TEXT, fontsize=7)
        ax3d.set_ylabel("Y", color=C_TEXT, fontsize=7)
        ax3d.set_zlabel("Z", color=C_TEXT, fontsize=7)
        ax3d.tick_params(colors="#666", labelsize=5)
        ax3d.set_title("3D orientacja płytki", color=C_TEXT, fontsize=9, pad=2)

        # ---- Kąty (tekst) ---------------------------------------------------
        roll_t.set_text(f"{roll:+.1f}°")
        pitch_t.set_text(f"{pitch:+.1f}°")
        yaw_t.set_text(f"{yaw:+.1f}°")

        # Prosty wskaźnik – obrót kropki wokół osi
        for bar, angle in zip(bar_ax, [roll, pitch, yaw]):
            a = math.radians(angle)
            x = 0.60 + 0.12 * math.sin(a)
            bar.set_position((x, bar.get_position()[1]))

        # ---- Info panel -----------------------------------------------------
        for txt, fn in info_texts:
            txt.set_text(fn())
        info_texts[4][0].set_text(str(cnt))   # próbki (aktualizacja)

        # ---- Wykresy czasowe ------------------------------------------------
        data_sets = [
            (lines_acc,  [d_ax, d_ay, d_az]),
            (lines_gyro, [d_gx, d_gy, d_gz]),
            (lines_ang,  [d_r,  d_p,  d_y]),
        ]
        axes_list = [ax_acc, ax_gyro, ax_ang]
        titles    = ["Akcelerometr [mg]", "Żyroskop [dps]", "Kąty orientacji [°]"]

        for lines, datasets, ax, title in zip(data_sets, [None]*3, axes_list, titles):
            lines_set, data = lines[0], lines[1]
            for line, d in zip(lines_set, data):
                line.set_ydata(d)
            vals = [v for d in data for v in d]
            if vals:
                mn, mx = min(vals), max(vals)
                margin = max(abs(mx - mn) * 0.1, 1.0)
                ax.set_ylim(mn - margin, mx + margin)
            ax.set_xlim(0, HISTORY)
            style_axes(ax, title)
            ax.legend(fontsize=6, loc="upper right",
                      facecolor=BG_DARK, labelcolor=C_TEXT, framealpha=0.7)
            for line, d in zip(lines_set, data):
                ax.add_line(line)

        return []

    ani = animation.FuncAnimation(
        fig, update, interval=50, blit=False, cache_frame_data=False
    )

    plt.show()


if __name__ == "__main__":
    main()
