import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import threading
import queue
import re
import argparse
import sys

PORT = "/dev/ttyACM0"
BAUD = 115200

# ---------------------------------------------------------------------------
# Parser stanu — czyta linie z UART i wyciąga pomiary
# ---------------------------------------------------------------------------

STATE_IDLE = 0
STATE_DIST = 1
STATE_SIG  = 2

def serial_reader(port, baud, q):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[BŁĄD] Nie można otworzyć portu {port}: {e}")
        sys.exit(1)

    print(f"[INFO] Połączono z {port} @ {baud} baud")

    state      = STATE_IDLE
    dist_rows  = []
    sig_rows   = []
    pomiar_num = 0

    while True:
        try:
            raw = ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if not line:
            continue

        m = re.match(r"---\s*Pomiar\s*#(\d+)\s*---", line)
        if m:
            pomiar_num = int(m.group(1))
            dist_rows  = []
            sig_rows   = []
            state      = STATE_IDLE
            continue

        if line == "Odleglosc [mm]:":
            state = STATE_DIST
            continue

        if line == "Sygnal:":
            state = STATE_SIG
            continue

        if state == STATE_DIST:
            try:
                vals = list(map(int, line.split()))
            except ValueError:
                continue
            if len(vals) == 8:
                dist_rows.append(vals)
                if len(dist_rows) == 8:
                    state = STATE_IDLE

        elif state == STATE_SIG:
            try:
                vals = list(map(int, line.split()))
            except ValueError:
                continue
            if len(vals) == 8:
                sig_rows.append(vals)
                if len(sig_rows) == 8:
                    q.put((pomiar_num,
                           np.array(dist_rows, dtype=float),
                           np.array(sig_rows,  dtype=float)))
                    state = STATE_IDLE

# ---------------------------------------------------------------------------
# Wykres
# ---------------------------------------------------------------------------

def make_text_color(val, vmin, vmax):
    """Biały tekst na ciemnym tle, czarny na jasnym."""
    norm = (val - vmin) / (vmax - vmin + 1e-9)
    return "white" if norm > 0.55 else "black"

def main():
    parser = argparse.ArgumentParser(description="VL53L8CX live heatmap")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--baud", type=int, default=BAUD)
    args = parser.parse_args()

    data_queue = queue.Queue()

    t = threading.Thread(target=serial_reader,
                         args=(args.port, args.baud, data_queue),
                         daemon=True)
    t.start()

    # --- layout ---
    fig = plt.figure(figsize=(13, 6))
    fig.patch.set_facecolor("#1e1e1e")

    ax_d = fig.add_subplot(1, 2, 1)
    ax_s = fig.add_subplot(1, 2, 2)

    for ax in (ax_d, ax_s):
        ax.set_facecolor("#1e1e1e")
        ax.tick_params(colors="gray")
        for spine in ax.spines.values():
            spine.set_edgecolor("gray")

    empty = np.zeros((8, 8))

    im_d = ax_d.imshow(empty, cmap="RdYlGn_r", vmin=0, vmax=2000, aspect="equal")
    im_s = ax_s.imshow(empty, cmap="plasma",    vmin=0, vmax=5000, aspect="equal")

    ax_d.set_title("Odległość [mm]", color="white", fontsize=12, pad=8)
    ax_s.set_title("Sygnał",         color="white", fontsize=12, pad=8)

    ax_d.set_xticks(range(8)); ax_d.set_yticks(range(8))
    ax_s.set_xticks(range(8)); ax_s.set_yticks(range(8))
    ax_d.set_xticklabels(range(8), color="gray", fontsize=8)
    ax_d.set_yticklabels(range(8), color="gray", fontsize=8)
    ax_s.set_xticklabels(range(8), color="gray", fontsize=8)
    ax_s.set_yticklabels(range(8), color="gray", fontsize=8)

    cb_d = fig.colorbar(im_d, ax=ax_d, fraction=0.046, pad=0.04)
    cb_s = fig.colorbar(im_s, ax=ax_s, fraction=0.046, pad=0.04)
    cb_d.ax.yaxis.set_tick_params(color="gray")
    cb_s.ax.yaxis.set_tick_params(color="gray")
    plt.setp(cb_d.ax.yaxis.get_ticklabels(), color="gray")
    plt.setp(cb_s.ax.yaxis.get_ticklabels(), color="gray")

    # wartości w komórkach
    texts_d = [[ax_d.text(j, i, "", ha="center", va="center", fontsize=7, fontweight="bold")
                for j in range(8)] for i in range(8)]
    texts_s = [[ax_s.text(j, i, "", ha="center", va="center", fontsize=7, fontweight="bold")
                for j in range(8)] for i in range(8)]

    title = fig.suptitle("VL53L8CX — oczekiwanie na dane...",
                          color="white", fontsize=14, y=0.98)
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # --- animacja ---
    def update(_frame):
        try:
            num, dist, sig = data_queue.get_nowait()
        except queue.Empty:
            return

        im_d.set_data(dist)
        im_d.set_clim(0, 2000)

        sig_max = max(sig.max(), 1)
        im_s.set_data(sig)
        im_s.set_clim(0, sig_max)

        for i in range(8):
            for j in range(8):
                d_val = int(dist[i, j])
                s_val = int(sig[i, j])
                texts_d[i][j].set_text(str(d_val))
                texts_d[i][j].set_color(make_text_color(d_val, 0, 2000))
                texts_s[i][j].set_text(str(s_val))
                texts_s[i][j].set_color(make_text_color(s_val, 0, sig_max))

        title.set_text(f"VL53L8CX  —  Pomiar #{num}")

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    main()
