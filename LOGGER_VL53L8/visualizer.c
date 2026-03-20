/*
 * visualizer.c  –  VL53L8CX Live Heatmap Visualizer  (ncursesw TUI)
 *
 * Format danych z STM32:
 *   --- Pomiar #N ---
 *   Odleglosc [mm]:
 *     100  200  300 ... (8 wartości × 8 wierszy)
 *   Sygnal:
 *      50   60   70 ... (8 wartości × 8 wierszy)
 *
 * Kompilacja:
 *   sudo apt install libncurses-dev
 *   gcc -O2 -o visualizer visualizer.c -lncursesw -lpthread -lm
 *
 * Użycie:
 *   ./visualizer                  →  /dev/ttyACM0 @ 115200
 *   ./visualizer /dev/ttyUSB0
 *   ./visualizer --demo           →  symulacja bez sprzętu
 *
 * Klawisze:  q – wyjście
 */

#define _GNU_SOURCE
#include <locale.h>
#include <ncurses.h>
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

/* ── Konfiguracja ─────────────────────────────────────────────── */
#define DEFAULT_PORT   "/dev/ttyACM0"
#define DIST_MAX       2000.0f   /* mm – maksymalna odległość (obcięcie) */
#define HISTORY        80        /* historia średniej odległości (sparkline) */

/* ── Pary kolorów ─────────────────────────────────────────────── */
#define CP_STATUS  1
#define CP_TITLE   2
#define CP_CONN    3
#define CP_DISC    4
#define CP_SPARK   5
#define CP_STATS   6
/* 10..25: 16 poziomów heatmapy odległości                        */
/* 26..41: 16 poziomów heatmapy sygnału                           */
#define CP_DIST(i)  (10 + (i))
#define CP_SIG(i)   (26 + (i))

/* ── Sparkline ────────────────────────────────────────────────── */
typedef struct {
    float data[HISTORY];
    int   head;
    float mn, mx;
} Ring;

static void ring_push(Ring *r, float v) {
    r->data[r->head] = v;
    r->head = (r->head + 1) % HISTORY;
    r->mn = r->data[0]; r->mx = r->data[0];
    for (int i = 1; i < HISTORY; i++) {
        if (r->data[i] < r->mn) r->mn = r->data[i];
        if (r->data[i] > r->mx) r->mx = r->data[i];
    }
}
static float ring_get(const Ring *r, int i) {
    return r->data[(r->head + i) % HISTORY];
}

static const char *SPKS[8] = {"▁","▂","▃","▄","▅","▆","▇","█"};

static void draw_spark(int y, int x, int w, const Ring *r) {
    float mn = r->mn, rng = r->mx - r->mn;
    if (rng < 1.f) rng = 1.f;
    int start = (HISTORY > w) ? HISTORY - w : 0;
    int drawn = 0;
    for (int i = start; i < HISTORY && drawn < w; i++, drawn++) {
        int lvl = (int)(7.f * (ring_get(r, i) - mn) / rng);
        if (lvl < 0) lvl = 0; if (lvl > 7) lvl = 7;
        mvaddstr(y, x + drawn, SPKS[lvl]);
    }
}

/* ── Stan współdzielony ───────────────────────────────────────── */
typedef struct {
    pthread_mutex_t lock;
    float dist[8][8];
    float sig[8][8];
    float sig_max;           /* dynamiczny maks sygnału dla skalowania */
    int   meas_num;
    float hz;
    int   connected;
    /* statystyki */
    float dist_min, dist_max_val, dist_avg;
    float sig_min_val, sig_max_val, sig_avg;
} Shared;

static Shared g = { .lock = PTHREAD_MUTEX_INITIALIZER };
static Ring   avg_dist_hist;

/* ── Aktualizacja statystyk ───────────────────────────────────── */
static void update_stats(float dist[8][8], float sig[8][8])
{
    float dmn = dist[0][0], dmx = dist[0][0], dsum = 0;
    float smn = sig[0][0],  smx = sig[0][0],  ssum = 0;
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++) {
            float d = dist[i][j], s = sig[i][j];
            if (d < dmn) dmn = d; if (d > dmx) dmx = d; dsum += d;
            if (s < smn) smn = s; if (s > smx) smx = s; ssum += s;
        }
    pthread_mutex_lock(&g.lock);
    memcpy(g.dist, dist, sizeof(g.dist));
    memcpy(g.sig,  sig,  sizeof(g.sig));
    g.dist_min    = dmn; g.dist_max_val = dmx; g.dist_avg = dsum / 64.f;
    g.sig_min_val = smn; g.sig_max_val  = smx; g.sig_avg  = ssum / 64.f;
    if (smx > g.sig_max) g.sig_max = smx;     /* max sygnału – rosnący */
    ring_push(&avg_dist_hist, dsum / 64.f);
    pthread_mutex_unlock(&g.lock);
}

/* ── Wątek UART / demo ────────────────────────────────────────── */
typedef struct { const char *port; int demo; } TArgs;

/* Parser linii – maszyna stanów */
typedef enum { ST_IDLE, ST_DIST, ST_SIG } ParseState;

static void *uart_thread(void *a_)
{
    TArgs *a = (TArgs *)a_;

    /* ── DEMO ─────────────────────────────────────────────────── */
    if (a->demo) {
        pthread_mutex_lock(&g.lock); g.connected = 1; pthread_mutex_unlock(&g.lock);
        double t = 0.0;
        int num = 0;
        while (1) {
            float dist[8][8], sig[8][8];
            /* Symulacja: kula zbliżająca się i oddalająca */
            float base_d = 600.f + 400.f * sinf((float)t * 0.4f);
            float base_s = 3000.f + 2000.f * sinf((float)t * 0.3f + 1.f);
            for (int r = 0; r < 8; r++)
                for (int c = 0; c < 8; c++) {
                    float dr = r - 3.5f, dc = c - 3.5f;
                    float d = sqrtf(dr*dr + dc*dc);
                    float noise = 20.f * sinf((float)t * 3.f + d * 1.3f);
                    dist[r][c] = base_d + d * 80.f + noise;
                    if (dist[r][c] < 0)        dist[r][c] = 0;
                    if (dist[r][c] > DIST_MAX)  dist[r][c] = DIST_MAX;
                    sig[r][c]  = base_s - d * 300.f + noise * 5.f;
                    if (sig[r][c] < 0) sig[r][c] = 0;
                }
            update_stats(dist, sig);
            pthread_mutex_lock(&g.lock);
            g.meas_num = ++num;
            g.hz = 3.0f;
            pthread_mutex_unlock(&g.lock);
            t += 1.0 / 3.0;
            struct timespec ts = { 0, 333333333L }; nanosleep(&ts, NULL);
        }
    }

    /* ── Sprzęt ───────────────────────────────────────────────── */
    int fd = -1;
    char line[256]; int pos = 0;
    ParseState pst = ST_IDLE;
    float dist[8][8], sig[8][8];
    int dist_row = 0, sig_row = 0;
    int meas_num = 0;
    static struct timeval t0; static int wc = 0;

    while (1) {
        if (fd < 0) {
            fd = open(a->port, O_RDONLY | O_NOCTTY);
            if (fd < 0) { sleep(1); continue; }
            struct termios tty = {0};
            tcgetattr(fd, &tty);
            cfsetispeed(&tty, B115200); cfsetospeed(&tty, B115200);
            tty.c_cflag = CS8 | CREAD | CLOCAL;
            tty.c_iflag = IGNPAR; tty.c_oflag = 0; tty.c_lflag = 0;
            tty.c_cc[VMIN] = 1; tty.c_cc[VTIME] = 0;
            tcsetattr(fd, TCSANOW, &tty);
            pthread_mutex_lock(&g.lock); g.connected = 1; pthread_mutex_unlock(&g.lock);
            pos = 0; pst = ST_IDLE;
        }

        char c;
        if (read(fd, &c, 1) != 1) {
            close(fd); fd = -1;
            pthread_mutex_lock(&g.lock); g.connected = 0; pthread_mutex_unlock(&g.lock);
            sleep(1); continue;
        }

        if (c == '\n') {
            line[pos] = '\0'; pos = 0;

            /* Nagłówek pomiaru */
            int n;
            if (sscanf(line, "--- Pomiar #%d ---", &n) == 1) {
                meas_num = n; pst = ST_IDLE;
                dist_row = 0; sig_row = 0;
                wc++;
                struct timeval now; gettimeofday(&now, NULL);
                if (!t0.tv_sec) t0 = now;
                double el = (now.tv_sec-t0.tv_sec)+(now.tv_usec-t0.tv_usec)/1e6;
                if (el >= 1.0) {
                    pthread_mutex_lock(&g.lock);
                    g.hz = (float)(wc / el);
                    pthread_mutex_unlock(&g.lock);
                    wc = 0; t0 = now;
                }
                continue;
            }

            /* Sekcja odległości */
            if (strstr(line, "Odleglosc")) { pst = ST_DIST; dist_row = 0; continue; }
            if (strstr(line, "Sygnal"))    { pst = ST_SIG;  sig_row  = 0; continue; }

            /* Parsowanie wierszy siatki 8×8 */
            if (pst == ST_DIST && dist_row < 8) {
                int vals[8], n_vals = 0;
                char *tok = strtok(line, " \t\r");
                while (tok && n_vals < 8) {
                    vals[n_vals++] = atoi(tok);
                    tok = strtok(NULL, " \t\r");
                }
                if (n_vals == 8) {
                    for (int j = 0; j < 8; j++)
                        dist[dist_row][j] = (float)vals[j];
                    dist_row++;
                }
            } else if (pst == ST_SIG && sig_row < 8) {
                int vals[8], n_vals = 0;
                char *tok = strtok(line, " \t\r");
                while (tok && n_vals < 8) {
                    vals[n_vals++] = atoi(tok);
                    tok = strtok(NULL, " \t\r");
                }
                if (n_vals == 8) {
                    for (int j = 0; j < 8; j++)
                        sig[sig_row][j] = (float)vals[j];
                    sig_row++;
                    if (sig_row == 8) {
                        /* Kompletny pomiar – zapisz */
                        update_stats(dist, sig);
                        pthread_mutex_lock(&g.lock);
                        g.meas_num = meas_num;
                        pthread_mutex_unlock(&g.lock);
                        pst = ST_IDLE;
                    }
                }
            }
        } else if (pos < 255) {
            line[pos++] = c;
        }
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════
 *  Inicjalizacja palet kolorów
 * ═══════════════════════════════════════════════════════════════ */
static void init_color_pairs(void)
{
    if (COLORS >= 256) {
        /* ── Odległość: zielony(bliski) → żółty → czerwony(daleki) ─ */
        static const short d_bg[16] = {
             46,  46,  82, 118, 154, 190, 226, 226,
            220, 214, 208, 202, 196, 160, 124,  88
        };
        static const short d_fg[16] = {
            0,0, 0,0, 0,0, 0,0,
            0,0, 0,7, 7,7, 7,7
        };
        for (int i = 0; i < 16; i++)
            init_pair(CP_DIST(i), d_fg[i], d_bg[i]);

        /* ── Sygnał: ciemny błękit → fiolet → pomarańcz → biały ─── */
        static const short s_bg[16] = {
             17,  18,  19,  20,  56,  57,  93, 129,
            165, 201, 205, 214, 220, 226, 227, 231
        };
        static const short s_fg[16] = {
            7,7,7,7, 7,7,7,7,
            7,7,0,0, 0,0,0,0
        };
        for (int i = 0; i < 16; i++)
            init_pair(CP_SIG(i), s_fg[i], s_bg[i]);
    } else {
        /* ── 8-kolorowy fallback ──────────────────────────────────── */
        static const short d8_bg[8] = {
            COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW,
            COLOR_RED,   COLOR_RED,   COLOR_MAGENTA, COLOR_BLACK
        };
        static const short d8_fg[8] = {
            COLOR_BLACK, COLOR_BLACK, COLOR_BLACK, COLOR_BLACK,
            COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE
        };
        for (int i = 0; i < 8; i++) {
            init_pair(CP_DIST(i),   d8_fg[i], d8_bg[i]);
            init_pair(CP_DIST(i+8), d8_fg[i], d8_bg[i]);   /* duplikuj */
        }
        static const short s8_bg[8] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,  COLOR_CYAN,
            COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE
        };
        static const short s8_fg[8] = {
            COLOR_WHITE, COLOR_WHITE, COLOR_BLACK, COLOR_BLACK,
            COLOR_BLACK, COLOR_BLACK, COLOR_BLACK, COLOR_BLACK
        };
        for (int i = 0; i < 8; i++) {
            init_pair(CP_SIG(i),   s8_fg[i], s8_bg[i]);
            init_pair(CP_SIG(i+8), s8_fg[i], s8_bg[i]);
        }
    }
}

/* Indeks palety (0..15) na podstawie wartości */
static int dist_level(float mm) {
    int l = (int)(mm / DIST_MAX * 15.f);
    if (l < 0) l = 0; if (l > 15) l = 15;
    return l;
}
static int sig_level(float s, float smax) {
    if (smax <= 0) return 0;
    int l = (int)(s / smax * 15.f);
    if (l < 0) l = 0; if (l > 15) l = 15;
    return l;
}

/* ── Prymitywy ramki (identyczne jak w LAB01) ─────────────────── */
static void hstr(int y, int x, const char *c, int n) {
    for (int i = 0; i < n; i++) mvaddstr(y, x+i, c);
}
static void vbar(int y0, int y1, int x) {
    for (int y = y0; y <= y1; y++) mvaddstr(y, x, "║");
}
static void draw_box(int y, int x, int h, int w) {
    mvaddstr(y,   x,       "╔"); hstr(y,   x+1, "═", w-2); mvaddstr(y,   x+w-1, "╗");
    mvaddstr(y+h-1,x,      "╚"); hstr(y+h-1,x+1,"═", w-2); mvaddstr(y+h-1,x+w-1,"╝");
    vbar(y+1, y+h-2, x); vbar(y+1, y+h-2, x+w-1);
}
static void inner_hline(int y, int x, int w, const int *tj, int ntj) {
    mvaddstr(y, x, "╠");
    for (int i = 1; i < w-1; i++) {
        int jt = 0;
        for (int j = 0; j < ntj; j++) if (x+i == tj[j]) { jt=1; break; }
        mvaddstr(y, x+i, jt ? "╬" : "═");
    }
    mvaddstr(y, x+w-1, "╣");
}
static void inner_hline_b(int y, int x, int w, const int *tj, int ntj) {
    /* ╠═╩═╣ – dolny T-junction */
    mvaddstr(y, x, "╠");
    for (int i = 1; i < w-1; i++) {
        int jt = 0;
        for (int j = 0; j < ntj; j++) if (x+i == tj[j]) { jt=1; break; }
        mvaddstr(y, x+i, jt ? "╩" : "═");
    }
    mvaddstr(y, x+w-1, "╣");
}

static void centered(int y, int x, int w, const char *s, int attr) {
    int len = (int)strlen(s);
    int pad = (w - len) / 2; if (pad < 0) pad = 0;
    attron(attr);
    mvprintw(y, x+pad, "%.*s", w, s);
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════
 *  Główny render
 * ═══════════════════════════════════════════════════════════════ */
static void render(const char *port, int demo)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    if (rows < 20 || cols < 80) {
        erase();
        mvaddstr(1, 2, "Okno za małe! Minimum 80×20.");
        refresh(); return;
    }

    /* ── Snapshot ─────────────────────────────────────────────── */
    pthread_mutex_lock(&g.lock);
    float dist[8][8], sig[8][8];
    memcpy(dist, g.dist, sizeof(dist));
    memcpy(sig,  g.sig,  sizeof(sig));
    float smax   = g.sig_max;
    int   num    = g.meas_num;
    float hz     = g.hz;
    int   conn   = g.connected;
    float dmn    = g.dist_min, dmx = g.dist_max_val, davg = g.dist_avg;
    float smn    = g.sig_min_val, smx = g.sig_max_val, savg = g.sig_avg;
    Ring  hist   = avg_dist_hist;
    pthread_mutex_unlock(&g.lock);

    erase();

    /* ── Pasek statusu ────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_STATUS) | A_BOLD);
    mvhline(0, 0, ' ', cols);
    mvprintw(0, 1, " VL53L8CX  │  %-14s │  %.1f Hz  │  Pomiar #%d",
             demo ? "DEMO" : port, hz, num);
    attroff(COLOR_PAIR(CP_STATUS) | A_BOLD);
    const char *cs = conn ? "● POŁĄCZONO" : "○ BRAK";
    attron(COLOR_PAIR(conn ? CP_CONN : CP_DISC) | A_BOLD);
    mvprintw(0, cols - (int)strlen(cs) - 2, "%s ", cs);
    attroff(A_BOLD);
    attroff(COLOR_PAIR(conn ? CP_CONN : CP_DISC));

    /* ── Geometria ────────────────────────────────────────────── */
    int bx=0, by=1, bw=cols, bh=rows-1;
    /* Szerokość komórki siatki: adaptacyjna */
    int cell_w = (cols >= 100) ? 6 : 5;
    int grid_w = 8 * cell_w;           /* np. 48 lub 40 */
    /* Podział: lewa połowa dist, prawa sig */
    int x2 = bx + grid_w + 3;         /* x separatora wewnętrznego */
    /* Jeśli grids się nie mieszczą – wycentruj */
    int total_grids = grid_w * 2 + 6;
    if (total_grids > cols) x2 = cols / 2;

    draw_box(by, bx, bh, bw);
    vbar(by+1, by+bh-2, x2);

    /* ── Nagłówki ─────────────────────────────────────────────── */
    int tj[1] = { x2 };
    attron(COLOR_PAIR(CP_TITLE) | A_BOLD);
    centered(by+1, bx+1,    x2-bx-1,  " ODLEGŁOŚĆ [mm]",    0);
    centered(by+1, x2+1,    bw-x2-2,  " SYGNAŁ",            0);
    attroff(COLOR_PAIR(CP_TITLE) | A_BOLD);
    inner_hline(by+2, bx, bw, tj, 1);

    /* Nagłówki kolumn siatki (0..7) */
    int gr = by+3;
    attron(A_DIM);
    for (int c = 0; c < 8; c++)
        mvprintw(gr, bx+1 + c*cell_w, "%*d", cell_w, c);
    for (int c = 0; c < 8; c++)
        mvprintw(gr, x2+1 + c*cell_w, "%*d", cell_w, c);
    attroff(A_DIM);
    gr++;

    /* ── Siatki heatmapy ──────────────────────────────────────── */
    for (int row = 0; row < 8; row++) {
        if (gr + row >= by + bh - 1) break;

        /* Numer wiersza */
        attron(A_DIM);
        /* mvprintw(gr+row, bx+1, "%d", row); */
        attroff(A_DIM);

        for (int col = 0; col < 8; col++) {
            /* ── Odległość ── */
            int dl = dist_level(dist[row][col]);
            attron(COLOR_PAIR(CP_DIST(dl)));
            mvprintw(gr+row, bx+1 + col*cell_w, "%*d", cell_w, (int)dist[row][col]);
            attroff(COLOR_PAIR(CP_DIST(dl)));

            /* ── Sygnał ── */
            int sl = sig_level(sig[row][col], smax > 0 ? smax : 1.f);
            attron(COLOR_PAIR(CP_SIG(sl)));
            mvprintw(gr+row, x2+1 + col*cell_w, "%*d", cell_w, (int)sig[row][col]);
            attroff(COLOR_PAIR(CP_SIG(sl)));
        }
    }
    int after_grid = gr + 8;

    /* ── Separator + statystyki ───────────────────────────────── */
    if (after_grid < by + bh - 3) {
        inner_hline(after_grid, bx, bw, tj, 1);
        int sr = after_grid + 1;
        attron(COLOR_PAIR(CP_STATS));
        mvprintw(sr, bx+2,  " Min:%4d  Max:%4d  Śr:%4d mm ",
                 (int)dmn, (int)dmx, (int)davg);
        mvprintw(sr, x2+2,  " Min:%5d  Max:%5d  Śr:%5d  ",
                 (int)smn, (int)smx, (int)savg);
        attroff(COLOR_PAIR(CP_STATS));
        after_grid = sr + 1;
    }

    /* ── Sparkline historii średniej odległości ───────────────── */
    if (after_grid + 2 < by + bh - 1) {
        inner_hline_b(after_grid, bx, bw, tj, 1);
        int sr = after_grid + 1;
        int sw = bw - 20;
        attron(COLOR_PAIR(CP_SPARK));
        mvaddstr(sr, bx+2, " Śr.dist ");
        draw_spark(sr, bx+11, sw > 0 ? sw : 10, &hist);
        attroff(COLOR_PAIR(CP_SPARK));
        after_grid = sr + 1;
    }

    /* ── Legenda kolorów (odległość) ──────────────────────────── */
    if (after_grid < by + bh - 1) {
        /* Pasek gradientu – zielony→żółty→czerwony */
        mvaddstr(by+bh-1, bx+2, " 0mm ");
        int lbar_x = bx+7, lbar_w = (cols/2 - 14);
        if (lbar_w > 0) {
            for (int i = 0; i < lbar_w; i++) {
                int l = (int)(i * 15.f / lbar_w);
                attron(COLOR_PAIR(CP_DIST(l)));
                mvaddch(by+bh-1, lbar_x+i, ' ');
                attroff(COLOR_PAIR(CP_DIST(l)));
            }
        }
        mvprintw(by+bh-1, lbar_x+lbar_w+1, " %dmm", (int)DIST_MAX);
        /* Prawa: skala sygnału */
        int rx = x2+2;
        mvaddstr(by+bh-1, rx, " sygnał: ");
        int sbar_x = rx+9, sbar_w = (bw-x2-14);
        if (sbar_w > 0) {
            for (int i = 0; i < sbar_w; i++) {
                int l = (int)(i * 15.f / sbar_w);
                attron(COLOR_PAIR(CP_SIG(l)));
                mvaddch(by+bh-1, sbar_x+i, ' ');
                attroff(COLOR_PAIR(CP_SIG(l)));
            }
        }
        attron(A_DIM);
        mvaddstr(by+bh-1, bx+2, " 0mm");
        mvprintw(by+bh-1, lbar_x+lbar_w+1, " %dmm  [q] wyjście", (int)DIST_MAX);
        attroff(A_DIM);
    }

    refresh();
}

/* ── main ─────────────────────────────────────────────────────── */
int main(int argc, char **argv)
{
    const char *port = DEFAULT_PORT;
    int demo = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--demo")) demo = 1;
        else                            port = argv[i];
    }

    memset(&g, 0, sizeof(g));
    pthread_mutex_init(&g.lock, NULL);

    TArgs targs = { port, demo };
    pthread_t tid;
    pthread_create(&tid, NULL, uart_thread, &targs);
    pthread_detach(tid);

    setlocale(LC_ALL, "");
    initscr(); cbreak(); noecho();
    nodelay(stdscr, TRUE); curs_set(0); keypad(stdscr, TRUE);

    if (has_colors()) {
        start_color(); use_default_colors();
        init_pair(CP_STATUS, COLOR_BLACK, COLOR_CYAN);
        init_pair(CP_TITLE,  COLOR_WHITE, -1);
        init_pair(CP_CONN,   COLOR_GREEN, COLOR_CYAN);
        init_pair(CP_DISC,   COLOR_RED,   COLOR_CYAN);
        init_pair(CP_SPARK,  COLOR_CYAN,  -1);
        init_pair(CP_STATS,  COLOR_WHITE, -1);
        init_color_pairs();
    }

    const struct timespec frame = { 0, 100000000L };   /* 10 fps (ToF = 3 Hz) */
    while (1) {
        int ch = getch();
        if (ch == 'q' || ch == 'Q') break;
        render(port, demo);
        nanosleep(&frame, NULL);
    }

    endwin();
    return 0;
}
