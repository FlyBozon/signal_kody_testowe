/*
 * visualizer.c – TUI dla COMBINED: IKS4A1 (LSM6DSV16X) + VL53L8CX
 *
 * Format wejściowy UART:
 *   MEMS Ax=123 Ay=-45 Az=980 Gx=0 Gy=1 Gz=-2
 *   TDIST 500 520 ... (64 wartości mm)
 *   TSIG  3000 3200 ... (64 wartości kcps)
 *   NEAI sim=95 ok=1
 *
 * Kompilacja:
 *   gcc -O2 -o visualizer visualizer.c -lncursesw -lpthread -lm
 *
 * Uruchomienie:
 *   TERM=xterm-256color ./visualizer [--port /dev/ttyACM0] [--baud 115200] [--demo]
 *
 * Layout (min 80×24):
 *   Lewy panel  (22 znaki): AKCELEROMETR + ŻYROSKOP + NEAI
 *   Prawy panel (55 znaków): mapa 8×8 odległości + mapa 8×8 sygnału
 */

#define _GNU_SOURCE
#include <ncursesw/ncurses.h>
#include <stdarg.h>
#include <locale.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

/* =========================================================================
 * Konfiguracja
 * =========================================================================*/
#define DEFAULT_PORT  "/dev/ttyACM0"
#define DEFAULT_BAUD  115200
#define HISTORY       60
#define TOF_N         64
#define TOF_ROWS      8
#define TOF_COLS      8
#define FPS           15
#define NEAI_NONE     (-1)

/* Layout (terminal min 80×24) */
#define LEFT_W      22          /* szerokość treści lewego panelu */
#define DIV_COL     (LEFT_W+1)  /* col 23 – pionowy separator */
#define RSTART      (DIV_COL+1) /* col 24 – start prawego panelu */
#define TOTAL_W     80

/* =========================================================================
 * Pary kolorów
 * =========================================================================*/
#define CP_STATUS   1
#define CP_TITLE    2
#define CP_LABEL    3
#define CP_VALUE    4
#define CP_CONN     5
#define CP_DISC     6
#define CP_OK       7
#define CP_ANO      8
#define CP_SPARK    9
#define CP_POS     10
#define CP_NEG     11
/* 20–35: dist heatmap; 36–51: sig heatmap */

/* =========================================================================
 * Unicode
 * =========================================================================*/
static const char *SPARKS[8] = {"▁","▂","▃","▄","▅","▆","▇","█"};
#define BFULL  "█"
#define BEMPTY "░"

/* =========================================================================
 * Gradienty 256-kolorów
 * =========================================================================*/
static const short D_BG[16] = {
     46,  46,  82, 118, 154, 190, 226, 226,
    220, 214, 208, 202, 196, 160, 124,  88
};
static const short D_FG[16] = {
     16,  16,  16,  16,  16,  16,  16,  16,
     16,  16,  16,  16, 231, 231, 231, 231
};
static const short S_BG[16] = {
     17,  18,  19,  20,  56,  57,  93, 129,
    165, 201, 205, 214, 220, 226, 227, 231
};
static const short S_FG[16] = {
    231, 231, 231, 231, 231, 231, 231, 231,
    231, 231,  16,  16,  16,  16,  16,  16
};

/* =========================================================================
 * Bufor kołowy
 * =========================================================================*/
typedef struct { float b[HISTORY]; int ptr, cnt; } Ring;

static void ring_push(Ring *r, float v)
{
    r->b[r->ptr] = v;
    r->ptr = (r->ptr+1) % HISTORY;
    if (r->cnt < HISTORY) r->cnt++;
}
static float ring_get(const Ring *r, int age)
{
    int i = ((r->ptr-1-age) % HISTORY + HISTORY) % HISTORY;
    return r->b[i];
}

/* =========================================================================
 * Dane współdzielone
 * =========================================================================*/
typedef struct {
    float ax,ay,az;         /* mg  */
    float gx,gy,gz;         /* dps */
    Ring  rax,ray,raz;
    Ring  rgx,rgy,rgz;

    int   dist[TOF_N];
    int   sig[TOF_N];
    long  tof_num;
    int   dmin,dmax,davg;
    int   smin,smax,savg;
    Ring  rdavg;

    int   neai_sim;          /* 0-100 lub NEAI_NONE */
    int   neai_ok;

    int   connected;
    pthread_mutex_t mtx;
} Shared;

static void shared_init(Shared *s)
{
    memset(s, 0, sizeof(*s));
    s->neai_sim = NEAI_NONE;
    for (int i=0;i<TOF_N;i++) s->dist[i]=1000;
    pthread_mutex_init(&s->mtx, NULL);
}

static void tof_stats(Shared *s)
{
    int dn=9999,dx=0,ds=0, sn=9999,sx=0,ss=0;
    for (int i=0;i<TOF_N;i++) {
        if(s->dist[i]<dn) dn=s->dist[i];
        if(s->dist[i]>dx) dx=s->dist[i];
        ds+=s->dist[i];
        if(s->sig[i]<sn) sn=s->sig[i];
        if(s->sig[i]>sx) sx=s->sig[i];
        ss+=s->sig[i];
    }
    s->dmin=dn; s->dmax=dx; s->davg=ds/TOF_N;
    s->smin=sn; s->smax=sx; s->savg=ss/TOF_N;
}

/* =========================================================================
 * Wątek UART
 * =========================================================================*/
static int open_serial(const char *port, int baud)
{
    int fd = open(port, O_RDWR|O_NOCTTY);
    if (fd<0) return -1;
    struct termios t;
    tcgetattr(fd,&t);
    speed_t sp = B115200;
    if (baud==9600)   sp=B9600;
    if (baud==57600)  sp=B57600;
    if (baud==230400) sp=B230400;
    cfsetispeed(&t,sp); cfsetospeed(&t,sp);
    cfmakeraw(&t);
    t.c_cc[VMIN]=1; t.c_cc[VTIME]=0;
    tcsetattr(fd,TCSANOW,&t);
    return fd;
}

typedef struct { const char *port; int baud, demo; Shared *sh; } TArgs;

static void *uart_thread(void *arg)
{
    TArgs *ta = arg;
    Shared *s = ta->sh;
    int fd = open_serial(ta->port, ta->baud);
    if (fd<0) return NULL;
    pthread_mutex_lock(&s->mtx); s->connected=1; pthread_mutex_unlock(&s->mtx);

    char line[512]; int lp=0;
    while (1) {
        char c; if(read(fd,&c,1)<=0){usleep(1000);continue;}
        if (c=='\r'||c=='\n') {
            if (lp==0) continue;
            line[lp]='\0'; lp=0;
            pthread_mutex_lock(&s->mtx);
            if (!strncmp(line,"MEMS ",5)) {
                int ax,ay,az,gx,gy,gz;
                if (sscanf(line,"MEMS Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d",
                           &ax,&ay,&az,&gx,&gy,&gz)==6) {
                    s->ax=ax;s->ay=ay;s->az=az;
                    s->gx=gx;s->gy=gy;s->gz=gz;
                    ring_push(&s->rax,ax); ring_push(&s->ray,ay); ring_push(&s->raz,az);
                    ring_push(&s->rgx,gx); ring_push(&s->rgy,gy); ring_push(&s->rgz,gz);
                }
            } else if (!strncmp(line,"TDIST ",6)) {
                char *p=line+6;
                for(int i=0;i<TOF_N;i++){while(*p==' ')p++;s->dist[i]=atoi(p);while(*p&&*p!=' ')p++;}
                s->tof_num++; tof_stats(s); ring_push(&s->rdavg,(float)s->davg);
            } else if (!strncmp(line,"TSIG ",5)) {
                char *p=line+5;
                for(int i=0;i<TOF_N;i++){while(*p==' ')p++;s->sig[i]=atoi(p);while(*p&&*p!=' ')p++;}
            } else if (!strncmp(line,"NEAI ",5)) {
                int sim=0,ok=0;
                sscanf(line,"NEAI sim=%d ok=%d",&sim,&ok);
                s->neai_sim=sim; s->neai_ok=ok;
            }
            pthread_mutex_unlock(&s->mtx);
        } else { if(lp<(int)sizeof(line)-1) line[lp++]=c; }
    }
    return NULL;
}

/* =========================================================================
 * Wątek DEMO
 * =========================================================================*/
static void *demo_thread(void *arg)
{
    TArgs *ta = arg;
    Shared *s = ta->sh;
    pthread_mutex_lock(&s->mtx); s->connected=1; pthread_mutex_unlock(&s->mtx);
    double t=0.0;
    while (1) {
        struct timespec ts={0,333333333L}; nanosleep(&ts,NULL); t+=0.333;
        pthread_mutex_lock(&s->mtx);
        s->ax=(float)(1000*sin(t*1.1)); s->ay=(float)(500*cos(t*0.7));
        s->az=(float)(980+50*sin(t*2)); s->gx=(float)(30*sin(t*1.5));
        s->gy=(float)(20*cos(t*1.2));   s->gz=(float)(10*sin(t*0.9));
        ring_push(&s->rax,s->ax); ring_push(&s->ray,s->ay); ring_push(&s->raz,s->az);
        ring_push(&s->rgx,s->gx); ring_push(&s->rgy,s->gy); ring_push(&s->rgz,s->gz);
        double cx=3.5+3*sin(t*0.4), cy=3.5+3*cos(t*0.4);
        for(int r=0;r<8;r++) for(int c=0;c<8;c++) {
            double d2=(r-cy)*(r-cy)+(c-cx)*(c-cx);
            s->dist[r*8+c]=(int)(400+1200*d2/32.0); if(s->dist[r*8+c]>2000)s->dist[r*8+c]=2000;
            s->sig[r*8+c]=(int)(5000-3500*d2/32.0);  if(s->sig[r*8+c]<100) s->sig[r*8+c]=100;
        }
        s->tof_num++; tof_stats(s); ring_push(&s->rdavg,(float)s->davg);
        s->neai_sim=(int)(85+15*sin(t*0.3)); s->neai_ok=(s->neai_sim>=90)?1:0;
        pthread_mutex_unlock(&s->mtx);
    }
    return NULL;
}

/* =========================================================================
 * Rysowanie
 * =========================================================================*/
static void mprint(int row, int col, const char *fmt, ...)
{
    char buf[256]; va_list ap; va_start(ap,fmt);
    vsnprintf(buf,sizeof(buf),fmt,ap); va_end(ap);
    mvaddstr(row,col,buf);
}

/* Ramka + wewnętrzny pionowy separator */
static void draw_frame(int rows)
{
    mvaddstr(1,0,"╔");
    for(int c=1;c<=LEFT_W;c++) mvaddstr(1,c,"═");
    mvaddstr(1,DIV_COL,"╦");
    for(int c=RSTART;c<TOTAL_W-1;c++) mvaddstr(1,c,"═");
    mvaddstr(1,TOTAL_W-1,"╗");
    for(int r=2;r<rows-1;r++){
        mvaddstr(r,0,"║"); mvaddstr(r,DIV_COL,"║"); mvaddstr(r,TOTAL_W-1,"║");
    }
    mvaddstr(rows-1,0,"╚");
    for(int c=1;c<=LEFT_W;c++) mvaddstr(rows-1,c,"═");
    mvaddstr(rows-1,DIV_COL,"╩");
    for(int c=RSTART;c<TOTAL_W-1;c++) mvaddstr(rows-1,c,"═");
    mvaddstr(rows-1,TOTAL_W-1,"╝");
}

/* Separator w lewym panelu */
static void lsep(int r)
{
    mvaddstr(r,0,"╠");
    for(int c=1;c<=LEFT_W;c++) mvaddstr(r,c,"═");
    mvaddstr(r,DIV_COL,"╣");
}

/* Separator w prawym panelu z opcjonalnym tytułem */
static void rsep(int r, const char *title)
{
    mvaddstr(r,DIV_COL,"╠");
    int c=RSTART;
    if (title) { char b[64]; snprintf(b,sizeof(b)," %s ",title); mvaddstr(r,c,b); c+=strlen(b); }
    for(;c<TOTAL_W-1;c++) mvaddstr(r,c,"═");
    mvaddstr(r,TOTAL_W-1,"╣");
}

/* Pasek bipolarny */
static void draw_bar(int row, int col, float val, float range, int width)
{
    int center=width/2;
    int filled=(int)roundf(fabsf(val)/range*(float)center);
    if(filled>center) filled=center;
    int pos=(val>=0);
    move(row,col); addch('[');
    for(int i=0;i<width;i++){
        int in_fill = pos ? (i>=center && i<center+filled)
                         : (i>=center-filled && i<center);
        if(i==center){ attron(A_BOLD); addch('|'); attroff(A_BOLD); }
        else if(in_fill){
            attron(COLOR_PAIR(pos?CP_POS:CP_NEG)|A_BOLD);
            addstr(BFULL);
            attroff(COLOR_PAIR(pos?CP_POS:CP_NEG)|A_BOLD);
        } else { attron(A_DIM); addstr(BEMPTY); attroff(A_DIM); }
    }
    addch(']');
}

/* Sparkline */
static void sparkline(int row, int col, const Ring *r, int width)
{
    if(r->cnt==0) return;
    float mn=r->b[0],mx=r->b[0];
    for(int i=0;i<r->cnt;i++){ if(r->b[i]<mn)mn=r->b[i]; if(r->b[i]>mx)mx=r->b[i]; }
    float rng=mx-mn; if(rng<1e-3f) rng=1e-3f;
    attron(COLOR_PAIR(CP_SPARK));
    for(int i=0;i<width;i++){
        int age=width-1-i;
        if(age>=r->cnt){ mvaddch(row,col+i,' '); continue; }
        int idx=(int)((ring_get(r,age)-mn)/rng*7.0f);
        if(idx<0)idx=0; if(idx>7)idx=7;
        mvaddstr(row,col+i,SPARKS[idx]);
    }
    attroff(COLOR_PAIR(CP_SPARK));
}

/* Jeden wiersz heatmapy */
static void hmap_row(int row, int col, const int *data, int n, int vmax,
                     const short *bg, const short *fg)
{
    for(int c=0;c<n;c++){
        int v=data[c];
        int idx=(int)((float)v/(float)(vmax>0?vmax:1)*15.0f);
        if(idx<0)idx=0; if(idx>15)idx=15;
        attron(COLOR_PAIR(20+(int)(bg==D_BG?0:16)+idx));
        mprint(row, col+c*5, " %4d", v);
        attroff(COLOR_PAIR(20+(int)(bg==D_BG?0:16)+idx));
    }
}

/* =========================================================================
 * Renderowanie
 * =========================================================================*/
static void render(Shared *s, int demo)
{
    int rows,cols; getmaxyx(stdscr,rows,cols); (void)cols;

    /* Kopiuj dane pod lockiem */
    pthread_mutex_lock(&s->mtx);
    float ax=s->ax,ay=s->ay,az=s->az,gx=s->gx,gy=s->gy,gz=s->gz;
    Ring rax=s->rax,ray=s->ray,raz=s->raz,rgx=s->rgx,rgy=s->rgy,rgz=s->rgz;
    int dist[TOF_N],sig[TOF_N];
    memcpy(dist,s->dist,sizeof dist); memcpy(sig,s->sig,sizeof sig);
    long  tnum=s->tof_num;
    int   dmin=s->dmin,dmax=s->dmax,davg=s->davg;
    int   smin=s->smin,smax=s->smax,savg=s->savg;
    Ring  rdavg=s->rdavg;
    int   nsim=s->neai_sim, nok=s->neai_ok, conn=s->connected;
    pthread_mutex_unlock(&s->mtx);

    erase();

    /* ── Status bar ─────────────────────────────────────────────── */
    attron(COLOR_PAIR(CP_STATUS)|A_BOLD);
    for(int c=0;c<TOTAL_W;c++) mvaddch(0,c,' ');
    const char *mode = (nsim==NEAI_NONE) ? "LOG" : "INFERENCE";
    mprint(0,1," IKS4A1 + VL53L8CX │ %s │ %s",demo?"DEMO":"LIVE",mode);
    if(conn){ attron(COLOR_PAIR(CP_CONN)); mprint(0,TOTAL_W-13," ● POŁĄCZONO  "); }
    else    { attron(COLOR_PAIR(CP_DISC)); mprint(0,TOTAL_W-13," ○ BRAK POŁĄCZ"); }
    attroff(COLOR_PAIR(CP_STATUS)|COLOR_PAIR(CP_CONN)|COLOR_PAIR(CP_DISC)|A_BOLD);

    draw_frame(rows);

    /* ── Lewy panel ─────────────────────────────────────────────── */

    /* AKCELEROMETR */
    attron(COLOR_PAIR(CP_TITLE)|A_BOLD);
    mprint(2,1," AKCELEROMETR [mg]    ");
    attroff(COLOR_PAIR(CP_TITLE)|A_BOLD);

    /* wiersz: etykieta + wartość + pasek; wiersz+1: sparkline */
    struct { float val; Ring *r; float range; int vrow; } axes6[6]={
        {ax,&rax,2000,3},{ay,&ray,2000,5},{az,&raz,2000,7},
        {gx,&rgx, 250,11},{gy,&rgy, 250,13},{gz,&rgz, 250,15}
    };
    const char *lbl6[6]={"Ax","Ay","Az","Gx","Gy","Gz"};
    for(int i=0;i<6;i++){
        int vr=axes6[i].vrow;
        attron(COLOR_PAIR(CP_LABEL));  mprint(vr,1,"%s:",lbl6[i]); attroff(COLOR_PAIR(CP_LABEL));
        attron(COLOR_PAIR(CP_VALUE)|A_BOLD); mprint(vr,4,"%+7.0f",(double)axes6[i].val); attroff(COLOR_PAIR(CP_VALUE)|A_BOLD);
        draw_bar(vr,12,axes6[i].val,axes6[i].range,9);
        sparkline(vr+1,1,axes6[i].r,LEFT_W);
        if(i==2) lsep(9);   /* separator ACCEL|GYRO */
        if(i==2){ attron(COLOR_PAIR(CP_TITLE)|A_BOLD); mprint(10,1," ŻYROSKOP [dps]       "); attroff(COLOR_PAIR(CP_TITLE)|A_BOLD); }
    }

    /* Separator GYRO|NEAI */
    lsep(17);

    /* NEAI */
    if(nsim==NEAI_NONE){
        attron(A_DIM); mprint(18,1," NEAI: oczekiwanie... "); attroff(A_DIM);
        mprint(19,1,"                      ");
    } else {
        attron(COLOR_PAIR(CP_LABEL));  mprint(18,1,"NEAI:"); attroff(COLOR_PAIR(CP_LABEL));
        attron(COLOR_PAIR(CP_VALUE)|A_BOLD); mprint(18,7,"%3d%%",nsim); attroff(COLOR_PAIR(CP_VALUE)|A_BOLD);
        attron(COLOR_PAIR(nok?CP_OK:CP_ANO)|A_BOLD);
        mprint(18,12, nok?" ✓ OK      ":" ✗ ANOMALIA");
        attroff(COLOR_PAIR(CP_OK)|COLOR_PAIR(CP_ANO)|A_BOLD);
        /* pasek */
        int bw=LEFT_W-2, filled=nsim*bw/100;
        move(19,1);
        attron(COLOR_PAIR(nok?CP_OK:CP_ANO)|A_BOLD);
        for(int i=0;i<filled;i++)  addstr(BFULL);
        attroff(COLOR_PAIR(CP_OK)|COLOR_PAIR(CP_ANO)|A_BOLD);
        attron(A_DIM); for(int i=filled;i<bw;i++) addstr(BEMPTY); attroff(A_DIM);
    }

    /* ToF podsumowanie */
    attron(COLOR_PAIR(CP_LABEL)); mprint(20,1,"ToF #:"); attroff(COLOR_PAIR(CP_LABEL));
    attron(COLOR_PAIR(CP_VALUE)|A_BOLD); mprint(20,8,"%-8ld",tnum); attroff(COLOR_PAIR(CP_VALUE)|A_BOLD);
    attron(COLOR_PAIR(CP_LABEL)); mprint(21,1,"Dist śr:"); attroff(COLOR_PAIR(CP_LABEL));
    attron(COLOR_PAIR(CP_VALUE)|A_BOLD); mprint(21,10,"%4d mm",davg); attroff(COLOR_PAIR(CP_VALUE)|A_BOLD);
    attron(COLOR_PAIR(CP_LABEL)); mprint(22,1,"Sig  śr:"); attroff(COLOR_PAIR(CP_LABEL));
    attron(COLOR_PAIR(CP_VALUE)|A_BOLD); mprint(22,10,"%6d",savg); attroff(COLOR_PAIR(CP_VALUE)|A_BOLD);

    /* ── Prawy panel ─────────────────────────────────────────────── */

    /* ODLEGŁOŚĆ */
    attron(COLOR_PAIR(CP_TITLE)|A_BOLD); mprint(2,RSTART," ODLEGŁOŚĆ [mm]"); attroff(COLOR_PAIR(CP_TITLE)|A_BOLD);
    attron(A_DIM); mprint(2,RSTART+16,"  min=%4d max=%4d śr=%4d",dmin,dmax,davg); attroff(A_DIM);

    /* Nagłówek kolumn dist */
    attron(A_DIM);
    mprint(3,RSTART,"   ");
    for(int c=0;c<8;c++) mprint(3,RSTART+3+c*5,"    %d",c);
    attroff(A_DIM);

    /* Dane dist */
    for(int r=0;r<8;r++){
        attron(A_DIM); mprint(4+r,RSTART,"r%d ",r); attroff(A_DIM);
        for(int c=0;c<8;c++){
            int v=dist[r*8+c];
            int idx=(int)((float)v/2000.0f*15.0f); if(idx<0)idx=0; if(idx>15)idx=15;
            attron(COLOR_PAIR(20+idx));
            mprint(4+r,RSTART+3+c*5," %4d",v);
            attroff(COLOR_PAIR(20+idx));
        }
    }

    /* Separator DIST|SIG */
    rsep(12,"SYGNAŁ");
    attron(A_DIM);
    mprint(12,RSTART+8,"  min=%4d max=%4d śr=%4d",smin,smax,savg);
    attroff(A_DIM);

    /* Nagłówek kolumn sig */
    attron(A_DIM);
    mprint(13,RSTART,"   ");
    for(int c=0;c<8;c++) mprint(13,RSTART+3+c*5,"    %d",c);
    attroff(A_DIM);

    /* Dane sig */
    int smx=(smax>0)?smax:1;
    for(int r=0;r<8;r++){
        attron(A_DIM); mprint(14+r,RSTART,"r%d ",r); attroff(A_DIM);
        for(int c=0;c<8;c++){
            int v=sig[r*8+c];
            int idx=(int)((float)v/(float)smx*15.0f); if(idx<0)idx=0; if(idx>15)idx=15;
            attron(COLOR_PAIR(36+idx));
            mprint(14+r,RSTART+3+c*5," %4d",v);
            attroff(COLOR_PAIR(36+idx));
        }
    }

    /* Sparkline śr. odległości */
    attron(COLOR_PAIR(CP_LABEL)|A_DIM); mprint(22,RSTART,"śr.dist "); attroff(COLOR_PAIR(CP_LABEL)|A_DIM);
    sparkline(22,RSTART+8,&rdavg,TOTAL_W-RSTART-9);

    /* ── Stopka z legendą ────────────────────────────────────────── */
    attron(A_DIM); mprint(rows-1,2,"0mm"); attroff(A_DIM);
    for(int i=0;i<16;i++){
        attron(COLOR_PAIR(20+i)); mvaddstr(rows-1,6+i*2,"  "); attroff(COLOR_PAIR(20+i));
    }
    attron(A_DIM); mprint(rows-1,6+32,"2000mm "); attroff(A_DIM);
    for(int i=0;i<16;i++){
        attron(COLOR_PAIR(36+i)); mvaddstr(rows-1,6+32+7+i*2,"  "); attroff(COLOR_PAIR(36+i));
    }
    attron(A_DIM); mprint(rows-1,TOTAL_W-10,"[q]wyjście"); attroff(A_DIM);

    refresh();
}

/* =========================================================================
 * Inicjalizacja kolorów
 * =========================================================================*/
static void init_colors(void)
{
    start_color(); use_default_colors();
    init_pair(CP_STATUS, COLOR_BLACK,  COLOR_CYAN);
    init_pair(CP_TITLE,  COLOR_WHITE,  -1);
    init_pair(CP_LABEL,  COLOR_CYAN,   -1);
    init_pair(CP_VALUE,  COLOR_WHITE,  -1);
    init_pair(CP_CONN,   COLOR_GREEN,  COLOR_CYAN);
    init_pair(CP_DISC,   COLOR_RED,    COLOR_CYAN);
    init_pair(CP_OK,     COLOR_GREEN,  -1);
    init_pair(CP_ANO,    COLOR_RED,    -1);
    init_pair(CP_SPARK,  COLOR_CYAN,   -1);
    init_pair(CP_POS,    COLOR_GREEN,  -1);
    init_pair(CP_NEG,    COLOR_RED,    -1);

    if (COLORS >= 256) {
        for(int i=0;i<16;i++) init_pair(20+i, D_FG[i], D_BG[i]);
        for(int i=0;i<16;i++) init_pair(36+i, S_FG[i], S_BG[i]);
    } else {
        /* Fallback 8 kolorów */
        short df[16]={COLOR_GREEN,COLOR_GREEN,COLOR_GREEN,COLOR_GREEN,
                      COLOR_GREEN,COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,
                      COLOR_YELLOW,COLOR_RED,COLOR_RED,COLOR_RED,
                      COLOR_RED,COLOR_RED,COLOR_RED,COLOR_RED};
        short sf[16]={COLOR_BLUE,COLOR_BLUE,COLOR_BLUE,COLOR_BLUE,
                      COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA,COLOR_MAGENTA,
                      COLOR_CYAN,COLOR_CYAN,COLOR_CYAN,COLOR_CYAN,
                      COLOR_YELLOW,COLOR_YELLOW,COLOR_YELLOW,COLOR_WHITE};
        for(int i=0;i<16;i++){ init_pair(20+i,COLOR_BLACK,df[i]); init_pair(36+i,COLOR_BLACK,sf[i]); }
    }
}

/* =========================================================================
 * main()
 * =========================================================================*/
int main(int argc, char *argv[])
{
    const char *port = DEFAULT_PORT;
    int baud = DEFAULT_BAUD, demo = 0;
    for(int i=1;i<argc;i++){
        if(!strcmp(argv[i],"--port")&&i+1<argc) port=argv[++i];
        else if(!strcmp(argv[i],"--baud")&&i+1<argc) baud=atoi(argv[++i]);
        else if(!strcmp(argv[i],"--demo")) demo=1;
    }

    setlocale(LC_ALL,"");
    initscr(); cbreak(); noecho(); curs_set(0);
    keypad(stdscr,TRUE); timeout(1000/FPS);
    init_colors();

    Shared sh; shared_init(&sh);
    TArgs ta={.port=port,.baud=baud,.demo=demo,.sh=&sh};
    pthread_t tid;
    pthread_create(&tid,NULL,demo?demo_thread:uart_thread,&ta);

    while(1){
        int ch=getch();
        if(ch=='q'||ch=='Q') break;
        render(&sh,demo);
    }
    endwin();
    return 0;
}
