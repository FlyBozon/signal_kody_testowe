/*
 * visualizer.c  –  STM32 IMU Live Visualizer  (ncursesw TUI)
 *
 * Format danych z STM32:
 *   "Ax=123 Ay=-45 Az=980 Gx=0 Gy=1 Gz=-2\r\n"
 *
 * Kompilacja:
 *   sudo apt install libncurses-dev
 *   gcc -O2 -o visualizer visualizer.c -lncursesw -lpthread -lm
 *
 * Użycie:
 *   ./visualizer                   →  /dev/ttyACM1 @ 115200
 *   ./visualizer /dev/ttyUSB0      →  własny port
 *   ./visualizer --demo            →  symulacja bez sprzętu
 *
 * Klawisze:  q – wyjście  │  r – reset Yaw
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
#define DEFAULT_PORT   "/dev/ttyACM1"
#define SAMPLE_HZ      30.0f
#define ALPHA          0.98f
#define HISTORY        200

/* ── Pary kolorów ─────────────────────────────────────────────── */
#define CP_STATUS   1   /* pasek statusu        */
#define CP_TITLE    2   /* nagłówki paneli       */
#define CP_RED      3   /* Ax / Gx / Roll        */
#define CP_GREEN    4   /* Ay / Gy / Pitch       */
#define CP_CYAN     5   /* Az / Gz / Yaw         */
#define CP_CONN     6   /* "POŁĄCZONO" (zielony) */
#define CP_DISC     7   /* "BRAK"      (czerwony)*/
#define CP_DIM      8   /* sparklines            */
#define CP_BOX      9   /* ramki                 */

/* ── Bufor kołowy ─────────────────────────────────────────────── */
typedef struct {
    float data[HISTORY];
    int   head;
    float mn, mx;
} Ring;

static inline void ring_push(Ring *r, float v) {
    r->data[r->head] = v;
    r->head = (r->head + 1) % HISTORY;
    r->mn = r->data[0]; r->mx = r->data[0];
    for (int i = 1; i < HISTORY; i++) {
        if (r->data[i] < r->mn) r->mn = r->data[i];
        if (r->data[i] > r->mx) r->mx = r->data[i];
    }
}
static inline float ring_get(const Ring *r, int i) {
    return r->data[(r->head + i) % HISTORY];
}

/* ── Stan współdzielony ───────────────────────────────────────── */
typedef struct {
    pthread_mutex_t lock;
    float ax, ay, az, gx, gy, gz;
    float roll, pitch, yaw;
    float hz;
    int   cnt, connected;
} Shared;
static Shared g = { .lock = PTHREAD_MUTEX_INITIALIZER };
static Ring h_ax, h_ay, h_az, h_gx, h_gy, h_gz, h_roll, h_pitch, h_yaw;

/* ── Filtr komplementarny ─────────────────────────────────────── */
static struct { float roll, pitch, yaw; } filt;

static void filter_update(float ax_mg, float ay_mg, float az_mg,
                           float gx, float gy, float gz)
{
    float dt = 1.0f / SAMPLE_HZ;
    float ax = ax_mg/1000.f, ay = ay_mg/1000.f, az = az_mg/1000.f;
    float ar = atan2f(ay, sqrtf(ax*ax+az*az)) * (180.f/(float)M_PI);
    float ap = atan2f(-ax, sqrtf(ay*ay+az*az)) * (180.f/(float)M_PI);
    filt.roll  = ALPHA*(filt.roll  + gx*dt) + (1-ALPHA)*ar;
    filt.pitch = ALPHA*(filt.pitch + gy*dt) + (1-ALPHA)*ap;
    filt.yaw  += gz*dt;
}

/* ── Wątek UART / demo ────────────────────────────────────────── */
typedef struct { const char *port; int demo; } TArgs;

static void store(float ax,float ay,float az,
                  float gx,float gy,float gz, float hz_v)
{
    filter_update(ax,ay,az,gx,gy,gz);
    pthread_mutex_lock(&g.lock);
    g.ax=ax; g.ay=ay; g.az=az;
    g.gx=gx; g.gy=gy; g.gz=gz;
    g.roll=filt.roll; g.pitch=filt.pitch; g.yaw=filt.yaw;
    if (hz_v > 0) g.hz = hz_v;
    g.cnt++;
    ring_push(&h_ax,ax); ring_push(&h_ay,ay); ring_push(&h_az,az);
    ring_push(&h_gx,gx); ring_push(&h_gy,gy); ring_push(&h_gz,gz);
    ring_push(&h_roll,filt.roll);
    ring_push(&h_pitch,filt.pitch);
    ring_push(&h_yaw,filt.yaw);
    pthread_mutex_unlock(&g.lock);
}

static void *uart_thread(void *a_) {
    TArgs *a = (TArgs*)a_;

    /* ── DEMO ── */
    if (a->demo) {
        double t=0, dt=1.0/SAMPLE_HZ;
        pthread_mutex_lock(&g.lock); g.connected=1; pthread_mutex_unlock(&g.lock);
        while (1) {
            store((float)(sin(t*.3)*500),(float)(cos(t*.2)*300),
                  (float)(980+sin(t*.1)*50),
                  (float)(sin(t*.5)*20),(float)(cos(t*.4)*15),
                  (float)(sin(t*.15)*10), SAMPLE_HZ);
            t+=dt;
            struct timespec ts={0,(long)(dt*1e9)}; nanosleep(&ts,NULL);
        }
    }

    /* ── Sprzęt ── */
    int fd=-1; char line[256]; int pos=0;
    while (1) {
        if (fd<0) {
            fd=open(a->port,O_RDONLY|O_NOCTTY);
            if (fd<0){sleep(1);continue;}
            struct termios tty={0};
            tcgetattr(fd,&tty);
            cfsetispeed(&tty,B115200); cfsetospeed(&tty,B115200);
            tty.c_cflag=CS8|CREAD|CLOCAL; tty.c_iflag=IGNPAR;
            tty.c_oflag=0; tty.c_lflag=0;
            tty.c_cc[VMIN]=1; tty.c_cc[VTIME]=0;
            tcsetattr(fd,TCSANOW,&tty);
            pthread_mutex_lock(&g.lock); g.connected=1;
            pthread_mutex_unlock(&g.lock); pos=0;
        }
        char c;
        if (read(fd,&c,1)!=1) {
            close(fd); fd=-1;
            pthread_mutex_lock(&g.lock); g.connected=0;
            pthread_mutex_unlock(&g.lock); sleep(1); continue;
        }
        if (c=='\n') {
            line[pos]='\0'; pos=0;
            float ax=0,ay=0,az=0,gx=0,gy=0,gz=0; int found=0;
            char tmp[256]; strncpy(tmp,line,255); tmp[255]='\0';
            char *tok=strtok(tmp," \t\r");
            while (tok) {
                char key[8]; float val;
                if (sscanf(tok,"%7[^=]=%f",key,&val)==2) {
                    if      (!strcmp(key,"Ax")){ax=val;found|=1;}
                    else if (!strcmp(key,"Ay")){ay=val;found|=2;}
                    else if (!strcmp(key,"Az")){az=val;found|=4;}
                    else if (!strcmp(key,"Gx")){gx=val;found|=8;}
                    else if (!strcmp(key,"Gy")){gy=val;found|=16;}
                    else if (!strcmp(key,"Gz")){gz=val;found|=32;}
                }
                tok=strtok(NULL," \t\r");
            }
            if (found==63) {
                static struct timeval t0; static int wc=0;
                wc++; struct timeval now; gettimeofday(&now,NULL);
                if (!t0.tv_sec) t0=now;
                double el=(now.tv_sec-t0.tv_sec)+(now.tv_usec-t0.tv_usec)/1e6;
                float hz_v=0;
                if (el>=1.0){hz_v=(float)(wc/el);wc=0;t0=now;}
                store(ax,ay,az,gx,gy,gz,hz_v);
            }
        } else if (pos<255) { line[pos++]=c; }
    }
    return NULL;
}

/* ═══════════════════════════════════════════════════════════════
 *  Prymitywy rysowania
 * ═══════════════════════════════════════════════════════════════ */

/* Pozioma linia ze znakiem c powtórzonym w razy */
static void hstr(int y, int x, const char *c, int n) {
    for (int i=0;i<n;i++) mvaddstr(y,x+i,c);
}

/* Pionowa linia "║" */
static void vbar(int y0, int y1, int x) {
    for (int y=y0;y<=y1;y++) mvaddstr(y,x,"║");
}

/* Pełna ramka podwójna */
static void draw_box(int y,int x,int h,int w) {
    mvaddstr(y,   x,       "╔"); hstr(y,   x+1,"═",w-2); mvaddstr(y,   x+w-1,"╗");
    mvaddstr(y+h-1,x,      "╚"); hstr(y+h-1,x+1,"═",w-2); mvaddstr(y+h-1,x+w-1,"╝");
    vbar(y+1,y+h-2,x); vbar(y+1,y+h-2,x+w-1);
}

/* Wewnętrzna pozioma linia ╠═╬═╣ z opcjonalnymi T-junction na pozycjach[] */
static void inner_hline(int y, int x, int w,
                         const int *tj, int ntj)  /* tj: kolumny z ╬ */
{
    mvaddstr(y,x,"╠");
    for (int i=1;i<w-1;i++) {
        int is_tj=0;
        for (int j=0;j<ntj;j++) if (x+i==tj[j]){is_tj=1;break;}
        mvaddstr(y,x+i, is_tj?"╬":"═");
    }
    mvaddstr(y,x+w-1,"╣");
}

/* Sparkline (▁▂▃▄▅▆▇█) szerokości w znaków */
static const char *SPKS[8]={"▁","▂","▃","▄","▅","▆","▇","█"};
static void sparkline(int y,int x,int w,const Ring *r) {
    float mn=r->mn, rng=r->mx-r->mn;
    if (rng<1.f) rng=1.f;
    int start=(HISTORY>w)?HISTORY-w:0;
    int drawn=0;
    for (int i=start;i<HISTORY&&drawn<w;i++,drawn++) {
        int lvl=(int)(7.f*(ring_get(r,i)-mn)/rng);
        if (lvl<0)lvl=0; if(lvl>7)lvl=7;
        mvaddstr(y,x+drawn,SPKS[lvl]);
    }
}

/* Pasek bipolarny: ░░░░░█████|     lub      |█████░░░░░
   w: całkowita szerokość, full: wartość = pełny pasek */
static void bar(int y,int x,int w,float v,float full,int cp) {
    if (w<5) return;
    int half=w/2;
    int fill=(int)(fabsf(v)/full*(half-1));
    if (fill>half-1) fill=half-1;
    int neg=(v<0.f);
    for (int i=0;i<w;i++) {
        int in_bar = neg ? (i>=half-fill && i<half)
                        : (i>half && i<=half+fill);
        if (i==half) { mvaddstr(y,x+i,"│"); continue; }
        if (in_bar) { attron(COLOR_PAIR(cp)); mvaddstr(y,x+i,"█"); attroff(COLOR_PAIR(cp)); }
        else        { attron(A_DIM);          mvaddstr(y,x+i,"░"); attroff(A_DIM); }
    }
}

/* Poziomnica: ────O──── szerokość w, angle/range → pozycja O */
static void bubble(int y,int x,int w,float angle,float range) {
    float norm=angle/range;
    if(norm<-1.f)norm=-1.f; if(norm>1.f)norm=1.f;
    int half=w/2, pos=half+(int)(norm*(half-1));
    for (int i=0;i<w;i++) {
        if (i==pos)       mvaddstr(y,x+i,"◉");
        else if (i==half) mvaddstr(y,x+i,"┼");
        else              mvaddstr(y,x+i,"─");
    }
}

/* Wyśrodkowany napis w polu szerokości w */
static void centered(int y,int x,int w,const char *s,int attr) {
    int len=(int)strlen(s);          /* bajty, działa dla ASCII */
    int pad=(w-len)/2; if(pad<0)pad=0;
    attron(attr);
    mvprintw(y,x+pad,"%.*s",w,s);
    attroff(attr);
}

/* ═══════════════════════════════════════════════════════════════
 *  Główny render – jeden pełny przebieg
 * ═══════════════════════════════════════════════════════════════ */
static void render(const char *port, int demo)
{
    int rows,cols; getmaxyx(stdscr,rows,cols);
    if(rows<16||cols<72){
        erase();
        mvaddstr(1,2,"Okno za małe! Minimum 72×16.");
        refresh(); return;
    }

    /* Snapshot */
    pthread_mutex_lock(&g.lock);
    float ax=g.ax,ay=g.ay,az=g.az,gx=g.gx,gy=g.gy,gz=g.gz;
    float roll=g.roll,pitch=g.pitch,yaw=g.yaw,hz=g.hz;
    int   cnt=g.cnt, conn=g.connected;
    Ring  rax=h_ax,ray=h_ay,raz=h_az,rgx=h_gx,rgy=h_gy,rgz=h_gz;
    Ring  rrl=h_roll,rpi=h_pitch,ryw=h_yaw;
    pthread_mutex_unlock(&g.lock);

    erase();

    /* ── Pasek statusu (wiersz 0) ─────────────────────────────── */
    attron(COLOR_PAIR(CP_STATUS)|A_BOLD);
    mvhline(0,0,' ',cols);
    char src[32]; snprintf(src,32,"%s",demo?"DEMO":port);
    mvprintw(0,1," STM32 IMU  │  %-14s │  %.1f Hz  │  %d próbek",
             src,hz,cnt);
    /* Status połączenia (wyrównany do prawej) */
    const char *cs = conn?"● POŁĄCZONO":"○ BRAK";
    int clen=(int)strlen(cs);
    if(conn) {
        attroff(COLOR_PAIR(CP_STATUS)|A_BOLD);
        attron(COLOR_PAIR(CP_CONN)|A_BOLD);
    } else {
        attroff(COLOR_PAIR(CP_STATUS)|A_BOLD);
        attron(COLOR_PAIR(CP_DISC)|A_BOLD);
    }
    /* Dopełnij tło */
    attron(COLOR_PAIR(CP_STATUS)); mvhline(0,0,' ',cols); attroff(COLOR_PAIR(CP_STATUS));
    attron(COLOR_PAIR(CP_STATUS)|A_BOLD);
    mvprintw(0,1," STM32 IMU  │  %-14s │  %.1f Hz  │  %d próbek",src,hz,cnt);
    attroff(COLOR_PAIR(CP_STATUS)|A_BOLD);
    if(conn) attron(COLOR_PAIR(CP_CONN)|A_BOLD);
    else     attron(COLOR_PAIR(CP_DISC)|A_BOLD);
    mvprintw(0,cols-clen-2,"%s ",cs);
    attroff(A_BOLD);
    if(conn) attroff(COLOR_PAIR(CP_CONN));
    else     attroff(COLOR_PAIR(CP_DISC));

    /* ── Ramka zewnętrzna ─────────────────────────────────────── */
    int bx=0, by=1, bw=cols, bh=rows-1;
    draw_box(by,bx,bh,bw);

    /* ── Podział na 3 kolumny ─────────────────────────────────── */
    int w1=bw/3, w2=bw/3, w3=bw-w1-w2;
    int x2=bx+w1, x3=bx+w1+w2;
    /* Pionowe separatory wewnątrz ramki */
    vbar(by+1,by+bh-2,x2);
    vbar(by+1,by+bh-2,x3);

    /* ── Nagłówki kolumn (wiersz by+1) ───────────────────────── */
    attron(COLOR_PAIR(CP_TITLE)|A_BOLD);
    centered(by+1,bx+1,    w1-1,"  ORIENTACJA  ",0);
    centered(by+1,x2+1,w2-1,"AKCELEROMETR [mg]",0);
    centered(by+1,x3+1,w3-2,"  ŻYROSKOP [dps] ",0);
    attroff(COLOR_PAIR(CP_TITLE)|A_BOLD);

    /* Pozioma linia pod nagłówkami: ╠═╬═╬═╣ */
    int tjcols[2]={x2,x3};
    inner_hline(by+2,bx,bw,tjcols,2);

    /* ── LEWA: Orientacja ─────────────────────────────────────── */
    int r=by+3, lx=bx+2, lw=w1-3;

    /* Roll */
    attron(COLOR_PAIR(CP_RED)|A_BOLD);
    mvprintw(r,lx,"Roll   %+7.1f°",roll);
    attroff(COLOR_PAIR(CP_RED)|A_BOLD);
    r++;
    attron(COLOR_PAIR(CP_RED));
    bubble(r,lx,lw,roll,45.f);
    attroff(COLOR_PAIR(CP_RED));
    r+=2;

    /* Pitch */
    attron(COLOR_PAIR(CP_GREEN)|A_BOLD);
    mvprintw(r,lx,"Pitch  %+7.1f°",pitch);
    attroff(COLOR_PAIR(CP_GREEN)|A_BOLD);
    r++;
    attron(COLOR_PAIR(CP_GREEN));
    bubble(r,lx,lw,pitch,45.f);
    attroff(COLOR_PAIR(CP_GREEN));
    r+=2;

    /* Yaw */
    attron(COLOR_PAIR(CP_CYAN)|A_BOLD);
    mvprintw(r,lx,"Yaw    %+7.1f°",yaw);
    attroff(COLOR_PAIR(CP_CYAN));
    r+=2;

    /* Sparklines kątów */
    if (r+2 < by+bh-1) {
        int sw=lw-5;
        attron(COLOR_PAIR(CP_RED));
        mvaddstr(r,lx,"Rl "); sparkline(r,lx+3,sw,&rrl);
        attroff(COLOR_PAIR(CP_RED)); r++;
        attron(COLOR_PAIR(CP_GREEN));
        mvaddstr(r,lx,"Pi "); sparkline(r,lx+3,sw,&rpi);
        attroff(COLOR_PAIR(CP_GREEN)); r++;
        attron(COLOR_PAIR(CP_CYAN));
        mvaddstr(r,lx,"Yw "); sparkline(r,lx+3,sw,&ryw);
        attroff(COLOR_PAIR(CP_CYAN));
    }

    /* ── ŚRODEK: Akcelerometr ─────────────────────────────────── */
    r=by+3;
    int mx=x2+2, mw=w2-3;
    int bar_w=mw-12, bar_x=mx+10;
    if (bar_w<8) bar_w=8;

    struct { float v; const char *l; int cp; Ring *rng; } acc[3]={
        {ax,"Ax",CP_RED,  &rax},
        {ay,"Ay",CP_GREEN,&ray},
        {az,"Az",CP_CYAN, &raz},
    };
    for (int i=0;i<3;i++) {
        attron(COLOR_PAIR(acc[i].cp)|A_BOLD);
        mvprintw(r,mx," %s %+7.0f",acc[i].l,acc[i].v);
        attroff(COLOR_PAIR(acc[i].cp)|A_BOLD);
        bar(r,bar_x,bar_w,acc[i].v,4000.f,acc[i].cp);
        r++;
    }
    r++;
    /* Sparklines acc */
    int sw=mw-5;
    for (int i=0;i<3&&r<by+bh-1;i++) {
        attron(COLOR_PAIR(acc[i].cp));
        mvprintw(r,mx," %s ",acc[i].l);
        sparkline(r,mx+4,sw,acc[i].rng);
        attroff(COLOR_PAIR(acc[i].cp)); r++;
    }

    /* ── PRAWA: Żyroskop ──────────────────────────────────────── */
    r=by+3;
    int rx=x3+2, rw=w3-3;
    int gbar_w=rw-12, gbar_x=rx+10;
    if (gbar_w<6) gbar_w=6;

    struct { float v; const char *l; int cp; Ring *rng; } gyr[3]={
        {gx,"Gx",CP_RED,  &rgx},
        {gy,"Gy",CP_GREEN,&rgy},
        {gz,"Gz",CP_CYAN, &rgz},
    };
    for (int i=0;i<3;i++) {
        attron(COLOR_PAIR(gyr[i].cp)|A_BOLD);
        mvprintw(r,rx," %s %+7.1f",gyr[i].l,gyr[i].v);
        attroff(COLOR_PAIR(gyr[i].cp)|A_BOLD);
        bar(r,gbar_x,gbar_w,gyr[i].v,250.f,gyr[i].cp);
        r++;
    }
    r++;
    /* Sparklines gyro */
    int gsw=rw-5;
    for (int i=0;i<3&&r<by+bh-1;i++) {
        attron(COLOR_PAIR(gyr[i].cp));
        mvprintw(r,rx," %s ",gyr[i].l);
        sparkline(r,rx+4,gsw,gyr[i].rng);
        attroff(COLOR_PAIR(gyr[i].cp)); r++;
    }

    /* ── Stopka ───────────────────────────────────────────────── */
    attron(A_DIM);
    mvprintw(by+bh-1,bx+2,
             " [q] wyjście   [r] reset Yaw   "
             "filtr: komplementarny  α=0.98  ODR=30 Hz ");
    attroff(A_DIM);

    refresh();
}

/* ── main ─────────────────────────────────────────────────────── */
int main(int argc,char **argv)
{
    const char *port=DEFAULT_PORT; int demo=0;
    for (int i=1;i<argc;i++) {
        if (!strcmp(argv[i],"--demo")) demo=1;
        else                           port=argv[i];
    }

    memset(&g,0,sizeof(g));
    pthread_mutex_init(&g.lock,NULL);

    TArgs targs={port,demo};
    pthread_t tid;
    pthread_create(&tid,NULL,uart_thread,&targs);
    pthread_detach(tid);

    setlocale(LC_ALL,"");
    initscr(); cbreak(); noecho();
    nodelay(stdscr,TRUE); curs_set(0); keypad(stdscr,TRUE);

    if (has_colors()) {
        start_color(); use_default_colors();
        init_pair(CP_STATUS, COLOR_BLACK,  COLOR_CYAN);
        init_pair(CP_TITLE,  COLOR_WHITE,  -1);
        init_pair(CP_RED,    COLOR_RED,    -1);
        init_pair(CP_GREEN,  COLOR_GREEN,  -1);
        init_pair(CP_CYAN,   COLOR_CYAN,   -1);
        init_pair(CP_CONN,   COLOR_GREEN,  COLOR_CYAN);
        init_pair(CP_DISC,   COLOR_RED,    COLOR_CYAN);
        init_pair(CP_DIM,    COLOR_WHITE,  -1);
        init_pair(CP_BOX,    COLOR_WHITE,  -1);
    }

    const struct timespec frame={0,33333333L};  /* 30 fps */
    while (1) {
        int ch=getch();
        if (ch=='q'||ch=='Q') break;
        if (ch=='r'||ch=='R') {
            pthread_mutex_lock(&g.lock);
            filt.yaw=0.f; g.yaw=0.f;
            pthread_mutex_unlock(&g.lock);
        }
        render(port,demo);
        nanosleep(&frame,NULL);
    }

    endwin();
    return 0;
}
