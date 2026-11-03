#include "comms.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "control.h"
#include "hal.h"
#include "kinematics.h"
#include "motion.h"
#include "uros_cmd.h"

#define RAD2DEG (180.0f / (float)M_PI)
#define DEG2RAD ((float)M_PI / 180.0f)
#define LIVE_PERIODO_MS   250      /* refresco de los monitores en vivo del modo texto */
#define CSV_IDLE_MS       3000     /* sin tráfico en modo máquina -> vuelve al modo texto */

typedef enum { LIVE_NONE = 0, LIVE_SENS, LIVE_TELE } live_t;
typedef enum { PA_NONE = 0, PA_GOTO, PA_ANG, PA_PULSE } pending_t;

static bool      g_csv = false;          /* true = modo máquina (teleop.py): sin eco, sin atajos, telemetría CSV */
static bool      g_tele_csv = false;
static live_t    g_live = LIVE_NONE;
static pending_t g_pend = PA_NONE;
static char      linea[96];
static int       n = 0;
static int       esc_state = 0;
static bool      prev_cr = false;
static int64_t   t_last_rx_us = 0;

static int64_t now_us(void) { return esp_timer_get_time(); }

/* ------------------------------------------------------------------ presentación */
static void show_prompt(void)
{
    if (g_csv) return;
    switch (g_pend) {
    case PA_GOTO:  printf("  x y z [mm] (ej: 120 80 0)  [Esc = cancelar] > %s", linea); break;
    case PA_ANG:   printf("  q1 q2 [grados] (ej: 20 90)  [Esc = cancelar] > %s", linea); break;
    case PA_PULSE: printf("  eje (1 o 2)  [Esc = cancelar] > %s", linea); break;
    default:       printf("> %s", linea); break;
    }
}

static void print_menu(void)
{
    printf("\n"
           "==================== MENU SCARA ====================\n"
           "  1  Ver estado actual\n"
           "  2  Sensores en vivo (topes y encoders)    [cualquier tecla = salir]\n"
           "  3  Telemetria en vivo (q, x, y)           [cualquier tecla = salir]\n"
           "  4  Pulso de prueba de sentido del motor\n"
           "  5  HOME (tope, tope contrario y vuelve)    [cualquier tecla = detener]\n"
           "  6  Ir a una posicion x y z (mm)           [cualquier tecla = detener]\n"
           "  7  Ir a angulos q1 q2 (grados)            [cualquier tecla = detener]\n"
           "  8  DETENER (mantiene la posicion)\n"
           "  9  APAGAR motores\n"
           "  0  Borrar falla\n"
           "  m  este menu      help  lista de comandos escritos      cfg  configuracion\n"
           "  Esc o cualquier tecla en un monitor/movimiento = parar. Lo que escribes se ve en pantalla.\n"
           "====================================================\n");
}

static void print_help(void)
{
    printf("\nCOMANDOS ESCRITOS (terminan con Enter):\n"
           "  home                    ir a un tope, al contrario, y volver a donde estaba\n"
           "  goto x y z              ir a (x,y,z) en mm        ej: goto 120 80 0\n"
           "  ang q1 q2               ir a angulos en grados    ej: ang 20 90\n"
           "  stop | off | clear      detener y mantener | apagar motores | borrar falla\n"
           "  elbow +1 | elbow -1     elegir codo preferido\n"
           "  estop                   paro de emergencia (se rearma con clear)\n"
           "  motor 1|2|3 on|off      activar o desactivar un motor (3 = atornillador)\n"
           "  tool pct                motor 3: -100 a 100 %%\n"
           "  ref                     recorre los ejes de tope a tope: corrige la posicion y la velocidad\n"
           "  ajuste j pedido medido  corrige la velocidad con un angulo medido a mano (j = 1 o 2)\n"
           "  mapa                    muestra lo que midio Home\n"
           "  pulse j                 pulso corto para comprobar el sentido del motor j\n"
           "  tele 1 | tele 0         telemetria en vivo (cualquier tecla la detiene)\n"
           "  status | sens | cfg     estado | sensores en vivo | configuracion\n"
           "  fmt 1 | fmt 0           modo maquina (lo usa teleop.py) | modo texto\n");
}

static const char *si_no(bool b) { return b ? "si" : "no"; }

static void print_cfg(void)
{
    float rmin = fabsf(KIN.l1 - KIN.l2), rmax = KIN.l1 + KIN.l2;
    printf("\nCONFIGURACION\n");
    printf("  Eslabones: L1 = %.1f mm, L2 = %.1f mm  ->  alcance de %.1f a %.1f mm desde la base\n", KIN.l1, KIN.l2, rmin, rmax);
    printf("  Altura fija z = %.1f mm (se acepta +-%.1f)   Codo preferido: %+d\n", Z_FIJO_MM, Z_TOL_MM, motion_get_elbow());
    for (int j = 0; j < NJ; j++) {
        float lo, hi; ctl_get_limits(j, &lo, &hi);
        printf("  %s: limites vigentes %.1f a %.1f deg | q en el 1er tope %.1f deg | Home empieza por el tope %s | rango %.1f deg | motor_sign %+d\n",
               JC[j].name, lo * RAD2DEG, hi * RAD2DEG, JC[j].q_home * RAD2DEG,
               "MIN", JC[j].rango * RAD2DEG, JC[j].motor_sign);
    }
}

static void line_status(void)
{
    ctl_status_t s; ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    printf("[%-6s] q1=%8.2f q2=%8.2f deg | x=%7.1f y=%7.1f mm | home J1:%s J2:%s | topes J1[min:%s max:%s] J2[min:%s max:%s] | falla: %s\n",
           ctl_mode_str(s.mode), s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, si_no(s.homed[0]), si_no(s.homed[1]),
           (s.fc & 1) ? "ON" : "-", (s.fc & 2) ? "ON" : "-", (s.fc & 4) ? "ON" : "-", (s.fc & 8) ? "ON" : "-",
           s.fault == F_NONE ? "ninguna" : ctl_fault_str(s.fault));
}

static void line_sens(void)
{
    ctl_status_t s; ctl_get_status(&s);
    printf("topes J1[min:%s max:%s] J2[min:%s max:%s] | drivers %s\n",
           (s.fc & 1) ? "ON" : "-", (s.fc & 2) ? "ON" : "-", (s.fc & 4) ? "ON" : "-", (s.fc & 8) ? "ON" : "-",
           hal_stby_get() ? "ENCENDIDOS" : "apagados");
}

static void telemetria_csv(void)
{
    ctl_status_t s; ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    printf("ST,%s,%d,%d,%.3f,%.3f,%.2f,%.2f,%d,%u,%.0f,%.0f,%.3f,%.3f\n", ctl_mode_str(s.mode), s.homed[0], s.homed[1],
           s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, (int)s.fault, s.fc, s.duty[0], s.duty[1],
           s.qref[0] * RAD2DEG, s.qref[1] * RAD2DEG);
}

/* ------------------------------------------------------------------ órdenes de movimiento */
/* La validación, la cinemática inversa y el encolado viven en motion.c (compartido con micro-ROS). */
static void cmd_goto(float x, float y, float z)
{
    char m[200];
    motion_res_t r = motion_goto_xyz(x, y, z, m, sizeof m);
    printf(r == MOTION_OK ? "%s (cualquier tecla = detener)\n" : "%s\n", m);
}

static void cmd_ang(float a, float b)
{
    char m[200];
    motion_res_t r = motion_goto_ang(a, b, m, sizeof m);
    printf(r == MOTION_OK ? "%s (cualquier tecla = detener)\n" : "%s\n", m);
}

/* ------------------------------------------------------------------ intérprete de comandos escritos */
static void procesar(char *l)
{
    while (*l && ((unsigned char)*l < 32 || *l == ' ')) l++;       /* ignora Esc/espacios al inicio */
    for (char *p = l; *p; p++) *p = (char)tolower((unsigned char)*p);
    if (!*l) return;

    int dm, a, b, j; float x, y, z;
    if (sscanf(l, "d %d %d %d", &dm, &a, &b) == 3) { ctl_cmd_jog(dm != 0, a, b); return; }
    if (!strncmp(l, "home", 4)) { ctl_cmd_home(); return; }
    if (sscanf(l, "goto %f %f %f", &x, &y, &z) == 3) { cmd_goto(x, y, z); return; }
    if (sscanf(l, "goto %f %f", &x, &y) == 2) { cmd_goto(x, y, Z_FIJO_MM); return; }
    if (sscanf(l, "ang %f %f", &x, &y) == 2) { cmd_ang(x, y); return; }
    if (!strncmp(l, "stop", 4)) { ctl_cmd_stop(); printf("#INFO: orden enviada: detener.\n"); return; }
    if (!strncmp(l, "off", 3)) { ctl_cmd_off(); return; }
    if (!strncmp(l, "clear", 5)) { ctl_cmd_clear(); return; }
    if (sscanf(l, "elbow %d", &a) == 1) { motion_set_elbow(a); printf("#OK: codo preferido = %+d\n", motion_get_elbow()); return; }
    if (sscanf(l, "pulse %d", &j) == 1) {
        if (ctl_cmd_pulse(j - 1)) printf("#INFO: pulso de prueba en el eje %d...\n", j);
        else printf("#ERROR: el eje debe ser 1 o 2.\n");
        return;
    }
    if (sscanf(l, "fmt %d", &a) == 1) {
        g_csv = a != 0; g_tele_csv = false; g_live = LIVE_NONE; g_pend = PA_NONE;
        printf("#OK: modo %s\n", g_csv ? "maquina (CSV, sin eco)" : "texto");
        return;
    }
    if (sscanf(l, "tele %d", &a) == 1) {
        if (g_csv) { g_tele_csv = a != 0; printf("#OK: telemetria CSV %s\n", a ? "activada" : "desactivada"); }
        else if (a) { g_live = LIVE_TELE; printf("Telemetria en vivo. Cualquier tecla = salir.\n"); }
        else g_live = LIVE_NONE;
        return;
    }
    if (!strcmp(l, "ref")) { ctl_cmd_ref(); return; }
    if (sscanf(l, "ajuste %d %f %f", &j, &x, &y) == 3) {
        if (!ctl_cmd_ajuste(j - 1, x, y)) printf("#ERROR: usa ajuste eje pedido medido, con el eje 1 o 2\n");
        return;
    }
    if (!strcmp(l, "mapa")) {
        for (int i = 0; i < NJ; i++) {
            float m[5]; ctl_get_mapa(i, m);
            printf("J%d: v- = %.1f, v+ = %.1f grados/s | arranque %.0f y %.0f ms | ajuste %.4f\n", i + 1, m[0], m[1], m[2] * 1000.0f, m[3] * 1000.0f, m[4]);
        }
        return;
    }
    if (!strncmp(l, "status", 6)) { line_status(); return; }
    if (!strncmp(l, "sens", 4)) {
        if (g_csv) line_sens();
        else { g_live = LIVE_SENS; printf("Sensores en vivo. Mueve los ejes a mano y pisa los topes. Cualquier tecla = salir.\n"); }
        return;
    }
    if (!strncmp(l, "cfg", 3)) { print_cfg(); return; }
    if (!strncmp(l, "menu", 4) || !strcmp(l, "m")) { print_menu(); return; }
    if (!strncmp(l, "help", 4) || *l == '?') { print_help(); return; }
    /* estop, motor, tool, ctrl, rstform, rst: mismo intérprete que ROS 2 */
    {
        char reply[200];
        (void)uros_cmd_exec(l, reply, sizeof reply);
        printf("%s\n", reply);
    }
}

/* ------------------------------------------------------------------ menú numérico y entradas con argumentos */
static bool motion_active(void)
{
    ctl_status_t s; ctl_get_status(&s);
    return s.mode == M_HOMING || s.mode == M_MOVE || s.mode == M_PULSE;
}

static void menu_action(char c)
{
    switch (c) {
    case '1': line_status(); break;
    case '2': g_live = LIVE_SENS; printf("Sensores en vivo. Mueve los ejes a mano y pisa los topes. Cualquier tecla = salir.\n"); break;
    case '3': g_live = LIVE_TELE; printf("Telemetria en vivo. Cualquier tecla = salir.\n"); break;
    case '4': g_pend = PA_PULSE; break;
    case '5': printf("Home: recorre cada eje de tope a tope midiendo tiempos. M3 debe estar arriba.\n"); ctl_cmd_home(); break;
    case '6':
        printf("Alcance: %.1f a %.1f mm desde la base; z fija = %.1f mm. Si omites z se usa la z fija.\n",
               fabsf(KIN.l1 - KIN.l2), KIN.l1 + KIN.l2, Z_FIJO_MM);
        g_pend = PA_GOTO; break;
    case '7':
        {
            float l1, h1, l2, h2; ctl_get_limits(0, &l1, &h1); ctl_get_limits(1, &l2, &h2);
            printf("Limites: q1 %.0f a %.0f deg, q2 %.0f a %.0f deg\n", l1 * RAD2DEG, h1 * RAD2DEG, l2 * RAD2DEG, h2 * RAD2DEG);
        }
        g_pend = PA_ANG; break;
    case '8': ctl_cmd_stop(); printf("Orden enviada: detener (mantiene la posicion).\n"); break;
    case '9': ctl_cmd_off(); break;
    case '0': ctl_cmd_clear(); printf("Orden enviada: borrar falla.\n"); break;
    default: break;
    }
}

static void finalizar_pending(void)
{
    pending_t p = g_pend;
    g_pend = PA_NONE;
    if (!linea[0]) { printf("[cancelado]\n"); return; }
    float x, y, z; int j;
    switch (p) {
    case PA_GOTO: {
        int k = sscanf(linea, "%f %f %f", &x, &y, &z);
        if (k == 2) cmd_goto(x, y, Z_FIJO_MM);
        else if (k == 3) cmd_goto(x, y, z);
        else printf("#ERROR: escribe dos o tres numeros separados por espacio, ej: 120 80 0\n");
        break; }
    case PA_ANG:
        if (sscanf(linea, "%f %f", &x, &y) == 2) cmd_ang(x, y);
        else printf("#ERROR: escribe dos numeros separados por espacio, ej: 20 90\n");
        break;
    case PA_PULSE:
        if (sscanf(linea, "%d", &j) == 1 && ctl_cmd_pulse(j - 1)) printf("#INFO: pulso de prueba en el eje %d...\n", j);
        else printf("#ERROR: el eje debe ser 1 o 2.\n");
        break;
    default: break;
    }
}

/* Esc / cualquier tecla: sale de lo que esté activo */
static void salir_de_lo_activo(const char *motivo)
{
    if (g_live != LIVE_NONE) { g_live = LIVE_NONE; printf("\n[monitor detenido]\n"); }
    else if (g_pend != PA_NONE) { g_pend = PA_NONE; printf("\n[cancelado]\n"); }
    else if (motion_active()) { ctl_cmd_stop(); printf("\n[%s]\n", motivo); }
    else { printf("\n"); }
    n = 0; linea[0] = 0;
    show_prompt();
}

static void on_byte(uint8_t c)
{
    t_last_rx_us = now_us();

    /* ---- modo máquina: líneas sin eco ni atajos ---- */
    if (g_csv) {
        if (c == '\n' || c == '\r') { if (n) { linea[n] = 0; n = 0; procesar(linea); } }
        else if (n < (int)sizeof linea - 1) linea[n++] = (char)c;
        else n = 0;
        return;
    }

    /* ---- modo texto ---- */
    if (c == '\n' && prev_cr) { prev_cr = false; return; }       /* Enter = CR+LF: ignorar el LF */
    prev_cr = (c == '\r');

    /* flechas y otras secuencias ESC [ ... se descartan */
    if (esc_state == 1) { if (c == '[' || c == 'O') { esc_state = 2; return; } esc_state = 0; }
    else if (esc_state == 2) { if (c >= 0x40 && c <= 0x7E) esc_state = 0; return; }
    if (c == 0x1B) { esc_state = 1; salir_de_lo_activo("detenido por Esc"); return; }

    /* cualquier tecla detiene un monitor en vivo o un movimiento en curso */
    if (g_live != LIVE_NONE || (g_pend == PA_NONE && motion_active())) {
        salir_de_lo_activo("detenido por tecla");
        return;
    }

    if (c == '\r' || c == '\n') {
        printf("\n");
        linea[n] = 0; n = 0;
        if (g_pend != PA_NONE) finalizar_pending();
        else if (linea[0]) procesar(linea);
        linea[0] = 0;
        if (g_live == LIVE_NONE) show_prompt();
        return;
    }
    if (c == 0x08 || c == 0x7F) {                                   /* borrar */
        if (n > 0) { n--; linea[n] = 0; printf("\b \b"); }
        return;
    }
    if (c < 32 || c > 126) return;

    if (n == 0 && g_pend == PA_NONE && c >= '0' && c <= '9') {      /* atajo de menú: un solo toque */
        printf("%c\n", c);
        menu_action((char)c);
        if (g_live == LIVE_NONE) show_prompt();
        return;
    }
    if (n < (int)sizeof linea - 1) { linea[n++] = (char)c; linea[n] = 0; printf("%c", c); }   /* eco */
}

static void imprimir_evento(const char *ev)
{
    if (g_csv || g_live != LIVE_NONE) { printf("%s\n", ev); return; }
    printf("\n%s\n", ev);
    show_prompt();
}

static void comms_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);
    TickType_t t_csv = xTaskGetTickCount(), t_live = xTaskGetTickCount();

    printf("\n\n*** SCARA v5 - firmware de EMERGENCIA, sin encoders ***\n");
    print_cfg();
    line_sens();
    print_menu();
    show_prompt();

    for (;;) {
        uint8_t buf[64];
        int r = uart_read_bytes(UART_NUM_0, buf, sizeof buf, pdMS_TO_TICKS(5));
        for (int i = 0; i < r; i++) on_byte(buf[i]);

        char ev[160];
        while (ctl_event_pop(ev, sizeof ev, 0)) imprimir_evento(ev);

        TickType_t t = xTaskGetTickCount();
        if (g_csv) {
            if (g_tele_csv && (t - t_csv) >= pdMS_TO_TICKS(TELE_PERIODO_MS)) { t_csv = t; telemetria_csv(); }
            if (now_us() - t_last_rx_us > CSV_IDLE_MS * 1000LL) {      /* teleop.py se cerró sin avisar */
                g_csv = false; g_tele_csv = false; n = 0; linea[0] = 0;
                ctl_cmd_jog(false, 0, 0);
                printf("\n[sin trafico del PC: vuelvo al modo texto]\n");
                show_prompt();
            }
        } else if (g_live != LIVE_NONE && (t - t_live) >= pdMS_TO_TICKS(LIVE_PERIODO_MS)) {
            t_live = t;
            if (g_live == LIVE_SENS) line_sens(); else line_status();
        }
        esc_state = (esc_state == 1 && r == 0) ? 0 : esc_state;      /* un Esc suelto no deja bloqueado el intérprete */
        esp_task_wdt_reset();
    }
}

void comms_start(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);           /* lo que se escribe aparece al instante */
    t_last_rx_us = now_us();
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    xTaskCreatePinnedToCore(comms_task, "comms", 6144, NULL, 5, NULL, 0);
}
