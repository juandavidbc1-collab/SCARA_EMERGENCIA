/* Prueba en el PC de la interfaz de texto (src/comms.c): eco, menu de un toque, salir con una tecla, cancelar. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "../../src/comms.c"

/* ---- control falso ---- */
static ctl_mode_t fake_mode = M_OFF;
static int n_home, n_stop, n_off, n_clear, n_move, n_pulse;
static float last_q[2];
int64_t esp_timer_get_time(void) { return 0; }
TickType_t xTaskGetTickCount(void) { return 0; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n_, uint32_t s, void *a, int p, TaskHandle_t *h, int c)
{ (void)f; (void)n_; (void)s; (void)a; (void)p; (void)h; (void)c; return 1; }
esp_err_t esp_task_wdt_add(void *p) { (void)p; return 0; }
esp_err_t esp_task_wdt_reset(void) { return 0; }
esp_err_t uart_driver_install(int a, int b, int c, int d, void *e, int f) { (void)a; (void)b; (void)c; (void)d; (void)e; (void)f; return 0; }
int uart_read_bytes(int u, void *b, uint32_t l, TickType_t t) { (void)u; (void)b; (void)l; (void)t; return 0; }
void ctl_get_status(ctl_status_t *s) { memset(s, 0, sizeof *s); s->mode = fake_mode; }
void ctl_get_mapa(int j, float o[5]) { (void)j; for (int i = 0; i < 5; i++) o[i] = i; }
bool ctl_cmd_home(void) { n_home++; return true; }
void ctl_cmd_stop(void) { n_stop++; fake_mode = M_HOLD; }
void ctl_cmd_off(void) { n_off++; }
void ctl_cmd_clear(void) { n_clear++; }
bool ctl_cmd_pulse(int j) { n_pulse++; return j == 0 || j == 1; }
bool ctl_cmd_move(const float q[2], const float t[3]) { (void)t; n_move++; last_q[0] = q[0]; last_q[1] = q[1]; return true; }
void ctl_cmd_jog(bool a, int b, int c) { (void)a; (void)b; (void)c; }
bool ctl_cmd_ref(void) { return true; }
bool ctl_cmd_ajuste(int j, float a, float b) { (void)j; (void)a; (void)b; return true; }
const char *ctl_mode_str(ctl_mode_t m) { static const char *s[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT" }; return s[m]; }
const char *ctl_fault_str(fault_t f) { (void)f; return "ninguna"; }
bool ctl_event_pop(char *b, size_t n_, uint32_t w) { (void)b; (void)n_; (void)w; return false; }
bool hal_stby_get(void) { return false; }
static int n_estop;
void ctl_cmd_estop(void) { n_estop++; }
bool ctl_cmd_motor(int j, bool on) { (void)j; (void)on; return true; }
bool ctl_motor_enabled(int j) { (void)j; return true; }
bool ctl_cmd_tool(float p) { (void)p; return true; }
bool ctl_cmd_servo(float d) { (void)d; return true; }
bool ctl_cmd_screw(float v, float t) { (void)v; (void)t; return true; }
void ctl_get_limits(int j, float *lo, float *hi) { *lo = JC[j].q_min; *hi = JC[j].q_max; }

static int fallos = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("\n  >>> FALLO: %s\n", msg); fallos++; } else printf("\n  [ok] %s\n", msg); } while (0)
static void tipea(const char *s) { for (; *s; s++) on_byte((uint8_t)*s); }
static void marca(const char *t) { printf("\n---- %s ----\n", t); }

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    marca("1) escribir 'home' + Enter (CRLF): se ve lo escrito y se ejecuta una sola vez");
    show_prompt(); tipea("home\r\n");
    CHECK(n_home == 1, "home se ejecuto una vez (el LF del CRLF se ignora)");

    marca("1b) comandos nuevos por el mismo interprete: 'estop' escrito llega al control");
    tipea("estop\r");
    CHECK(n_estop == 1, "estop por el protocolo de texto");

    marca("2) backspace: escribo 'hox', borro, sigo con 'me'");
    show_prompt(); tipea("hox\x7f" "me\r");
    CHECK(n_home == 2, "'hox'<BS>'me' = home");

    marca("3) atajo de un toque: '2' abre sensores en vivo; cualquier tecla sale");
    tipea("2");
    CHECK(g_live == LIVE_SENS, "monitor en vivo activo");
    tipea("x");
    CHECK(g_live == LIVE_NONE, "una tecla cualquiera lo detiene");

    marca("4) flechas no hacen nada raro (ESC [ A) con el prompt vacio");
    tipea("\x1b[A\x1b[B");
    CHECK(esc_state == 0 && n == 0, "secuencia de flecha descartada");

    marca("5) opcion 6: pide x y z; escribo '100 50' (z omitida = z fija)");
    tipea("6"); CHECK(g_pend == PA_GOTO, "esperando x y z");
    tipea("100 50\r");
    CHECK(n_move == 1 && g_pend == PA_NONE, "goto enviado");
    printf("  q enviada: %.2f %.2f deg\n", last_q[0] * RAD2DEG, last_q[1] * RAD2DEG);

    marca("6) opcion 6 y cancelo con Esc");
    int antes = n_move;
    tipea("6"); tipea("12"); tipea("\x1b");
    CHECK(g_pend == PA_NONE && n_move == antes && n == 0, "Esc cancela y limpia la linea");

    marca("7) opcion 6 con punto inalcanzable (500 0)");
    tipea("6"); tipea("500 0\r");
    CHECK(n_move == antes, "no se envio movimiento (mensaje de error con el alcance)");

    marca("8) opcion 6 y Enter vacio = cancelar");
    tipea("6"); tipea("\r");
    CHECK(g_pend == PA_NONE, "cancelado con Enter vacio");

    marca("9) movimiento en curso: cualquier tecla lo detiene");
    fake_mode = M_MOVE; int s0 = n_stop;
    tipea("a");
    CHECK(n_stop == s0 + 1 && n == 0, "una tecla durante MOVE = stop, y la tecla no se escribe en la linea");

    marca("10) opciones 8, 9, 0");
    tipea("8"); tipea("9"); tipea("0");
    CHECK(n_off == 1 && n_clear == 1, "apagar y borrar falla");

    marca("11) digitos dentro de un comando NO son atajos: 'ajuste 1 60 61'");
    int h = n_home; tipea("ajuste 1 60 61\r");
    CHECK(n_home == h, "se interpreto como comando completo");

    marca("12) modo maquina (fmt 1): sin eco, el latido pasa, y vuelve a texto");
    tipea("fmt 1\r"); CHECK(g_csv, "modo maquina activo");
    printf("  (a continuacion NO debe haber eco)\n");
    tipea("d 1 1 0\n");
    tipea("fmt 0\n"); CHECK(!g_csv, "modo texto otra vez");

    marca("13) menu y ayuda");
    tipea("m\r"); tipea("help\r"); tipea("cfg\r");

    printf(fallos ? "\n%d FALLOS\n" : "\nTODO OK\n", fallos);
    return fallos != 0;
}
