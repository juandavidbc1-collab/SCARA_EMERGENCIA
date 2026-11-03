/*
 * Planta y sustitutos de FreeRTOS y de la capa de hardware para correr src/control.c en el PC. Lo incluyen las
 * pruebas (tools/host_tests/test_control.c) y el simulador de ROS 2 (tools/sim/scara_sim.c), siempre después de incluir
 * src/control.c.
 *
 * Modelo: dos motores Pololu 37D (R, L, Ke, J, B de la hoja de datos) con reductor 100.37:1, fricción seca, finales de
 * carrera con histéresis y tope duro 5 grados más allá de ellos. Ejes desacoplados. Sin encoders.
 * Además, lo que más afecta a un control por tiempo: fricción que cambia, voltaje que baja, juego del reductor y rango mal medido.
 * M3: Pololu 37D 50:1 con un tornillo que cae cuando el servo se abre y se aprieta al asentar la cabeza.
 */
#pragma once
/* ---------------- reloj simulado y stubs de FreeRTOS ---------------- */
static int64_t g_now = 0;
int64_t esp_timer_get_time(void) { return g_now; }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(g_now / 1000); }
void vTaskDelayUntil(TickType_t *p, TickType_t i) { (void)p; (void)i; }
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t f, const char *n, uint32_t s, void *a, int p, TaskHandle_t *h, int c)
{ (void)f; (void)n; (void)s; (void)a; (void)p; (void)h; (void)c; return 1; }
esp_err_t esp_task_wdt_add(void *p) { (void)p; return 0; }
esp_err_t esp_task_wdt_reset(void) { return 0; }

typedef struct { int n, sz, head, count; char *data; } q_t;
QueueHandle_t xQueueCreate(int n, int sz) { q_t *q = calloc(1, sizeof *q); q->n = n; q->sz = sz; q->data = malloc((size_t)n * sz); return q; }
BaseType_t xQueueSend(QueueHandle_t h, const void *item, TickType_t t) { (void)t; q_t *q = h; if (q->count == q->n) return 0;
    memcpy(q->data + (size_t)((q->head + q->count) % q->n) * q->sz, item, q->sz); q->count++; return 1; }
BaseType_t xQueueReceive(QueueHandle_t h, void *item, TickType_t t) { (void)t; q_t *q = h; if (!q->count) return 0;
    memcpy(item, q->data + (size_t)q->head * q->sz, q->sz); q->head = (q->head + 1) % q->n; q->count--; return 1; }

/* ---------------- planta simulada ---------------- */
typedef struct {
    double v_fac;            /* voltaje real / 12 V */
    double tc_fac[NJ];       /* fricción real / fricción nominal */
    double ruido;            /* desviación de la fricción en cada arranque (fracción) */
    double juego_deg;        /* juego total del reductor */
    int    sentido[NJ];      /* -1: motor cableado al revés */
    bool   fc_roto[NJ][2];   /* nunca se pulsa */
    bool   fc_pegado[NJ][2]; /* siempre pulsado */
    bool   trabado_desde[NJ];/* se traba al llegar a trabado_en_deg */
    double trabado_en_deg[NJ];
    double j_fac[NJ];        /* inercia real / inercia estimada */
} plant_cfg_t;
typedef struct { double i, w, th; double V; int mode; /*0 libre,1 drive,2 freno*/ double salida_deg, tc_ahora; bool sw[2]; } mot_t;
static mot_t P[NJ];
static plant_cfg_t PC;
static bool g_stby = false;
static bool g_power = true;              /* 12 V de motores (el paro físico los corta en serie) */
static int  g_tool_duty = 0, g_tool_max = 0;
typedef enum { SIM_TORNILLO_NORMAL = 0, SIM_TORNILLO_ATASCO, SIM_TORNILLO_BARRIDO, SIM_TORNILLO_NO_CAE } sim_tornillo_t;
static sim_tornillo_t g_screw_mode = SIM_TORNILLO_NORMAL;
static float  g_servo_deg = -1.0f;
static double g_m3_w, g_m3_th, g_m3_i_real, g_m3_torque_real, g_m3_torque_max;  /* rad/s, rad, A, N·m en el eje de salida */
static double g_m3_th_tornillo = NAN;    /* ángulo de M3 cuando cayó el tornillo, que puede ser negativo; NAN: no hay tornillo */
static bool   g_m3_barrido = false;
static bool g_lock[NJ] = { false, false };   /* eje trabado (inyección de fallas) */
static const double J_carga[NJ] = { 0.04, 0.01 };      /* inercia del eslabón en el eje de salida [kg m²], estimada */
static double sw_min_deg[NJ] = { -125.0, -145.0 }, sw_max_deg[NJ] = { 125.0, 145.0 };
static const int motor_dir_fisico[NJ] = { -1, -1 };    /* sentido físico del motor con duty>0 (antes de motor_sign) */
static double g_phys_max_exceso = 0;     /* cuánto se pasó del final de carrera (grados) */

#define NGEAR 100.37
#define FC_HIST_DEG 0.3

static double plant_randn(void)
{
    const double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

/* Lo que queda el eje más allá de donde suelta el final de carrera tras "ref": el usuario mide el ángulo final, así que
 * q_home ya lo incluye. Medido con esta misma planta; la prueba "queda en el punto de Home" lo vigila. */
static const double SOLTAR_EXTRA_DEG[NJ] = { 1.91, 1.23 };

static void plant_reset(const plant_cfg_t *c, double q1, double q2)
{
    memset(P, 0, sizeof P);
    if (c) PC = *c; else memset(&PC, 0, sizeof PC);
    if (PC.v_fac == 0) PC.v_fac = 1;
    const double q[NJ] = { q1, q2 };
    for (int j = 0; j < NJ; j++) {
        if (PC.tc_fac[j] == 0) PC.tc_fac[j] = 1;
        if (PC.sentido[j] == 0) PC.sentido[j] = 1;
        if (PC.j_fac[j] == 0) PC.j_fac[j] = 1;
        P[j].tc_ahora = PC.tc_fac[j];
        P[j].salida_deg = q[j];
        P[j].th = q[j] * NGEAR * 2 * M_PI / 360;
        g_lock[j] = false;
        sw_min_deg[j] = JC[j].q_home * 180.0 / M_PI - FC_HIST_DEG - SOLTAR_EXTRA_DEG[j];
        sw_max_deg[j] = (JC[j].q_home + JC[j].rango) * 180.0 / M_PI;
    }
    g_stby = false; g_now = 0; g_power = true; g_tool_duty = g_tool_max = 0;
    g_screw_mode = SIM_TORNILLO_NORMAL; g_servo_deg = -1.0f;
    g_m3_w = g_m3_th = g_m3_i_real = g_m3_torque_real = g_m3_torque_max = 0.0; g_m3_th_tornillo = NAN; g_m3_barrido = false;
    g_phys_max_exceso = 0;
}

void hal_stby(bool on) { g_stby = on; }
bool hal_stby_get(void) { return g_stby; }
void hal_motor_free(int j) { P[j].mode = 0; P[j].V = 0; }
void hal_motor_brake(int j) { P[j].mode = 2; P[j].V = 0; }
void hal_all_free(void) { for (int j = 0; j < NJ; j++) hal_motor_free(j); }
void hal_motor_drive(int j, int duty)
{
    duty *= JC[j].motor_sign;
    if (duty == 0) { hal_motor_free(j); return; }
    if (abs(duty) > PWM_MAX) duty = duty > 0 ? PWM_MAX : -PWM_MAX;
    if (P[j].mode != 1) P[j].tc_ahora = PC.tc_fac[j] * (1.0 + PC.ruido * plant_randn());
    P[j].mode = 1; P[j].V = duty / 1023.0 * 12.0 * PC.v_fac * motor_dir_fisico[j] * PC.sentido[j];
}
bool hal_fc_raw(int j, int side)
{
    if (PC.fc_roto[j][side]) return false;
    if (PC.fc_pegado[j][side]) return true;
    const double q = P[j].salida_deg;
    if (side == 0) {
        if (q <= sw_min_deg[j]) P[j].sw[0] = true; else if (q > sw_min_deg[j] + FC_HIST_DEG) P[j].sw[0] = false;
    } else {
        if (q >= sw_max_deg[j]) P[j].sw[1] = true; else if (q < sw_max_deg[j] - FC_HIST_DEG) P[j].sw[1] = false;
    }
    return P[j].sw[side];
}
void hal_tool_drive(int duty) { g_tool_duty = duty; if (abs(duty) > g_tool_max) g_tool_max = abs(duty); }
void hal_tool_free(void) { g_tool_duty = 0; }
void hal_servo_set_deg(float deg)
{
    /* Al abrirse, cae un tornillo nuevo sobre el hueco; tiene que llegar a M3 antes de atornillar. */
    if (deg > SERVO_GRADOS_MAX / 4 && g_servo_deg <= SERVO_GRADOS_MAX / 4 && g_servo_deg >= 0.0f) {
        g_m3_th_tornillo = g_screw_mode == SIM_TORNILLO_NO_CAE ? NAN : g_m3_th;
        g_m3_barrido = false;
    }
    g_servo_deg = deg;
}
void hal_init(void) {}

/* Torque que el tornillo opone a M3 [N·m]. La rosca cuesta poco hasta que asienta la cabeza; ahí sube como un resorte. */
static double m3_par_tornillo(void)
{
    if (isnan(g_m3_th_tornillo)) return 0.0;
    const double vueltas = (g_m3_th - g_m3_th_tornillo) / (2 * M_PI);
    if (vueltas < 0.0) return 0.0;
    if (g_m3_barrido) return 0.005;
    const double asiento = g_screw_mode == SIM_TORNILLO_ATASCO ? 3.0 : SCREW_VUELTAS_META;
    const double par = M3_MODEL.torque_rosca + (vueltas > asiento ? 0.32 * (vueltas - asiento) * 2 * M_PI : 0.0);
    if (g_screw_mode == SIM_TORNILLO_BARRIDO && par > 0.25) { g_m3_barrido = true; return 0.005; }
    return par;
}

static void m3_planta_1ms(void)
{
    const double R = M3_MODEL.r, Ke = M3_MODEL.ke, Kt = M3_MODEL.kt;
    const double J = 0.0049, dt = 1e-4;   /* inercia en el eje de salida: constante mecánica de unos 50 ms, estimada */
    const double Tf = Kt * M3_MODEL.i0;
    const bool activo = g_stby && g_power && g_tool_duty != 0;
    const double V = activo ? g_tool_duty * TOOL_MOTOR_SIGN / 1023.0 * 12.0 : 0.0;
    if (V < 0.0) g_m3_th_tornillo = NAN;   /* supuesto sin medir: al girar al revés, la punta se suelta sola del tornillo */
    for (int k = 0; k < 10; k++) {
        const double par = m3_par_tornillo();
        g_m3_i_real = activo ? (V - Ke * g_m3_w) / R : 0.0;
        const double Tm = Kt * g_m3_i_real;
        const double resiste = Tf + par;
        double a;
        if (fabs(g_m3_w) < 1e-3 && fabs(Tm) <= resiste) { g_m3_w = 0.0; a = 0.0; }   /* calado o quieto */
        else a = (Tm - copysign(resiste, g_m3_w != 0.0 ? g_m3_w : Tm)) / J;
        double wn = g_m3_w + a * dt;
        if ((g_m3_w > 0.0 && wn < 0.0) || (g_m3_w < 0.0 && wn > 0.0)) wn = 0.0;
        g_m3_w = wn;
        g_m3_th += g_m3_w * dt;
        g_m3_torque_real = par;
        if (activo && par > g_m3_torque_max) g_m3_torque_max = par;
    }
}

static void planta_1ms(void)
{
    const double R = 2.4, L = 1.5e-3, Ke = 0.01056, B = 2.97e-6, Tc = 3e-3;
    const double dt = 1e-4;
    m3_planta_1ms();
    for (int j = 0; j < NJ; j++) {
        mot_t *m = &P[j];
        const double J = 3e-6 + J_carga[j] * PC.j_fac[j] / (NGEAR * NGEAR);
        const double V = ((g_stby && g_power) ? m->V : 0.0);
        const bool abierto = (!g_stby || m->mode == 0);
        for (int k = 0; k < 10; k++) {
            if (g_lock[j] || (PC.trabado_desde[j] && fabs(m->salida_deg - PC.trabado_en_deg[j]) < 1.0)) { m->w = 0; m->i = 0; continue; }
            if (abierto) m->i = 0; else m->i += (V - R * m->i - Ke * m->w) / L * dt;
            const double T = Tc * m->tc_ahora;
            const double Tl = T * (m->w > 0.5 ? 1 : (m->w < -0.5 ? -1 : m->w / 0.5));
            m->w += (Ke * m->i - B * m->w - Tl) / J * dt;
            m->th += m->w * dt;
            /* juego: la salida solo sigue al motor cuando se acaba la holgura */
            const double mot = m->th / NGEAR / (2 * M_PI) * 360.0;
            if (mot > m->salida_deg + PC.juego_deg / 2) m->salida_deg = mot - PC.juego_deg / 2;
            if (mot < m->salida_deg - PC.juego_deg / 2) m->salida_deg = mot + PC.juego_deg / 2;
            const double a = m->salida_deg;            /* tope mecánico duro 5° más allá del final de carrera */
            const double lo = sw_min_deg[j] - 5, hi = sw_max_deg[j] + 5;
            if (a < lo || a > hi) {
                const double lim = a < lo ? lo : hi, dq = lim - a;
                m->salida_deg = lim; m->th += dq / 360.0 * 2 * M_PI * NGEAR; m->w = 0;
            }
            const double ex = fmax(sw_min_deg[j] - a, a - sw_max_deg[j]);
            if (ex > g_phys_max_exceso) g_phys_max_exceso = ex;
        }
    }
}

static double plant_error_mm(double x, double y)
{
    const double a = P[0].salida_deg * M_PI / 180, b = (P[0].salida_deg + P[1].salida_deg) * M_PI / 180;
    return hypot(KIN.l1 * cos(a) + KIN.l2 * cos(b) - x, KIN.l1 * sin(a) + KIN.l2 * sin(b) - y);
}
