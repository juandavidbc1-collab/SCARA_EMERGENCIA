#include "control.h"
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "hal.h"
#include "screw.h"

#define R2D (180.0f / (float)M_PI)
#define D2R ((float)M_PI / 180.0f)
#define MS(x) ((int64_t)(x) * 1000)
#define L_MIN 0
#define L_MAX 1

/* Por dentro todo va en grados; la interfaz (control.h) sigue en radianes, como en el firmware principal. */
#define Q_HOME(j) (JC[j].q_home * R2D)
#define RANGO(j)  (JC[j].rango * R2D)

typedef enum { C_HOME, C_MOVE, C_STOP, C_OFF, C_CLEAR, C_PULSE, C_MOTOR, C_TOOL, C_SERVO, C_SCREW, C_REF, C_AJUSTE } cmd_type_t;
typedef struct {
    cmd_type_t t;
    float q[NJ];
    float tgt[3];
    int   j;
    float g[2];
} cmd_t;

static QueueHandle_t cmd_q, evt_q;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* latido del modo manual (escrito por la tarea de comunicación, leído por el lazo) */
static struct { bool dm; int8_t dir[NJ]; int64_t t_us; } hb;

/* estado visible (copia protegida con lock) */
static ctl_status_t pub;
static float pub_mapa[NJ][5], pub_lim[NJ][2];
static volatile bool estop_req = false;   /* el paro no pasa por la cola: no se puede perder ni esperar */
static float ajuste[NJ] = { J1_AJUSTE, J2_AJUSTE };   /* sobrevive a un nuevo Home; un reinicio vuelve a config.h */
static const int HOME_ORDER[NJ] = HOME_ORDER_INIT;

/* ---------------------------------------------------------------- estado interno del lazo */
typedef enum { T_NADA, T_TIEMPO, T_HASTA_FC, T_SOLTAR, T_JOG, T_FRENO } tarea_t;

typedef struct {
    tarea_t tarea;
    int dir, pwm, lado;             /* lado: final de carrera que se busca o se suelta; -1 si ninguno */
    float duty;
    int64_t dur_us, on_us, fin_freno_us, freno_us;
    bool ini[2];                    /* qué finales de carrera estaban pulsados al arrancar */
    bool llego;
    float q0, q_dest;
    bool con_destino;
} eje_t;

enum { M_A_MIN, M_SOLTAR_0, M_A_MIN_1, M_RAPIDO, M_A_MIN_2, M_SOLTAR_1, M_SUBE_0, M_SOLTAR_2, M_BAJA, M_SOLTAR_3, M_SUBE, M_SOLTAR_4,
       M_PULSOS_BAJA, M_SOLTAR_5, M_PULSOS_SUBE, M_SOLTAR_6, M_BAJA_FIN, M_SOLTAR_7, M_FIN };

static struct {
    ctl_mode_t mode;
    fault_t fault;
    bool homed[NJ], en[NJ], tool_en, jog_lock;
    float q[NJ], qref[NJ];
    /* lo que mide Home, por eje y sentido (0 = hacia -, 1 = hacia +) */
    float vel[NJ][2], t0[NJ][2], arranque[NJ][2], trav[NJ][2];
    eje_t eje[NJ];
    bool fc[NJ][2];
    uint8_t fc_cnt[NJ][2];
    int64_t t_ant, fault_t_us;
    uint32_t ticks;
    /* M3 y servo */
    float tool_cmd, tool_duty, servo_deg;
    int64_t tool_t_us;
    struct { screw_phase_t fase; screw_result_t res; float t, T, meta, vps, vueltas; } screw;
    /* mover */
    float obj[NJ], tgt[3];
    int etapa[NJ];
    /* Home */
    bool h_ref;                      /* false: Home completo; true: solo "ref" */
    int h_step;
    int map_paso, map_n;
    int64_t map_ton;
    float map_T[2], map_Ton[2], map_te, h_primero[NJ];
    int map_N[2];
    /* ref */
    int ref_fase;
    float ref_T[2], ref_primero, ref_q0;
    float ref_sesgo[NJ]; bool ref_sesgo_ok[NJ];   /* distancia fija del reposo al final de carrera, medida en el primer ref tras Home */
    int p_j;
} S;

/* ---------------------------------------------------------------- utilidades */
static int64_t now_us(void) { return esp_timer_get_time(); }

static void emit(const char *fmt, ...)
{
    char b[160];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    xQueueSend(evt_q, b, 0);
}

const char *ctl_mode_str(ctl_mode_t m)
{
    static const char *n[] = { "OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT" };
    return (m <= M_FAULT) ? n[m] : "?";
}

const char *ctl_fault_str(fault_t f)
{
    switch (f) {
    case F_NONE:         return "ninguna";
    case F_FOLLOW:       return "error de seguimiento";
    case F_STALL:        return "motor trabado";
    case F_LIMIT:        return "final de carrera inesperado (se perdio la referencia)";
    case F_HOME_TIMEOUT: return "tiempo agotado buscando un final de carrera";
    case F_DIAG_FC:      return "MIN y MAX del mismo eje presionados a la vez (cableado o final de carrera)";
    case F_ESTOP:        return "paro de emergencia";
    case F_PERDIDO:      return "el brazo no estaba donde se creia";
    }
    return "?";
}

static float limite_bajo(int j) { return Q_HOME(j) + HOME_MARGEN_RAD * R2D; }
static float limite_alto(int j) { return Q_HOME(j) + RANGO(j) - HOME_MARGEN_RAD * R2D; }

static bool all_homed(void)
{
    for (int j = 0; j < NJ; j++) if (S.en[j] && !S.homed[j]) return false;
    return true;
}

static bool algun_eje_activo(void)
{
    for (int j = 0; j < NJ; j++) if (S.en[j]) return true;
    return false;
}

static void servo_mover(float deg)
{
    S.servo_deg = deg;
    hal_servo_set_deg(deg);
}

static bool screw_activo(void) { return S.screw.fase == SCREW_APRIETE; }

/* Sin medición, "sin_medir" es lo único honesto que se puede decir de un tornillo terminado. Los ceros no son mediciones. */
static void screw_informar(void)
{
    emit("#TORNILLO: %s vueltas=%.2f prof=%.2fmm avance=%.0f%% torque=0.000Nm fuerza=0N presion=0.0MPa corriente=0.00A t=%.1fs",
         screw_result_str(S.screw.res), S.screw.vueltas, screw_profundidad_mm(&M3_MODEL, S.screw.vueltas),
         100.0f * S.screw.vueltas / S.screw.meta, S.screw.t);
}

static void tool_detener(void)
{
    S.tool_cmd = 0.0f;
    if (screw_activo()) {
        S.screw.fase = SCREW_FIN;
        S.screw.res = SCREW_R_CANCELADO;
        screw_informar();
    }
}

static void tool_apagar(void)
{
    tool_detener();
    S.tool_duty = 0.0f;
    hal_tool_free();
}

/*
 * Cuánto ha girado un eje que lleva T segundos encendido. En un movimiento largo es v (T - t0). Al empezar, el motor todavía
 * no ha perdido todo t0: se aproxima con una exponencial, porque si no, un toque corto en manual se estima muy mal.
 */
static float q_en_marcha(int j)
{
    const eje_t *e = &S.eje[j];
    const int s = e->dir > 0;
    const float T = (float)e->on_us * 1e-6f, t0 = S.t0[j][s];
    const float perdido = t0 > 0.0f ? t0 * (1.0f - expf(-T / (0.8f * t0))) : t0;
    const float v = S.vel[j][s] * ((float)e->pwm - PWM_SIN_MOVER) / ((float)HOME_DUTY - PWM_SIN_MOVER);
    const float t = T - perdido;
    return e->q0 + (float)e->dir * v * (t > 0.0f ? t : 0.0f);
}

static bool eje_girando(int j)
{
    const tarea_t t = S.eje[j].tarea;
    return t == T_TIEMPO || t == T_HASTA_FC || t == T_SOLTAR || t == T_JOG;
}

/* Deja de contar: el ángulo se queda con la mejor estimación que haya. */
static void congelar(int j)
{
    eje_t *e = &S.eje[j];
    if (eje_girando(j)) S.q[j] = q_en_marcha(j);
    S.qref[j] = S.q[j];
    e->tarea = T_NADA;
    e->duty = 0.0f;
}

static void set_fault(fault_t f, const char *fmt, ...)
{
    if (S.mode == M_FAULT) return;
    char b[110];
    va_list ap; va_start(ap, fmt);
    vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    for (int j = 0; j < NJ; j++) { congelar(j); hal_motor_brake(j); S.homed[j] = false; }
    S.mode = M_FAULT;
    S.fault = f;
    S.fault_t_us = now_us();
    S.jog_lock = true;
    tool_apagar();
    servo_mover(SERVO_CERRADO_DEG);
    emit("#FALLA: %s. Motores apagados. Usa clear y repite Home.", b);
}

/* Paro de emergencia: manda sobre cualquier otro estado, incluida una falla ya activa. */
static void entrar_paro(void)
{
    if (S.mode == M_FAULT && S.fault == F_ESTOP) return;
    const bool en_home = S.mode == M_HOMING && !S.h_ref;
    for (int j = 0; j < NJ; j++) {
        congelar(j);
        hal_motor_brake(j);
        if (en_home) S.homed[j] = false;              /* un Home a medias no deja referencia */
    }
    tool_apagar();
    servo_mover(SERVO_CERRADO_DEG);
    S.mode = M_FAULT;
    S.fault = F_ESTOP;
    S.fault_t_us = now_us();
    S.jog_lock = true;
    emit("#FALLA: PARO DE EMERGENCIA. Motores apagados. Usa clear para rearmar.");
}

/* ---------------------------------------------------------------- sensores */
static void leer_sensores(void)
{
    for (int j = 0; j < NJ; j++) for (int s = 0; s < 2; s++) {
        const bool raw = hal_fc_raw(j, s);
        if (raw == S.fc[j][s]) { S.fc_cnt[j][s] = 0; }
        else if (++S.fc_cnt[j][s] >= FC_ANTIREBOTE_MS * CONTROL_HZ / 1000) {
            S.fc[j][s] = raw; S.fc_cnt[j][s] = 0;
        }
    }
}

/* ---------------------------------------------------------------- tareas de un eje */
static void arrancar(int j, tarea_t t, int dir, int pwm, int lado, int64_t dur_us, int64_t freno_us)
{
    eje_t *e = &S.eje[j];
    e->tarea = t;
    e->dir = dir;
    e->pwm = pwm;
    e->lado = lado;
    e->dur_us = dur_us;
    e->freno_us = freno_us;
    e->on_us = 0;
    e->llego = false;
    e->con_destino = false;
    e->q0 = S.q[j];
    e->ini[L_MIN] = S.fc[j][L_MIN];
    e->ini[L_MAX] = S.fc[j][L_MAX];
}

static void frenar_eje(int j)
{
    eje_t *e = &S.eje[j];
    hal_motor_brake(j);
    S.q[j] = e->con_destino ? e->q_dest : q_en_marcha(j);
    e->duty = 0.0f;
    e->tarea = T_FRENO;
    e->fin_freno_us = now_us() + e->freno_us;
}

static float angulo_del_tope(int j, int lado) { return lado == L_MIN ? Q_HOME(j) : Q_HOME(j) + RANGO(j); }

static void motor_con_rampa(int j, int64_t dt_us)
{
    eje_t *e = &S.eje[j];
    const float obj = (float)(e->dir * e->pwm);
    float d = obj - e->duty;
    const float paso = (float)DUTY_SLEW_PER_MS * (float)dt_us / 1000.0f;
    if (d > paso) d = paso;
    if (d < -paso) d = -paso;
    e->duty += d;
    hal_motor_drive(j, (int)lroundf(e->duty));
    e->on_us += dt_us;
}

static void eje_tick(int j, int64_t dt_us)
{
    eje_t *e = &S.eje[j];
    if (e->tarea == T_NADA) return;
    if (e->tarea == T_FRENO) {
        if (now_us() >= e->fin_freno_us) { hal_motor_free(j); e->tarea = T_NADA; }
        return;
    }
    const int J = j + 1;
    switch (e->tarea) {
    case T_HASTA_FC: {
        const int otro = !e->lado;
        if (S.fc[j][e->lado]) { e->llego = true; frenar_eje(j); return; }
        if (S.fc[j][otro] && !e->ini[otro]) { set_fault(F_LIMIT, "J%d: se pulso el final de carrera contrario, revisa el sentido del motor o los cables", J); return; }
        if (S.fc[j][otro] && e->on_us > MS(T_SALIR_FC_MS)) { set_fault(F_LIMIT, "J%d: no suelta el final de carrera, revisa el sentido del motor", J); return; }
        if (e->on_us > e->dur_us) { set_fault(F_HOME_TIMEOUT, "J%d: no llego al final de carrera, esta trabado o no funciona", J); return; }
        break;
    }
    case T_SOLTAR:
        if (!S.fc[j][e->lado]) { frenar_eje(j); return; }
        if (e->on_us > MS(T_SALIR_FC_MS)) { set_fault(F_LIMIT, "J%d: no suelta el final de carrera, revisa el sentido del motor", J); return; }
        break;
    case T_TIEMPO:
        if (e->lado >= 0 && S.fc[j][e->lado]) { e->llego = true; frenar_eje(j); return; }
        for (int s = 0; s < 2; s++) {
            const bool hacia_el = (s == L_MAX) == (e->dir > 0);      /* empujar contra un tope ya pulsado también es perderse */
            if (s != e->lado && S.fc[j][s] && (!e->ini[s] || hacia_el)) {
                set_fault(F_LIMIT, "J%d: final de carrera inesperado, el brazo no esta donde se creia", J);
                return;
            }
        }
        if (e->on_us >= e->dur_us) { frenar_eje(j); return; }
        break;
    case T_JOG: {
        /* Manual: los topes y los límites paran, y un tope pulsado de paso corrige la estimación. */
        const int lado = e->dir > 0 ? L_MAX : L_MIN;
        if (S.fc[j][lado]) {
            frenar_eje(j);
            if (S.homed[j]) S.q[j] = angulo_del_tope(j, lado);
            return;
        }
        if (S.homed[j]) {
            const float q = q_en_marcha(j);
            if ((e->dir > 0 && q >= limite_alto(j)) || (e->dir < 0 && q <= limite_bajo(j))) { frenar_eje(j); return; }
        }
        break;
    }
    default:
        return;
    }
    motor_con_rampa(j, dt_us);
}

static void hasta_fc(int j, int lado) { arrancar(j, T_HASTA_FC, lado == L_MAX ? 1 : -1, HOME_DUTY, lado, MS(T_TRAVESIA_MS), MS(PAUSA_FRENO_MS)); }
static void soltar_fc(int j, int lado) { arrancar(j, T_SOLTAR, lado == L_MAX ? -1 : 1, HOME_DUTY_SLOW, lado, 0, MS(PAUSA_FRENO_MS)); }

/* ---------------------------------------------------------------- mover */
static void mover_tramo(int j, float destino)
{
    const float d = destino - S.q[j];
    const int s = d > 0.0f;
    const float T = fabsf(d) / S.vel[j][s] + S.t0[j][s];
    if (fabsf(d) < MOVER_MIN_DEG || T <= 0.0f) { S.q[j] = destino; return; }
    arrancar(j, T_TIEMPO, s ? 1 : -1, HOME_DUTY, -1, (int64_t)(T * 1e6f), MS(PAUSA_FRENO_MS));
    S.eje[j].q_dest = destino;
    S.eje[j].con_destino = true;
}

static void mover_iniciar(const float q[NJ])
{
    for (int j = 0; j < NJ; j++) {
        S.obj[j] = S.en[j] ? q[j] : S.q[j];              /* un eje desactivado no participa */
        S.qref[j] = S.obj[j];
        S.etapa[j] = (!S.en[j] || fabsf(S.obj[j] - S.q[j]) < MOVER_MIN_DEG) ? 2 : 0;
    }
}

/* Punto desde donde se hace el último tramo. Devuelve false si conviene ir directo. */
static bool punto_previo(int j, float *pre)
{
    const float q = S.q[j], obj = S.obj[j];
    float p = obj - APROX_DEG;
    if (fabsf(p - q) >= TRAMO_MIN_DEG && p >= limite_bajo(j)) { *pre = p; return true; }
    p = (q < p ? q : p) - TRAMO_MIN_DEG;               /* retrocede más */
    if (p >= limite_bajo(j)) { *pre = p; return true; }
    p = q + TRAMO_MIN_DEG;                             /* junto al tope: primero avanza */
    if (obj - p >= APROX_DEG) { *pre = p; return true; }
    return false;
}

static bool mover_tick(void)
{
    bool fin = true;
    for (int j = 0; j < NJ; j++) {
        if (S.eje[j].tarea != T_NADA) { fin = false; continue; }
        if (S.etapa[j] >= 2) continue;
        fin = false;
        float pre;
        if (S.etapa[j] == 0 && punto_previo(j, &pre)) { mover_tramo(j, pre); S.etapa[j] = 1; }
        else { mover_tramo(j, S.obj[j]); S.etapa[j] = 2; }
    }
    return fin;
}

/* ---------------------------------------------------------------- Home: medir sin encoder */
static void pulso_hacia(int j, int lado) { arrancar(j, T_TIEMPO, lado == L_MAX ? 1 : -1, HOME_DUTY, lado, MS(PULSO_MS), MS(PAUSA_FRENO_MS)); }

static void publicar_mapa(int j)
{
    portENTER_CRITICAL(&lock);
    pub_mapa[j][0] = S.vel[j][0]; pub_mapa[j][1] = S.vel[j][1];
    pub_mapa[j][2] = S.t0[j][0]; pub_mapa[j][3] = S.t0[j][1];
    pub_mapa[j][4] = ajuste[j];
    pub_lim[j][0] = limite_bajo(j) * D2R; pub_lim[j][1] = limite_alto(j) * D2R;
    portEXIT_CRITICAL(&lock);
}

static bool calcular_mapa(int j)
{
    const int J = j + 1;
    for (int s = 0; s < 2; s++) {
        if (S.map_N[s] < 2) { set_fault(F_LIMIT, "J%d: muy pocos pulsos, el rango parece muy corto", J); return false; }
        float t0 = (S.map_Ton[s] - S.map_T[s]) / (float)(S.map_N[s] - 1);
        if (fabsf(t0) >= PULSO_MS * 0.5e-3f) {
            emit("#AVISO: J%d tiempo de arranque raro (%.0f ms); se usa 0", J, t0 * 1000.0f);
            t0 = 0.0f;
        }
        /* El recorrido continuo termina en el final de carrera, sin frenar: su retraso es el del arranque, no t0.
         * t0 = arranque - inercia al frenar; te = arranque + inercia al frenar. */
        const float arranque = (S.map_te + t0) / 2.0f;
        const float v = ajuste[j] * RANGO(j) / (S.map_T[s] - arranque);
        if (v < VEL_MIN_DEG_S || v > VEL_MAX_DEG_S) { set_fault(F_LIMIT, "J%d: velocidad imposible (%.1f grados/s), revisa el rango", J, v); return false; }
        S.vel[j][s] = v;
        S.t0[j][s] = t0;
        S.trav[j][s] = S.map_T[s];
        S.arranque[j][s] = S.map_T[s] - RANGO(j) / v;
    }
    const float vm = (S.vel[j][0] + S.vel[j][1]) / 2.0f;
    if (fabsf(S.vel[j][0] - S.vel[j][1]) / vm > VEL_DIFERENCIA_MAX) {
        set_fault(F_LIMIT, "J%d: va muy distinto en cada sentido (%.1f y %.1f grados/s), revisa la mecanica", J, S.vel[j][0], S.vel[j][1]);
        return false;
    }
    publicar_mapa(j);
    emit("#HOME: eje %d medido: %.1f y %.1f grados/s, arranque %.0f y %.0f ms. Limites por software: %.1f a %.1f deg.",
         J, S.vel[j][0], S.vel[j][1], S.t0[j][0] * 1000.0f, S.t0[j][1] * 1000.0f, limite_bajo(j), limite_alto(j));
    return true;
}

/* Sin encoder, "donde estaba" se deduce: lo que tardó en llegar al primer tope, ahora que se conoce la velocidad. */
static void home_terminar(void)
{
    float q[NJ];
    for (int j = 0; j < NJ; j++) {
        q[j] = S.q[j];
        if (!S.en[j]) continue;
        q[j] = Q_HOME(j) + S.vel[j][0] * (S.h_primero[j] - S.arranque[j][0]);
        if (q[j] < limite_bajo(j)) q[j] = limite_bajo(j);
        if (q[j] > limite_alto(j)) q[j] = limite_alto(j);
    }
    mover_iniciar(q);
    S.tgt[0] = S.tgt[1] = S.tgt[2] = NAN;
    S.mode = M_MOVE;
    emit("#HOME: terminado. Volviendo a la posicion inicial (estimada: q1 = %.2f, q2 = %.2f deg)...", q[0], q[1]);
}

static void mapeo_tick(void)
{
    const int j = HOME_ORDER[S.h_step];
    eje_t *e = &S.eje[j];
    if (e->tarea != T_NADA) return;
    switch (S.map_paso) {
    case M_A_MIN:    hasta_fc(j, L_MIN); S.map_paso = M_SOLTAR_0; break;
    case M_SOLTAR_0: S.h_primero[j] = (float)e->on_us * 1e-6f; soltar_fc(j, L_MIN); S.map_paso = M_A_MIN_1; break;
    case M_A_MIN_1:  hasta_fc(j, L_MIN); S.map_paso = M_RAPIDO; break;
    /* Desde el sobrepaso del tope, a velocidad normal, cuánto tarda en salir: incluye lo que sigue girando al frenar. */
    case M_RAPIDO:   arrancar(j, T_SOLTAR, 1, HOME_DUTY, L_MIN, 0, MS(PAUSA_FRENO_MS)); S.map_paso = M_A_MIN_2; break;
    case M_A_MIN_2:  S.map_te = (float)e->on_us * 1e-6f; hasta_fc(j, L_MIN); S.map_paso = M_SOLTAR_1; break;
    case M_SOLTAR_1: soltar_fc(j, L_MIN); S.map_paso = M_SUBE_0; break;
    /* Donde queda el eje al soltar depende de cómo llegó. Los tiempos se miden saliendo de un reposo tras un recorrido
     * largo, el mismo de "ref": si no, cada "ref" vería un cambio de velocidad que no existe. */
    case M_SUBE_0:   hasta_fc(j, L_MAX); S.map_paso = M_SOLTAR_2; break;
    case M_SOLTAR_2: soltar_fc(j, L_MAX); S.map_paso = M_BAJA; break;
    case M_BAJA:     hasta_fc(j, L_MIN); S.map_paso = M_SOLTAR_3; break;
    case M_SOLTAR_3: S.map_T[0] = (float)e->on_us * 1e-6f; soltar_fc(j, L_MIN); S.map_paso = M_SUBE; break;
    case M_SUBE:     hasta_fc(j, L_MAX); S.map_paso = M_SOLTAR_4; break;
    case M_SOLTAR_4: S.map_T[1] = (float)e->on_us * 1e-6f; soltar_fc(j, L_MAX); S.map_paso = M_PULSOS_BAJA; S.map_n = -1; break;
    case M_PULSOS_SUBE:
    case M_PULSOS_BAJA: {
        const int lado = S.map_paso == M_PULSOS_SUBE ? L_MAX : L_MIN;
        if (S.map_n < 0) { S.map_n = 0; S.map_ton = 0; }
        else {
            S.map_ton += e->on_us;
            if (e->llego) {
                S.map_Ton[lado] = (float)S.map_ton * 1e-6f;
                S.map_N[lado] = S.map_n;
                S.map_paso++;
                break;
            }
            if (S.map_ton > MS(T_TRAVESIA_MS)) { set_fault(F_HOME_TIMEOUT, "J%d: no llego al final de carrera en los pulsos", j + 1); break; }
        }
        pulso_hacia(j, lado);
        S.map_n++;
        break;
    }
    case M_SOLTAR_5: soltar_fc(j, L_MIN); S.map_paso = M_PULSOS_SUBE; S.map_n = -1; break;
    case M_SOLTAR_6: soltar_fc(j, L_MAX); S.map_paso = M_BAJA_FIN; break;
    case M_BAJA_FIN: hasta_fc(j, L_MIN); S.map_paso = M_SOLTAR_7; break;
    case M_SOLTAR_7: soltar_fc(j, L_MIN); S.map_paso = M_FIN; break;
    default:
        S.q[j] = S.qref[j] = Q_HOME(j);
        if (!calcular_mapa(j)) return;
        S.ref_sesgo_ok[j] = false;
        S.homed[j] = true;
        S.map_paso = M_A_MIN;
        do { S.h_step++; } while (S.h_step < NJ && !S.en[HOME_ORDER[S.h_step]]);
        if (S.h_step >= NJ) home_terminar();
        else emit("#HOME: ahora el eje %d...", HOME_ORDER[S.h_step] + 1);
        break;
    }
}

/* ---------------------------------------------------------------- "ref": corrige posición y velocidad */
static void ref_iniciar(void)
{
    S.h_ref = true;
    S.ref_fase = 0;
    S.h_step = 0;
    while (S.h_step < NJ && !S.en[HOME_ORDER[S.h_step]]) S.h_step++;
    S.mode = M_HOMING;
}

static void ref_tick(void)
{
    if (S.h_step >= NJ) {
        S.mode = M_HOLD;
        emit("#LLEGO: ejes recorridos, en su punto de Home | q1=%.3f q2=%.3f deg", S.q[0], S.q[1]);
        return;
    }
    const int j = HOME_ORDER[S.h_step];
    eje_t *e = &S.eje[j];
    if (e->tarea != T_NADA) return;
    const int J = j + 1;
    switch (S.ref_fase) {
    case 0: {
        const float T = (S.q[j] - Q_HOME(j) + DESVIO_MAX_DEG) / S.vel[j][0] + S.arranque[j][0] + 1.0f;
        arrancar(j, T_HASTA_FC, -1, HOME_DUTY, L_MIN, (int64_t)(T * 1e6f), MS(PAUSA_FRENO_MS));
        S.ref_q0 = S.q[j];
        S.ref_fase = 1;
        break;
    }
    case 1:
        S.ref_primero = (float)e->on_us * 1e-6f;
        soltar_fc(j, L_MIN);
        S.ref_fase = 2;
        break;
    case 2: hasta_fc(j, L_MAX); S.ref_fase = 3; break;
    case 3: S.ref_T[1] = (float)e->on_us * 1e-6f; soltar_fc(j, L_MAX); S.ref_fase = 4; break;
    case 4: hasta_fc(j, L_MIN); S.ref_fase = 5; break;
    case 5: S.ref_T[0] = (float)e->on_us * 1e-6f; soltar_fc(j, L_MIN); S.ref_fase = 6; break;
    default: {
        S.q[j] = S.qref[j] = Q_HOME(j);
        float cambio = 0.0f;
        for (int s = 0; s < 2; s++) {
            const float v = RANGO(j) / (S.ref_T[s] - S.arranque[j][s]);
            if (v < VEL_MIN_DEG_S || v > VEL_MAX_DEG_S) { set_fault(F_PERDIDO, "J%d: velocidad imposible (%.1f grados/s)", J, v); return; }
            cambio += (v / S.vel[j][s] - 1.0f) * 50.0f;
            S.vel[j][s] = v;
            S.trav[j][s] = S.ref_T[s];     /* ajuste recalcula el arranque con este tiempo */
        }
        /* Se compara con la velocidad recién medida: si el eje solo va más lento, el brazo no se ha perdido. */
        const float crudo = S.ref_q0 - S.vel[j][0] * (S.ref_primero - S.arranque[j][0]) - Q_HOME(j);
        /* Q_HOME es donde reposa al soltar, no donde se pisa el final: esa distancia es fija y la mide el primer ref tras Home. */
        const bool primero = !S.ref_sesgo_ok[j];
        if (primero) { S.ref_sesgo[j] = crudo; S.ref_sesgo_ok[j] = true; }
        const float desvio = primero ? crudo : crudo - S.ref_sesgo[j];
        const float tope = primero ? DESVIO_PRIMER_REF_MAX_DEG : DESVIO_MAX_DEG;
        publicar_mapa(j);
        emit("#REF J%d diferencia=%.2f grados, velocidad %.1f y %.1f grados/s (cambio %+.1f %%)", J, desvio, S.vel[j][0], S.vel[j][1], cambio);
        if (fabsf(desvio) > tope) {
            set_fault(F_PERDIDO, "J%d se perdio %.1f grados: se trabo, patino o alguien lo movio", J, desvio);
            return;
        }
        S.ref_fase = 0;
        do { S.h_step++; } while (S.h_step < NJ && !S.en[HOME_ORDER[S.h_step]]);
        break;
    }
    }
}

/* ---------------------------------------------------------------- órdenes */
static bool quieto_o_avisa(const char *que)
{
    if (S.mode == M_OFF || S.mode == M_HOLD) return true;
    emit("#ERROR: no se puede %s, el robot esta en modo %s (usa stop).", que, ctl_mode_str(S.mode));
    return false;
}

static void do_cmd(const cmd_t *c)
{
    switch (c->t) {
    case C_HOME:
        if (screw_activo()) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        if (!quieto_o_avisa("iniciar Home")) break;
        if (!algun_eje_activo()) { emit("#ERROR: no hay ejes activos para hacer Home."); break; }
        for (int j = 0; j < NJ; j++) if (S.en[j]) S.homed[j] = false;
        S.h_ref = false;
        S.h_step = 0;
        S.map_paso = M_A_MIN;
        while (!S.en[HOME_ORDER[S.h_step]]) S.h_step++;
        S.mode = M_HOMING;
        hal_stby(true);
        emit("#HOME: iniciando. M3 debe estar arriba. Eje %d: de tope a tope, midiendo tiempos.", HOME_ORDER[S.h_step] + 1);
        break;

    case C_REF:
        if (screw_activo()) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        if (!all_homed()) { emit("#ERROR: primero ejecuta HOME."); break; }
        if (!algun_eje_activo()) { emit("#ERROR: no hay ejes activos."); break; }
        if (!quieto_o_avisa("recorrer los ejes")) break;
        ref_iniciar();
        hal_stby(true);
        break;

    case C_MOVE: {
        if (screw_activo()) { emit("#ERROR: M3 esta atornillando: el brazo no se mueve hasta que termine."); break; }
        if (!all_homed()) { emit("#ERROR: primero ejecuta HOME."); break; }
        if (!quieto_o_avisa("mover")) break;
        float q[NJ];
        for (int j = 0; j < NJ; j++) q[j] = c->q[j] * R2D;
        mover_iniciar(q);
        memcpy(S.tgt, c->tgt, sizeof S.tgt);
        S.mode = M_MOVE;
        hal_stby(true);
        break;
    }

    case C_STOP:
        tool_detener();
        S.jog_lock = true;                             /* que no reanude solo si las teclas siguen pulsadas */
        if (S.mode == M_HOMING || S.mode == M_PULSE) {
            for (int j = 0; j < NJ; j++) { congelar(j); hal_motor_free(j); }
            S.mode = M_OFF; hal_stby(false);
            emit("#INFO: proceso detenido, motores apagados.");
        } else if (S.mode == M_MOVE || S.mode == M_JOG) {
            for (int j = 0; j < NJ; j++) congelar(j);
            S.mode = M_HOLD;
            emit("#INFO: movimiento detenido, ejes frenados.");
        }
        break;

    case C_OFF:
        if (S.mode != M_FAULT) {
            for (int j = 0; j < NJ; j++) { congelar(j); hal_motor_free(j); S.homed[j] = false; }   /* libre, nadie mediría si lo mueven */
            tool_apagar();
            S.jog_lock = true;
            S.mode = M_OFF; hal_stby(false);
            emit("#INFO: motores apagados (STBY = 0). Hay que repetir Home.");
        }
        break;

    case C_CLEAR:
        if (S.mode == M_FAULT) {
            const bool paro = S.fault == F_ESTOP;
            S.mode = M_OFF; S.fault = F_NONE;
            if (paro) {
                emit(all_homed() ? "#INFO: paro rearmado. La posicion es la estimada al parar; si algo se movio, usa ref. Motores apagados."
                                 : "#INFO: paro rearmado. Faltan ejes por referenciar: ejecuta HOME.");
            } else {
                emit("#INFO: falla borrada. Los ejes quedaron SIN referencia: ejecuta HOME de nuevo.");
            }
        }
        break;

    case C_PULSE:
        if (screw_activo()) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        if (!quieto_o_avisa("hacer el pulso")) break;
        if (!S.en[c->j]) { emit("#ERROR: el eje %d esta desactivado.", c->j + 1); break; }
        S.p_j = c->j;
        arrancar(c->j, T_TIEMPO, 1, HOME_DUTY, L_MAX, MS(PULSE_MS), MS(PAUSA_FRENO_MS));
        S.mode = M_PULSE;
        hal_stby(true);
        break;

    case C_MOTOR: {
        if (S.mode == M_HOMING || S.mode == M_PULSE) { emit("#ERROR: no se puede cambiar un motor durante %s.", ctl_mode_str(S.mode)); break; }
        const bool on = c->g[0] != 0.0f;
        if (c->j == NJ) {                              /* motor 3 */
            S.tool_en = on;
            if (!on) tool_apagar();
            emit("#INFO: M3 %s.", on ? "activado" : "desactivado");
            break;
        }
        const int j = c->j;
        if (S.en[j] == on) break;
        if (on) {
            S.en[j] = true;
            if (S.mode == M_MOVE || S.mode == M_JOG) {    /* el movimiento en curso no contaba con este eje */
                for (int i = 0; i < NJ; i++) congelar(i);
                S.mode = M_HOLD;
                emit("#INFO: movimiento detenido al reactivar J%d.", j + 1);
            }
            emit("#INFO: J%d activado. Haz Home: sin encoder no se sabe si lo movieron.", j + 1);
        } else {
            congelar(j);
            S.en[j] = false;
            S.homed[j] = false;
            hal_motor_free(j);
            S.etapa[j] = 2;
            S.tgt[0] = S.tgt[1] = S.tgt[2] = NAN;         /* ya no se llegará a la (x, y) pedida */
            if (!algun_eje_activo() && (S.mode == M_HOLD || S.mode == M_MOVE || S.mode == M_JOG)) S.mode = M_OFF;
            emit("#INFO: J%d desactivado y sin referencia.", j + 1);
        }
        break;
    }

    case C_TOOL:
        if (S.mode == M_FAULT) { emit("#ERROR: M3 no arranca con una falla o paro activo."); break; }
        if (!S.tool_en) { emit("#ERROR: M3 esta desactivado (usa motor 3 on)."); break; }
        if (screw_activo()) { emit("#ERROR: M3 esta atornillando: espera o detenlo."); break; }
        S.tool_cmd = c->g[0] * (float)TOOL_DUTY_MAX / 100.0f;
        S.tool_t_us = now_us();
        break;

    case C_SERVO:
        if (S.mode == M_FAULT && c->g[0] != SERVO_CERRADO_DEG) { emit("#ERROR: con una falla o paro activo el servo no se abre."); break; }
        servo_mover(c->g[0]);
        break;

    case C_SCREW: {
        if (S.mode == M_FAULT) { emit("#ERROR: no se atornilla con una falla o paro activo."); break; }
        if (!S.tool_en) { emit("#ERROR: M3 esta desactivado (usa motor 3 on)."); break; }
        if (screw_activo()) { emit("#ERROR: M3 ya esta atornillando."); break; }
        if (S.mode != M_OFF && S.mode != M_HOLD) { emit("#ERROR: el brazo debe estar quieto para atornillar (modo %s).", ctl_mode_str(S.mode)); break; }
        const float meta = c->g[0] > 0.0f ? c->g[0] : SCREW_VUELTAS_META;
        const float torque = c->g[1] > 0.0f ? c->g[1] : SCREW_TORQUE_META;
        const float duty = screw_duty_para_torque(&M3_MODEL, torque);
        const float vps = screw_vueltas_por_s(&M3_MODEL, duty);
        if (vps <= 0.0f || duty > TOOL_DUTY_MAX) { emit("#ERROR: con %.2f N.m el motor no gira o pide demasiado PWM.", torque); break; }
        memset(&S.screw, 0, sizeof S.screw);
        S.screw.fase = SCREW_APRIETE;
        S.screw.meta = meta;
        S.screw.vps = vps;
        S.screw.T = meta / vps * SCREW_MARGEN_T;
        S.tool_cmd = duty;
        emit("#INFO: apretando por tiempo: %.1f s con PWM %.0f. Sin medicion del resultado.", S.screw.T, duty);
        break;
    }

    case C_AJUSTE: {
        const int j = c->j;
        if (!S.homed[j]) { emit("#ERROR: primero ejecuta HOME."); break; }
        if (!quieto_o_avisa("ajustar")) break;
        const float recorrido = c->g[0] - Q_HOME(j), factor = (c->g[1] - Q_HOME(j)) / recorrido;
        if (fabsf(recorrido) < 30.0f) { emit("#ERROR: pide un angulo a mas de 30 grados del punto de Home para medir bien."); break; }
        if (factor < 0.8f || factor > 1.2f) { emit("#ERROR: difiere %.0f %%: revisa q_home y rango en config.c.", (factor - 1.0f) * 100.0f); break; }
        ajuste[j] *= factor;
        for (int s = 0; s < 2; s++) {
            S.vel[j][s] *= factor;
            S.arranque[j][s] = S.trav[j][s] - RANGO(j) / S.vel[j][s];
        }
        S.q[j] = S.qref[j] = c->g[1];
        publicar_mapa(j);
        emit("#OK: pon en config.h J%d_AJUSTE = %.4ff", j + 1, ajuste[j]);
        break;
    }
    }
}

/* ---------------------------------------------------------------- modos */
static void modo_jog(bool fresh_dm, const int8_t dir[NJ])
{
    bool moving = false;
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) continue;
        eje_t *e = &S.eje[j];
        const int want = (fresh_dm && !S.jog_lock) ? dir[j] : 0;
        if (e->tarea == T_JOG) {
            if (want != e->dir) frenar_eje(j);
            moving = true;
        } else if (e->tarea == T_FRENO) {
            moving = true;
        } else if (want != 0) {
            const int lado = want > 0 ? L_MAX : L_MIN;
            const bool en_limite = S.homed[j] && (want > 0 ? S.q[j] >= limite_alto(j) : S.q[j] <= limite_bajo(j));
            if (!S.fc[j][lado] && !en_limite) {
                arrancar(j, T_JOG, want, HOME_DUTY, -1, 0, MS(JOG_FRENO_MS));
                moving = true;
            }
        }
        S.qref[j] = S.q[j];
    }
    if (!moving && !fresh_dm) S.mode = M_HOLD;
}

static void modo_move(void)
{
    if (!mover_tick()) return;
    S.mode = M_HOLD;
    float x, y;
    kin_fk(&KIN, S.q[0] * D2R, S.q[1] * D2R, &x, &y);
    if (isnan(S.tgt[0]))
        emit("#LLEGO: q1=%.3f q2=%.3f deg | estimado x=%.2f y=%.2f mm", S.q[0], S.q[1], x, y);
    else
        emit("#LLEGO: objetivo x=%.2f y=%.2f z=%.2f | estimado x=%.2f y=%.2f | q1=%.3f q2=%.3f deg", S.tgt[0], S.tgt[1], S.tgt[2], x, y, S.q[0], S.q[1]);
}

static void modo_pulse(void)
{
    if (S.eje[S.p_j].tarea != T_NADA) return;
    S.mode = M_OFF;
    hal_stby(false);
    emit("#PULSO eje %d: dq estimado=%+.2f deg. Sin encoder, mira el brazo: q debe AUMENTAR. Si va al reves, cambia motor_sign en config.c.",
         S.p_j + 1, S.q[S.p_j] - S.eje[S.p_j].q0);
}

/* ---------------------------------------------------------------- salidas */
static void salida_m3(void)
{
    if (!screw_activo() && S.tool_cmd != 0.0f && now_us() - S.tool_t_us > TOOL_MAX_ON_MS * 1000LL) {
        S.tool_cmd = 0.0f;
        emit("#INFO: M3 apagado: no se renovo la orden en %d ms.", TOOL_MAX_ON_MS);
    }
    float d = S.tool_cmd - S.tool_duty;
    if (d >  DUTY_SLEW_PER_MS) d =  DUTY_SLEW_PER_MS;
    if (d < -DUTY_SLEW_PER_MS) d = -DUTY_SLEW_PER_MS;
    S.tool_duty += d;
    hal_tool_drive((int)lroundf(S.tool_duty));
}

static void screw_tick(float dt)
{
    if (!screw_activo()) return;
    S.screw.t += dt;
    S.screw.vueltas = fminf(S.screw.meta, S.screw.vps * S.screw.t);
    if (S.screw.t >= S.screw.T) {
        S.screw.fase = SCREW_FIN;
        S.screw.res = SCREW_R_SIN_MEDIR;
        S.tool_cmd = 0.0f;
        screw_informar();
    }
}

static void aplicar_salidas(void)
{
    if (S.mode == M_OFF) {
        hal_all_free();
        salida_m3();
        hal_stby(S.tool_duty != 0.0f);               /* M3 funciona con el brazo apagado */
        return;
    }
    if (S.mode == M_FAULT) {
        if (now_us() - S.fault_t_us < FALLA_FRENO_MS * 1000LL) return;   /* sigue frenando */
        hal_all_free(); hal_stby(false);
        return;
    }
    hal_stby(true);
    salida_m3();
    for (int j = 0; j < NJ; j++) {
        if (!S.en[j]) { hal_motor_free(j); continue; }
        if (S.eje[j].tarea != T_NADA) continue;      /* ya lo manejó su tarea */
        /* quieto: frenado es lo más parecido a mantener la posición */
        if (S.mode == M_PULSE || (S.mode == M_HOMING && !S.homed[j])) hal_motor_free(j);
        else hal_motor_brake(j);
    }
}

static void publicar(void)
{
    portENTER_CRITICAL(&lock);
    pub.mode = S.mode; pub.fault = S.fault;
    pub.fc = 0;
    pub.en = (S.en[0] ? 1 : 0) | (S.en[1] ? 2 : 0) | (S.tool_en ? 4 : 0);
    pub.ctrl = CTRL_POR_TIEMPO;
    pub.tool_duty = S.tool_duty;
    pub.tool_vueltas = S.screw.vueltas; pub.tool_prof = screw_profundidad_mm(&M3_MODEL, S.screw.vueltas);
    pub.servo_deg = S.servo_deg; pub.screw_fase = (uint8_t)S.screw.fase;
    for (int j = 0; j < NJ; j++) {
        pub.homed[j] = S.homed[j];
        pub.q[j] = (eje_girando(j) ? q_en_marcha(j) : S.q[j]) * D2R;
        pub.qref[j] = S.qref[j] * D2R;
        pub.duty[j] = S.eje[j].duty;
        pub.fc |= (S.fc[j][0] ? 1 : 0) << (2 * j);
        pub.fc |= (S.fc[j][1] ? 1 : 0) << (2 * j + 1);
    }
    portEXIT_CRITICAL(&lock);
}

/* ---------------------------------------------------------------- tick de 1 ms */
static void tick(void)
{
    const int64_t now = now_us();
    int64_t dt_us = now - S.t_ant;
    S.t_ant = now;
    if (dt_us <= 0) dt_us = 1000;

    leer_sensores();
    screw_tick((float)dt_us * 1e-6f);

    /* el paro va antes que cualquier orden o modo */
    if (estop_req) { estop_req = false; entrar_paro(); }

    cmd_t c;
    while (xQueueReceive(cmd_q, &c, 0) == pdTRUE) do_cmd(&c);

    /* MIN y MAX del mismo eje presionados a la vez es imposible: se revisa siempre, porque sin encoder es de lo poco que hay */
    if (S.ticks >= 200 && S.mode != M_FAULT) {
        for (int j = 0; j < NJ; j++) if (S.fc[j][0] && S.fc[j][1]) { set_fault(F_DIAG_FC, "J%d: MIN y MAX a la vez, revisa J9 a J12", j + 1); break; }
    }
    S.ticks++;

    /* latido del modo manual */
    bool fresh_dm; int8_t dir[NJ];
    portENTER_CRITICAL(&lock);
    fresh_dm = hb.dm && (now - hb.t_us) < DEADMAN_TIMEOUT_MS * 1000LL;
    for (int j = 0; j < NJ; j++) dir[j] = hb.dir[j];
    portEXIT_CRITICAL(&lock);
    if (!fresh_dm) S.jog_lock = false;
    bool quiere_jog = false;
    for (int j = 0; j < NJ; j++) if (fresh_dm && !S.jog_lock && S.en[j] && dir[j] != 0) quiere_jog = true;
    if (screw_activo()) quiere_jog = false;   /* la punta está dentro del tornillo */
    if ((S.mode == M_OFF || S.mode == M_HOLD) && quiere_jog) S.mode = M_JOG;

    for (int j = 0; j < NJ; j++) eje_tick(j, dt_us);

    switch (S.mode) {
    case M_HOMING: if (S.h_ref) ref_tick(); else mapeo_tick(); break;
    case M_MOVE:   modo_move(); break;
    case M_JOG:    modo_jog(fresh_dm, dir); break;
    case M_PULSE:  modo_pulse(); break;
    default: break;
    }

    aplicar_salidas();
    if ((S.ticks % 10) == 0) publicar();
}

static void control_task(void *arg)
{
    (void)arg;
    esp_task_wdt_add(NULL);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
        tick();
        esp_task_wdt_reset();
    }
}

/* ---------------------------------------------------------------- API pública */
void ctl_init(void)
{
    cmd_q = xQueueCreate(8, sizeof(cmd_t));
    evt_q = xQueueCreate(16, 160);
    memset(&S, 0, sizeof S);
    memset(&hb, 0, sizeof hb);
    S.mode = M_OFF;
    estop_req = false;
    S.tool_en = TOOL_ACTIVO_INIT;
    S.t_ant = now_us();
    servo_mover(SERVO_CERRADO_DEG);
    const bool en0[NJ] = MOTORES_INIT;
    for (int j = 0; j < NJ; j++) {
        S.en[j] = en0[j];
        for (int s = 0; s < 2; s++) {
            S.vel[j][s] = VEL_NOMINAL_DEG_S;
            S.t0[j][s] = S.arranque[j][s] = T0_NOMINAL_S;
        }
        /* antes de Home no hay referencia: q empieza en 0, como en el firmware principal */
        pub_mapa[j][4] = ajuste[j];
        pub_lim[j][0] = JC[j].q_min; pub_lim[j][1] = JC[j].q_max;
        S.fc[j][L_MIN] = hal_fc_raw(j, L_MIN);
        S.fc[j][L_MAX] = hal_fc_raw(j, L_MAX);
    }
    publicar();
}

void ctl_start(void)
{
    xTaskCreatePinnedToCore(control_task, "control", 6144, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

void ctl_cmd_jog(bool dm, int d1, int d2)
{
    portENTER_CRITICAL(&lock);
    hb.dm = dm;
    hb.dir[0] = (int8_t)(d1 > 0) - (int8_t)(d1 < 0);
    hb.dir[1] = (int8_t)(d2 > 0) - (int8_t)(d2 < 0);
    hb.t_us = now_us();
    portEXIT_CRITICAL(&lock);
}

static bool enviar(const cmd_t *c) { return xQueueSend(cmd_q, c, 0) == pdTRUE; }

bool ctl_cmd_home(void)  { cmd_t c = { .t = C_HOME }; return enviar(&c); }
bool ctl_cmd_ref(void)   { cmd_t c = { .t = C_REF }; return enviar(&c); }
void ctl_cmd_stop(void)  { cmd_t c = { .t = C_STOP }; enviar(&c); }
void ctl_cmd_off(void)   { cmd_t c = { .t = C_OFF }; enviar(&c); }
void ctl_cmd_clear(void) { cmd_t c = { .t = C_CLEAR }; enviar(&c); }

bool ctl_cmd_move(const float q[NJ], const float tgt_xyz[3])
{
    cmd_t c = { .t = C_MOVE };
    memcpy(c.q, q, sizeof c.q);
    memcpy(c.tgt, tgt_xyz, sizeof c.tgt);
    return enviar(&c);
}

bool ctl_cmd_pulse(int j)
{
    if (j < 0 || j >= NJ) return false;
    cmd_t c = { .t = C_PULSE, .j = j };
    return enviar(&c);
}

bool ctl_cmd_ajuste(int j, float pedido_deg, float medido_deg)
{
    if (j < 0 || j >= NJ || !isfinite(pedido_deg) || !isfinite(medido_deg)) return false;
    cmd_t c = { .t = C_AJUSTE, .j = j, .g = { pedido_deg, medido_deg } };
    return enviar(&c);
}

void ctl_get_mapa(int j, float out[5])
{
    portENTER_CRITICAL(&lock);
    for (int i = 0; i < 5; i++) out[i] = pub_mapa[j][i];
    portEXIT_CRITICAL(&lock);
}

void ctl_cmd_estop(void) { estop_req = true; }

bool ctl_cmd_motor(int m, bool on)
{
    if (m < 0 || m > NJ) return false;
    cmd_t c = { .t = C_MOTOR, .j = m, .g = { on ? 1.0f : 0.0f } };
    return enviar(&c);
}

bool ctl_motor_enabled(int m)
{
    if (m < 0 || m > NJ) return false;
    uint8_t en;
    portENTER_CRITICAL(&lock);
    en = pub.en;
    portEXIT_CRITICAL(&lock);
    return (en >> m) & 1;
}

bool ctl_cmd_tool(float pct)
{
    if (!(pct >= -100.0f && pct <= 100.0f)) return false;
    cmd_t c = { .t = C_TOOL, .g = { pct } };
    return enviar(&c);
}

bool ctl_cmd_servo(float deg)
{
    if (!(deg >= 0.0f && deg <= SERVO_GRADOS_MAX)) return false;
    cmd_t c = { .t = C_SERVO, .g = { deg } };
    return enviar(&c);
}

bool ctl_cmd_screw(float vueltas, float torque)
{
    if (!(vueltas >= 0.0f && vueltas <= 40.0f) || !(torque >= 0.0f && torque <= 1.5f)) return false;
    cmd_t c = { .t = C_SCREW, .g = { vueltas, torque } };
    return enviar(&c);
}

void ctl_get_limits(int j, float *qmin, float *qmax)
{
    portENTER_CRITICAL(&lock);
    *qmin = pub_lim[j][0]; *qmax = pub_lim[j][1];
    portEXIT_CRITICAL(&lock);
}

void ctl_get_status(ctl_status_t *s)
{
    portENTER_CRITICAL(&lock);
    *s = pub;
    portEXIT_CRITICAL(&lock);
}

bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms)
{
    char tmp[160];
    if (xQueueReceive(evt_q, tmp, pdMS_TO_TICKS(wait_ms)) != pdTRUE) return false;
    snprintf(buf, n, "%s", tmp);
    return true;
}
