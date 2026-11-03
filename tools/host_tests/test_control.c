/* Pruebas del firmware de emergencia contra el robot simulado, con la misma interfaz de comandos que usa el panel. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kinematics.h"
#include "../../src/control.c"
#include "../sim/sim_core.h"
#include "motion.h"
#include "uros_cmd.h"

static int fallos;
#define CHECK(c, ...) do { if (!(c)) { printf("  >>> FALLO: " __VA_ARGS__); printf("\n"); fallos++; } else { printf("  ok: " __VA_ARGS__); printf("\n"); } } while (0)

static char LOG[60000]; static size_t LOG_N;
static void anotar(const char *s) { size_t n = strlen(s); if (LOG_N + n + 2 < sizeof LOG) { memcpy(LOG + LOG_N, s, n); LOG_N += n; LOG[LOG_N++] = '\n'; LOG[LOG_N] = 0; } }
static void limpiar(void) { LOG_N = 0; LOG[0] = 0; }
static bool hay(const char *t) { return strstr(LOG, t) != NULL; }

static void paso(void)
{
    g_now += 1000; planta_1ms(); tick();
    char b[160];
    while (ctl_event_pop(b, sizeof b, 0)) anotar(b);
}
static void correr(double s) { for (long k = 0; k < (long)(s * 1000); k++) paso(); }
static void cmd(const char *l) { char r[240]; uros_cmd_exec(l, r, sizeof r); anotar(r); paso(); }
static void inicio(const plant_cfg_t *c, double q1, double q2) { plant_reset(c, q1, q2); ctl_init(); limpiar(); paso(); }
static bool ocupado(void) { return S.mode == M_HOMING || S.mode == M_MOVE || S.mode == M_PULSE; }
static bool esperar(double s) { for (long k = 0; k < (long)(s * 1000); k++) { paso(); if (!ocupado()) return true; } return false; }
static bool home_ok(void) { cmd("home"); return esperar(240) && S.homed[0] && S.homed[1]; }
static void ang(float a, float b) { char l[64]; snprintf(l, sizeof l, "ang %g %g", a, b); cmd(l); esperar(30); }
static void jog(double s, int dm, int d1, int d2) { for (long k = 0; k < (long)(s * 1000); k++) { if (k % 40 == 0) ctl_cmd_jog(dm, d1, d2); paso(); } }

static double vel_real(int j, int sentido)
{
    plant_reset(NULL, 0, 0);
    g_stby = true;
    hal_motor_drive(j, sentido * HOME_DUTY);
    for (int k = 0; k < 1000; k++) planta_1ms();
    const double a = P[j].salida_deg;
    for (int k = 0; k < 500; k++) planta_1ms();
    hal_motor_free(j);
    return fabs(P[j].salida_deg - a) / 0.5;
}

/* Lo que hace el usuario tras medir el robot: pide un ángulo, lo mide y lo escribe en el comando ajuste. */
static void ajustar_como_usuario(void)
{
    inicio(NULL, 10, -30); home_ok();
    cmd("ref"); esperar(60);          /* desde el punto de Home: es el recorrido más largo que se puede medir */
    ang(60, 40);
    char b[64];
    snprintf(b, sizeof b, "ajuste 1 60 %.1f", P[0].salida_deg); cmd(b);
    snprintf(b, sizeof b, "ajuste 2 40 %.1f", P[1].salida_deg); cmd(b);
    printf("Ajuste: J1 %.4f, J2 %.4f\n", ajuste[0], ajuste[1]);
}

int main(void)
{
    ajustar_como_usuario();

    double q0;
    printf("== Sin Home no se mueve a un punto, igual que el firmware principal ==\n");
    inicio(NULL, 10, -30);
    cmd("goto 100 -100");
    CHECK(S.mode == M_OFF && hay("#ERROR") && !g_stby, "goto sin Home se rechaza");
    limpiar(); cmd("ref"); CHECK(S.mode == M_OFF && hay("#ERROR"), "ref sin Home se rechaza");
    limpiar(); cmd("ajuste 1 60 61"); CHECK(hay("#ERROR"), "ajuste sin Home se rechaza");

    printf("== Home mide el robot y vuelve a donde estaba ==\n");
    inicio(NULL, 10, -30);
    CHECK(home_ok(), "termina Home");
    printf("  (tiempo de Home: %.0f s)\n", g_now * 1e-6);
    CHECK(S.mode == M_HOLD && hay("#LLEGO"), "queda en HOLD y avisa, como en el firmware principal");
    /* Volver es aproximado: no se sabe cuánto hay del final de carrera al punto de Home. Lo que importa es saber dónde quedó. */
    CHECK(fabs(P[0].salida_deg - 10) < 3 && fabs(P[1].salida_deg + 30) < 3, "vuelve cerca de donde estaba (%.2f, %.2f)", P[0].salida_deg, P[1].salida_deg);
    CHECK(fabs(S.q[0] - P[0].salida_deg) < 1 && fabs(S.q[1] - P[1].salida_deg) < 1, "sabe dónde quedó (error %.2f, %.2f grados)", S.q[0] - P[0].salida_deg, S.q[1] - P[1].salida_deg);
    CHECK(g_phys_max_exceso < 5.0, "no llega al tope duro (se pasa %.2f grados del final de carrera)", g_phys_max_exceso);
    CHECK(hay("eje 1 medido") && hay("eje 2 medido"), "informa lo medido");
    limpiar(); cmd("mapa"); CHECK(hay("#MAPA: J1") && hay("J2"), "mapa muestra lo medido");
    float mv[NJ][2], mt[NJ][2];
    memcpy(mv, S.vel, sizeof mv); memcpy(mt, S.t0, sizeof mt);
    for (int j = 0; j < NJ; j++) for (int s = 0; s < 2; s++) {
        const double v = vel_real(j, s ? 1 : -1);
        CHECK(fabs(mv[j][s] - v) / v < 0.0075, "J%d sentido %d: velocidad medida %.1f, real %.1f grados/s", j + 1, s, mv[j][s], v);
        CHECK(mt[j][s] > 0.03f && mt[j][s] < 0.15f, "J%d: tiempo perdido al arrancar %.0f ms", j + 1, mt[j][s] * 1000);
    }
    inicio(NULL, -110, 0); CHECK(home_ok(), "arranca sobre el final de carrera MIN");
    inicio(NULL, 121, 0);  CHECK(home_ok(), "arranca sobre el final de carrera MAX");
    inicio(NULL, 10, -30); home_ok(); ang(50, 20); cmd("ref"); esperar(60);
    CHECK(S.mode == M_HOLD && fabs(P[0].salida_deg - JC[0].q_home * R2D) < 0.2 && fabs(P[1].salida_deg - JC[1].q_home * R2D) < 0.2,
          "ref queda en el punto de Home, donde se mide con transportador (%.2f, %.2f)", P[0].salida_deg, P[1].salida_deg);

    printf("== Home detecta problemas ==\n");
    plant_cfg_t c; memset(&c, 0, sizeof c); c.sentido[0] = -1;
    inicio(&c, 10, -30); cmd("home"); esperar(240);
    CHECK(S.mode == M_FAULT && !S.homed[0] && hay("sentido"), "motor al revés: falla y lo dice");
    correr(0.3);
    CHECK(P[0].mode != 1 && P[1].mode != 1 && !g_stby, "los motores quedan apagados");
    memset(&c, 0, sizeof c); c.fc_roto[1][0] = true;
    inicio(&c, 10, -30); cmd("home"); esperar(240);
    CHECK(S.mode == M_FAULT && S.fault == F_HOME_TIMEOUT && hay("J2"), "final de carrera roto: falla al agotar el tiempo");
    memset(&c, 0, sizeof c); c.fc_pegado[0][0] = true; c.fc_pegado[0][1] = true;
    inicio(&c, 10, -30); correr(0.5);
    CHECK(S.mode == M_FAULT && S.fault == F_DIAG_FC, "MIN y MAX a la vez: falla");
    inicio(NULL, 10, -30); g_lock[1] = true; cmd("home"); esperar(240);
    CHECK(S.mode == M_FAULT, "eje trabado: falla");
    inicio(NULL, 10, -30); cmd("home"); correr(1.0); cmd("stop"); correr(0.5);
    CHECK(S.mode == M_OFF && !S.homed[1] && !g_stby, "detener durante Home: no queda referenciado");

    printf("== Punto a punto ==\n");
    inicio(NULL, 10, -30); home_ok();
    limpiar(); cmd("goto 200 200"); CHECK(S.mode == M_HOLD && hay("#ERROR"), "fuera de alcance se rechaza");
    limpiar(); cmd("elbow -1"); cmd("goto 141.4 -50"); CHECK(S.mode == M_MOVE, "goto entra en MOVE"); esperar(30);
    CHECK(S.mode == M_HOLD && hay("#LLEGO: objetivo") && hay("estimado"), "llega, avisa y dice que es estimado");
    CHECK(plant_error_mm(141.4, -50) < 1.5, "llega al punto (error %.2f mm)", plant_error_mm(141.4, -50));
    cmd("goto 0 -150"); esperar(30);
    CHECK(plant_error_mm(0, -150) < 1.5, "y a otro (error %.2f mm)", plant_error_mm(0, -150));
    cmd("goto 0 -150"); esperar(30);
    CHECK(plant_error_mm(0, -150) < 1.5, "repetir el mismo punto no lo aparta (error %.2f mm)", plant_error_mm(0, -150));
    ang(0, -90);
    CHECK(fabs(P[0].salida_deg) < 1 && fabs(P[1].salida_deg + 90) < 1, "ang llega (%.2f, %.2f)", P[0].salida_deg, P[1].salida_deg);
    limpiar(); cmd("ang 900 0"); CHECK(hay("#ERROR"), "ángulo fuera de límites se rechaza");
    inicio(NULL, 10, -30); home_ok();
    sw_max_deg[0] -= 60;      /* el rango real es mucho más corto: el eje se encuentra el MAX en un movimiento */
    limpiar(); cmd("ang 90 -90"); esperar(30);
    CHECK(S.mode == M_FAULT && S.fault == F_LIMIT && !S.homed[0], "final de carrera inesperado en un movimiento: falla");
    inicio(NULL, 10, -30); home_ok();
    sw_min_deg[0] = P[0].salida_deg + 0.5;      /* el eje está sobre el final de carrera MIN, aunque cree estar en otro lado */
    correr(0.02);
    q0 = P[0].salida_deg;
    limpiar(); cmd("ang -90 -90"); esperar(30);
    CHECK(S.mode == M_FAULT && S.fault == F_LIMIT && fabs(P[0].salida_deg - q0) < 1.0, "no empuja contra un tope que ya estaba pulsado (se movió %.2f grados)", P[0].salida_deg - q0);

    printf("== Detener y apagar ==\n");
    inicio(NULL, 10, -30); home_ok();
    cmd("goto 141 -50"); correr(0.3);
    double q_antes = P[0].salida_deg;
    cmd("stop"); correr(0.3);
    CHECK(S.mode == M_HOLD && fabs(P[0].salida_deg - q_antes) < 4 && fabs(S.q[0] - P[0].salida_deg) < 3, "stop en movimiento: frena y sabe aproximadamente dónde quedó (error %.2f grados)", S.q[0] - P[0].salida_deg);
    CHECK(P[0].mode == 2 && g_stby, "en HOLD los ejes quedan frenados");
    cmd("off"); correr(0.05);
    CHECK(S.mode == M_OFF && !g_stby && P[0].mode == 0 && !S.homed[0] && !S.homed[1], "off: motores libres y pierde el Home, porque nadie mediría si mueven el brazo");
    limpiar(); cmd("ang 60 40"); correr(0.01);
    CHECK(S.mode == M_OFF && hay("primero ejecuta HOME"), "tras off no acepta movimientos hasta repetir Home");

    printf("== Hombre muerto ==\n");
    inicio(NULL, 10, -30); home_ok();
    q0 = P[0].salida_deg;
    jog(0.6, 0, 1, 0);
    CHECK(fabs(P[0].salida_deg - q0) < 0.01 && S.mode == M_HOLD, "sin dead-man no se mueve");
    jog(1.0, 1, 1, 0);
    CHECK(S.mode == M_JOG && P[0].salida_deg > q0 + 5.0, "con dead-man se mueve (%.1f grados)", P[0].salida_deg - q0);
    const double t_ult = g_now;
    double q_suelto = P[0].salida_deg;
    while (S.mode == M_JOG && g_now - t_ult < 2000000) paso();       /* deja de llegar el latido */
    CHECK(S.mode == M_HOLD && (g_now - t_ult) < (DEADMAN_TIMEOUT_MS + JOG_FRENO_MS + 80) * 1000LL, "sin latido se detiene solo (en %.0f ms)", (g_now - t_ult) * 1e-3);
    CHECK(fabs(P[0].salida_deg - q_suelto) < 25.0 && P[0].mode == 2, "y queda frenado");
    CHECK(fabs(S.q[0] - P[0].salida_deg) < 1.5, "un toque largo se estima bien (error %.2f grados)", S.q[0] - P[0].salida_deg);
    q0 = P[0].salida_deg; double est0 = S.q[0];
    for (int i = 0; i < 6; i++) { jog(0.2, 1, -1, 0); jog(0.5, 0, 0, 0); }
    CHECK(fabs((S.q[0] - est0) - (P[0].salida_deg - q0)) < 2.0, "6 toques cortos se estiman bien (real %.1f, estimado %.1f grados)", P[0].salida_deg - q0, S.q[0] - est0);
    limpiar(); jog(0.3, 1, 1, 1); ctl_cmd_estop(); correr(0.01);
    CHECK(S.mode == M_FAULT && S.fault == F_ESTOP, "paro durante manual");
    jog(0.3, 1, 1, 0); correr(0.2);
    CHECK(S.mode == M_FAULT && P[0].mode != 1, "con el paro activo el dead-man no mueve");
    jog(0.3, 0, 0, 0);
    cmd("clear"); q0 = P[0].salida_deg;
    jog(0.3, 1, 1, 0);
    CHECK(S.mode == M_JOG && P[0].salida_deg > q0, "tras rearmar, soltar y volver a pulsar, mueve");
    jog(0.5, 0, 0, 0);
    /* Manual contra los límites y los topes */
    inicio(NULL, 10, -30); home_ok();
    jog(8.0, 1, 1, 0);
    CHECK(S.q[0] <= limite_alto(0) + 0.5 && !S.fc[0][L_MAX] && g_phys_max_exceso < 5.0, "con Home, se para en el límite por software (%.1f grados)", S.q[0]);
    jog(0.5, 0, 0, 0);
    inicio(NULL, 10, -30);
    jog(8.0, 1, 1, 0);
    CHECK(S.fc[0][L_MAX] && P[0].mode != 1 && g_phys_max_exceso < 5.0, "sin Home, se para en el final de carrera (se pasa %.2f grados)", g_phys_max_exceso);
    jog(0.5, 0, 0, 0);

    printf("== Paro de emergencia ==\n");
    inicio(NULL, 10, -30); home_ok();
    cmd("servo abrir"); cmd("ang 60 40");
    correr(0.3); ctl_cmd_estop(); paso();
    CHECK(S.mode == M_FAULT && S.fault == F_ESTOP && P[0].mode == 2 && g_servo_deg == SERVO_CERRADO_DEG, "frena y cierra el servo");
    correr(0.3); CHECK(!g_stby, "drivers apagados");
    limpiar(); cmd("ang 0 -90"); correr(0.1); CHECK(S.mode == M_FAULT && hay("#ERROR") && P[0].mode != 1, "enclavado: no acepta movimiento");
    limpiar(); cmd("servo abrir"); correr(0.01); CHECK(g_servo_deg == SERVO_CERRADO_DEG && hay("#ERROR"), "no abre el servo con paro");
    cmd("clear"); correr(0.01);
    CHECK(S.mode == M_OFF && S.homed[0] && S.homed[1], "clear lo rearma y conserva el Home");
    inicio(NULL, 10, -30); cmd("home"); correr(1); ctl_cmd_estop(); correr(0.01); cmd("clear"); correr(0.01);
    CHECK(!S.homed[0] && !S.homed[1], "un paro durante Home no deja referencia");

    printf("== Motores ==\n");
    inicio(NULL, 10, -30); home_ok();
    limpiar(); cmd("motor 2 off"); correr(0.01);
    CHECK(!S.en[1] && !S.homed[1] && P[1].mode == 0, "desactivar J2: queda libre y sin referencia");
    limpiar(); cmd("goto 120 -80"); CHECK(hay("desactivado"), "goto exige los dos ejes");
    double q2_quieto = P[1].salida_deg;
    cmd("ang 30 500"); esperar(30);
    CHECK(fabs(P[0].salida_deg - 30) < 1.5 && fabs(P[1].salida_deg - q2_quieto) < 0.01, "ang mueve solo los activos (J1 %.2f)", P[0].salida_deg);
    cmd("motor 2 on"); correr(0.01); limpiar(); cmd("ang 30 0"); correr(0.01);
    CHECK(hay("HOME") && S.mode != M_MOVE, "al reactivarlo pide Home");
    inicio(NULL, 10, -30); cmd("motor 1 off"); correr(0.01); cmd("home"); esperar(240);
    CHECK(S.homed[1] && !S.homed[0] && S.mode == M_HOLD, "Home solo con los activos");
    inicio(NULL, 10, -30); home_ok(); cmd("ang 60 40"); correr(0.1); cmd("motor 1 off"); esperar(30);
    CHECK(S.mode == M_HOLD && P[0].mode == 0, "desactivar durante un movimiento: ese eje para y el otro termina");

    printf("== M3 y atornillado por tiempo ==\n");
    inicio(NULL, 10, -30);
    limpiar(); cmd("tool 40"); correr(0.01); CHECK(hay("desactivado"), "M3 arranca desactivado, como en el firmware principal");
    cmd("motor 3 on"); cmd("tool 40"); correr(0.3);
    CHECK(g_tool_duty > 200 && g_stby && S.mode == M_OFF, "gira sin Home y con el brazo apagado");
    correr(TOOL_MAX_ON_MS * 1e-3 + 0.5);
    CHECK(g_tool_duty == 0 && !g_stby, "sin renovar la orden se apaga solo");
    inicio(NULL, 10, -30); home_ok(); cmd("motor 3 on");
    cmd("servo abrir"); correr(0.5); cmd("servo cerrar"); correr(0.5);
    limpiar(); cmd("screw"); correr(0.1);
    CHECK(hay("por tiempo") && g_tool_duty > 0, "atornilla sin encoder");
    limpiar(); cmd("ang 0 -90"); correr(0.01); CHECK(hay("atornillando") && S.mode == M_HOLD, "el brazo no se mueve mientras atornilla");
    jog(0.3, 1, 1, 0); CHECK(S.mode == M_HOLD, "ni con el dead-man");
    jog(0.1, 0, 0, 0);
    limpiar(); for (long k = 0; k < 40000 && !hay("#TORNILLO"); k++) paso();
    CHECK(hay("#TORNILLO: sin_medir") && hay("avance=100%"), "al terminar dice que no midió");
    CHECK(g_tool_max <= 235, "nunca pasa del PWM que da el torque pedido (%d)", g_tool_max);
    CHECK(g_m3_torque_max > 0.3 && g_m3_torque_max < 0.45, "en la planta aprieta sin pasarse: %.3f N·m", g_m3_torque_max);
    CHECK((g_m3_th - 0) / (2 * M_PI) > 12.0, "en la planta alcanza las vueltas (%.1f)", g_m3_th / (2 * M_PI));
    correr(0.3); CHECK(g_tool_duty == 0, "y M3 se detiene");
    inicio(NULL, 10, -30); home_ok(); cmd("motor 3 on");
    cmd("servo abrir"); correr(0.5); cmd("servo cerrar"); correr(0.5);
    g_screw_mode = SIM_TORNILLO_ATASCO; g_m3_th_tornillo = g_m3_th;
    limpiar(); cmd("screw"); for (long k = 0; k < 40000 && !hay("#TORNILLO"); k++) paso();
    CHECK(hay("#TORNILLO: sin_medir"), "un tornillo atascado no se detecta: el firmware lo dice igual, sin_medir");
    limpiar(); cmd("screw"); correr(1); cmd("stop"); correr(0.1);
    CHECK(hay("#TORNILLO: cancelado") && g_tool_duty == 0, "stop lo cancela");
    limpiar(); cmd("screw 50 0.4"); CHECK(hay("#ERROR"), "screw fuera de rango");

    printf("== Recorrer los ejes corrige si cambian las condiciones ==\n");
    inicio(NULL, 10, -30); home_ok();
    cmd("ref"); esperar(60);
    PC.v_fac = 0.97; PC.tc_fac[0] = PC.tc_fac[1] = 1.10;
    ang(-50, -70);
    const double sin_corregir = fabs(P[0].salida_deg + 50);
    limpiar(); cmd("ref"); CHECK(S.mode == M_HOMING, "ref ocupa al robot"); esperar(60);
    CHECK(S.mode == M_HOLD && hay("velocidad") && hay("#LLEGO"), "ref vuelve a medir y avisa con #LLEGO");
    if (S.mode != M_HOLD) printf("%s\n", LOG);
    ang(-50, -70);
    CHECK(fabs(P[0].salida_deg + 50) < 2.0 && sin_corregir > 2.0, "error en J1: %.2f grados sin corregir, %.2f después", sin_corregir, fabs(P[0].salida_deg + 50));
    inicio(NULL, 10, -30); home_ok();
    PC.v_fac = 0.97; PC.tc_fac[0] = PC.tc_fac[1] = 1.10;
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_HOLD, "que el eje vaya más lento no es perderse: el brazo sigue donde estaba");
    inicio(NULL, 10, -30); home_ok();
    ang(0, -90);
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_HOLD && hay("#REF J1") && hay("#REF J2"), "ref informa la diferencia");
    ang(0, -90);
    P[0].salida_deg += 40; P[0].th += 40.0 / 360 * 2 * M_PI * NGEAR;     /* alguien movió el brazo */
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_FAULT && S.fault == F_PERDIDO && !S.homed[0] && hay("perdio"), "si el brazo se perdió 40 grados, falla");
    inicio(NULL, 10, -30); home_ok();
    ang(0, -90);
    g_lock[0] = true;
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_FAULT && !S.homed[0], "si el eje se trabó, falla");
    inicio(NULL, 10, -30); home_ok();
    g_lock[0] = true; ang(50, -90);
    CHECK(S.mode == M_HOLD && hay("#LLEGO"), "un eje trabado NO se nota en un movimiento: sin encoder dice que llegó");
    inicio(NULL, 10, -30); home_ok(); cmd("ref"); correr(1); ctl_cmd_estop(); correr(0.01); cmd("clear"); correr(0.01);
    CHECK(S.homed[0] && S.homed[1], "un paro durante ref conserva el Home");

    printf("== Pruebas de sentido y ajuste ==\n");
    inicio(NULL, 10, -30); home_ok();
    float q1_a = S.q[0], q2_a = S.q[1];
    limpiar(); cmd("pulse 2"); CHECK(S.mode == M_PULSE, "pulse"); esperar(5);
    CHECK(fabs(S.q[0] - q1_a) < 0.01, "pulso en J2 no cambia J1 (%.2f a %.2f)", q1_a, S.q[0]);
    CHECK(S.q[1] > q2_a + 5.0 && fabs(S.q[1] - P[1].salida_deg) < 1.5, "J2 estima el pulso (estimado %.2f, real %.2f)", S.q[1], P[1].salida_deg);
    CHECK(S.mode == M_OFF && !g_stby && hay("#PULSO") && hay("mira el brazo"), "termina apagado y explica cómo leerlo");
    limpiar(); cmd("ajuste 1 60 900"); correr(0.01); CHECK(hay("revisa"), "ajuste absurdo se rechaza");
    limpiar(); cmd("ajuste 1 60"); CHECK(hay("#ERROR"), "ajuste incompleto");
    limpiar(); cmd("ajuste 1 -100 -99"); correr(0.01); CHECK(hay("30 grados"), "ajuste demasiado cerca del punto de Home");
    limpiar(); cmd("ctrl pid"); CHECK(hay("sin encoders"), "ctrl explica que no aplica");
    limpiar(); cmd("gain"); CHECK(hay("sin encoders"), "gain explica que no aplica");
    limpiar(); cmd("rstform pos"); CHECK(hay("sin encoders"), "rstform explica que no aplica");
    ctl_status_t st; correr(0.05); ctl_get_status(&st);
    CHECK(st.ctrl == CTRL_POR_TIEMPO, "el estado dice que va por tiempo");

    printf("== ref: la distancia fija hasta el final de carrera no es perderse ==\n");
    { plant_cfg_t pesado = {0}; pesado.j_fac[0] = pesado.j_fac[1] = 3.0;
      inicio(&pesado, 10, -30); home_ok();
      for (int i = 0; i < 3 && S.mode != M_FAULT; i++) { limpiar(); cmd("ref"); esperar(60); if (S.mode != M_FAULT) ang(60 - i * 10, -40 + i * 5); }
      CHECK(S.mode != M_FAULT, "con el triple de inercia en la carga, tres ref seguidos no dan falla");
    }
    inicio(NULL, 10, -30); home_ok(); cmd("ref"); esperar(60); ang(30, 30);
    P[0].salida_deg += 6; P[0].th += 6.0 / 360 * 2 * M_PI * NGEAR;
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_HOLD && hay("#REF J1"), "un empujon de 6 grados queda dentro de los 8 permitidos");
    inicio(NULL, 10, -30); home_ok(); cmd("ref"); esperar(60); ang(30, 30);
    P[0].salida_deg += 12; P[0].th += 12.0 / 360 * 2 * M_PI * NGEAR;
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_FAULT && S.fault == F_PERDIDO, "un empujon de 12 grados se detecta");
    inicio(NULL, 10, -30); home_ok(); ang(30, 30);
    P[0].salida_deg += 30; P[0].th += 30.0 / 360 * 2 * M_PI * NGEAR;
    limpiar(); cmd("ref"); esperar(60);
    CHECK(S.mode == M_FAULT && S.fault == F_PERDIDO, "el primer ref tras Home tambien detecta un empujon grande");

    printf("== ajuste despues de un ref que cambio la velocidad ==\n");
    { const float guardado = ajuste[0];
      inicio(NULL, 10, -30); home_ok(); PC.v_fac = 0.97;
      cmd("ref"); esperar(60); ang(60, 40);
      char l[64]; snprintf(l, sizeof l, "ajuste 1 60 %.3f", P[0].salida_deg); cmd(l); correr(0.01);
      cmd("ref"); esperar(60); ang(60, 40);
      CHECK(fabs(P[0].salida_deg - 60) < 1.5, "tras ajuste y ref, ang 60 llega a %.2f", P[0].salida_deg);
      ajuste[0] = guardado;
    }

    printf("== Numeros que no son numeros ==\n");
    inicio(NULL, 10, -30); home_ok();
    ang(10, 20);
    limpiar(); cmd("ang nan 0"); esperar(5);
    CHECK(hay("#ERROR") && isfinite(S.q[0]) && isfinite(S.q[1]), "ang nan no toca la estimacion");
    ang(30, 30);
    CHECK(fabs(S.q[0] - P[0].salida_deg) < 1.5, "tras ang nan el brazo sigue donde se cree (estimado %.2f, real %.2f)", S.q[0], P[0].salida_deg);
    cmd("motor 3 on");
    limpiar(); cmd("tool nan"); correr(0.1);
    CHECK(hay("#ERROR") && S.tool_duty == 0.0f, "tool nan se rechaza y M3 sigue en 0");
    limpiar(); cmd("ajuste 1 60 nan"); correr(0.01);
    CHECK(hay("#ERROR") && isfinite(ajuste[0]) && isfinite(S.vel[0][0]) && isfinite(S.q[0]), "ajuste con medido nan no toca la calibracion");
    limpiar(); cmd("ajuste 1 nan 40"); correr(0.01);
    CHECK(hay("#ERROR") && isfinite(ajuste[0]) && isfinite(S.vel[0][0]) && isfinite(S.q[0]), "ajuste con pedido nan no toca la calibracion");

    printf("\n%s\n", fallos ? "HAY FALLOS" : "TODO OK");
    if (fallos) printf("%d FALLOS\n", fallos);
    return fallos != 0;
}
