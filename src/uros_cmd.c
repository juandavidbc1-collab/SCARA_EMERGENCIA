#include "uros_cmd.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "control.h"
#include "kinematics.h"
#include "motion.h"

#define RAD2DEG (180.0f / (float)M_PI)

static void cmd_status(char *reply, size_t n)
{
    ctl_status_t s; ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    snprintf(reply, n, "#INFO: [%s] q1=%.2f q2=%.2f deg x=%.1f y=%.1f mm home J1:%d J2:%d motores J1:%d J2:%d M3:%d falla:%s",
             ctl_mode_str(s.mode), s.q[0] * RAD2DEG, s.q[1] * RAD2DEG, x, y, s.homed[0], s.homed[1],
             s.en & 1, (s.en >> 1) & 1, (s.en >> 2) & 1,
             s.fault == F_NONE ? "ninguna" : ctl_fault_str(s.fault));
}

bool uros_cmd_exec(const char *line, char *reply, size_t n)
{
    char l[96];
    size_t k = 0;
    while (*line && (unsigned char)*line <= ' ') line++;                 /* espacios iniciales */
    for (; *line && k < sizeof l - 1; line++) l[k++] = (char)tolower((unsigned char)*line);
    l[k] = 0;
    while (k && (unsigned char)l[k - 1] <= ' ') l[--k] = 0;              /* espacios finales / \n */

    float f[6]; int j;
    if (!strcmp(l, "home")) {
        if (ctl_cmd_home()) { snprintf(reply, n, "#OK: home solicitado"); return true; }
        snprintf(reply, n, "#ERROR: home rechazado (falla activa o movimiento en curso)"); return false;
    }
    if (!strcmp(l, "stop"))  { ctl_cmd_stop();  snprintf(reply, n, "#OK: detenido (mantiene posicion)"); return true; }
    if (!strcmp(l, "off"))   { ctl_cmd_off();   snprintf(reply, n, "#OK: motores apagados"); return true; }
    if (!strcmp(l, "clear")) { ctl_cmd_clear(); snprintf(reply, n, "#OK: falla borrada"); return true; }
    if (!strcmp(l, "status")) { cmd_status(reply, n); return true; }
    if (!strcmp(l, "estop")) { ctl_cmd_estop(); snprintf(reply, n, "#OK: paro de emergencia solicitado"); return true; }
    {
        char w[8];
        if (sscanf(l, "motor %d %7s", &j, w) == 2) {
            bool on = !strcmp(w, "on");
            if ((on || !strcmp(w, "off")) && j >= 1 && j <= NJ + 1 && ctl_cmd_motor(j - 1, on)) {
                snprintf(reply, n, "#OK: motor %d %s", j, on ? "activar" : "desactivar");
                return true;
            }
            snprintf(reply, n, "#ERROR: usa motor 1|2|3 on|off"); return false;
        }
    }
    if (!strncmp(l, "ctrl", 4) || !strncmp(l, "rst", 3) || !strncmp(l, "gain", 4)) {
        snprintf(reply, n, "#ERROR: sin encoders no hay PID ni RST: los movimientos van por tiempo. Usa ajuste.");
        return false;
    }
    if (!strcmp(l, "ref")) {
        if (ctl_cmd_ref()) { snprintf(reply, n, "#OK: recorriendo los ejes de tope a tope"); return true; }
        snprintf(reply, n, "#ERROR: cola de ordenes llena"); return false;
    }
    if (sscanf(l, "ajuste %d %f %f", &j, &f[0], &f[1]) == 3) {
        if (j >= 1 && j <= NJ && ctl_cmd_ajuste(j - 1, f[0], f[1])) { snprintf(reply, n, "#OK: ajuste del eje %d solicitado", j); return true; }
        snprintf(reply, n, "#ERROR: usa ajuste eje pedido medido, con el eje 1 o 2"); return false;
    }
    if (!strcmp(l, "mapa")) {
        float a[5], b[5];
        ctl_get_mapa(0, a); ctl_get_mapa(1, b);
        snprintf(reply, n, "#MAPA: J1 v-=%.1f v+=%.1f t-=%.0f t+=%.0f aj=%.4f | J2 v-=%.1f v+=%.1f t-=%.0f t+=%.0f aj=%.4f",
                 a[0], a[1], a[2] * 1000.0f, a[3] * 1000.0f, a[4], b[0], b[1], b[2] * 1000.0f, b[3] * 1000.0f, b[4]);
        return true;
    }
    if (!strcmp(l, "servo abrir") || !strcmp(l, "servo cerrar") || sscanf(l, "servo %f", &f[0]) == 1) {
        float deg = !strcmp(l, "servo abrir") ? SERVO_ABIERTO_DEG : !strcmp(l, "servo cerrar") ? SERVO_CERRADO_DEG : f[0];
        if (ctl_cmd_servo(deg)) { snprintf(reply, n, "#OK: servo a %.0f grados", deg); return true; }
        snprintf(reply, n, "#ERROR: servo va de 0 a %.0f grados (o abrir / cerrar)", SERVO_GRADOS_MAX); return false;
    }
    if (!strcmp(l, "screw") || sscanf(l, "screw %f %f", &f[0], &f[1]) == 2) {
        float v = !strcmp(l, "screw") ? 0.0f : f[0], t = !strcmp(l, "screw") ? 0.0f : f[1];
        if (ctl_cmd_screw(v, t)) { snprintf(reply, n, "#OK: atornillado solicitado"); return true; }
        snprintf(reply, n, "#ERROR: usa screw [vueltas 0-40, torque 0-1.5 Nm]"); return false;
    }
    if (sscanf(l, "tool %f", &f[0]) == 1) {
        if (ctl_cmd_tool(f[0])) { snprintf(reply, n, "#OK: M3 a %.0f %%", f[0]); return true; }
        snprintf(reply, n, "#ERROR: tool va de -100 a 100 %%"); return false;
    }
    if (!strcmp(l, "help")) {
        snprintf(reply, n, "#INFO: home ref stop off clear estop goto ang elbow pulse status mapa | ajuste j pedido medido | motor n on/off | tool pct | servo g/abrir/cerrar | screw [v t]");
        return true;
    }
    if (sscanf(l, "goto %f %f %f", &f[0], &f[1], &f[2]) == 3) return motion_goto_xyz(f[0], f[1], f[2], reply, n) == MOTION_OK;
    if (sscanf(l, "goto %f %f", &f[0], &f[1]) == 2)            return motion_goto_xyz(f[0], f[1], Z_FIJO_MM, reply, n) == MOTION_OK;
    if (sscanf(l, "ang %f %f", &f[0], &f[1]) == 2)             return motion_goto_ang(f[0], f[1], reply, n) == MOTION_OK;
    if (sscanf(l, "elbow %d", &j) == 1) {
        motion_set_elbow(j);
        snprintf(reply, n, "#OK: codo preferido = %+d", motion_get_elbow());
        return true;
    }
    if (sscanf(l, "pulse %d", &j) == 1) {
        if (ctl_cmd_pulse(j - 1)) { snprintf(reply, n, "#OK: pulso en el eje %d", j); return true; }
        snprintf(reply, n, "#ERROR: pulso rechazado (eje 1 o 2, y sin movimiento en curso)"); return false;
    }
    snprintf(reply, n, "#ERROR: comando desconocido: %.40s  (usa 'help')", l);
    return false;
}
