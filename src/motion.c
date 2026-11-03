#include "motion.h"
#include <math.h>
#include <stdio.h>
#include "config.h"
#include "control.h"
#include "kinematics.h"

#define RAD2DEG (180.0f / (float)M_PI)
#define DEG2RAD ((float)M_PI / 180.0f)

static int g_elbow = ELBOW_DEFAULT;

int  motion_get_elbow(void) { return g_elbow; }
void motion_set_elbow(int e) { g_elbow = (e >= 0) ? 1 : -1; }

motion_res_t motion_goto_xyz(float x, float y, float z, char *msg, size_t n)
{
#if Z_CHECK
    if (!(fabsf(z - Z_FIJO_MM) <= Z_TOL_MM)) {             /* escrito asi para que NaN tampoco pase */
        snprintf(msg, n, "#ERROR: z = %.1f mm no es valida; la altura fija es %.1f +-%.1f mm.", z, Z_FIJO_MM, Z_TOL_MM);
        return MOTION_ERR_Z;
    }
#else
    (void)z;
#endif
    for (int j = 0; j < NJ; j++) {
        if (!ctl_motor_enabled(j)) {
            snprintf(msg, n, "#ERROR: J%d esta desactivado: ir a (x, y) necesita J1 y J2. Usa ang o activalo.", j + 1);
            return MOTION_ERR_MOTOR;
        }
    }
    float qmin[2], qmax[2];
    for (int j = 0; j < NJ; j++) ctl_get_limits(j, &qmin[j], &qmax[j]);
    float q[2]; int cod = 0;
    ik_status_t st = kin_ik(&KIN, x, y, g_elbow, qmin, qmax, q, &cod);
    if (st != IK_OK) {
        snprintf(msg, n, "#ERROR: no se puede ir a (%.1f, %.1f): %s. Alcance: %.1f a %.1f mm.", x, y, kin_ik_str(st),
                 fabsf(KIN.l1 - KIN.l2), KIN.l1 + KIN.l2);
        return MOTION_ERR_IK;
    }
    float tgt[3] = { x, y, z };
    if (!ctl_cmd_move(q, tgt)) {
        snprintf(msg, n, "#ERROR: cola de ordenes llena, intenta de nuevo.");
        return MOTION_ERR_COLA;
    }
    snprintf(msg, n, "#INFO: cinematica inversa (%.1f, %.1f, %.1f) -> q1 = %.2f deg, q2 = %.2f deg (codo %+d). Moviendo...",
             x, y, z, q[0] * RAD2DEG, q[1] * RAD2DEG, cod);
    return MOTION_OK;
}

motion_res_t motion_goto_ang(float a, float b, char *msg, size_t n)
{
    if (!isfinite(a) || !isfinite(b)) {
        snprintf(msg, n, "#ERROR: los angulos deben ser numeros.");
        return MOTION_ERR_LIMITES;
    }
    float q[2] = { a * DEG2RAD, b * DEG2RAD };
    for (int j = 0; j < NJ; j++) {
        if (!ctl_motor_enabled(j)) continue;           /* su ángulo se ignora: el eje no participa */
        float lo, hi; ctl_get_limits(j, &lo, &hi);
        if (q[j] < lo || q[j] > hi) {
            snprintf(msg, n, "#ERROR: q%d = %.1f deg esta fuera de sus limites (%.1f a %.1f deg).", j + 1,
                     (j == 0 ? a : b), lo * RAD2DEG, hi * RAD2DEG);
            return MOTION_ERR_LIMITES;
        }
    }
    float tgt[3] = { NAN, NAN, NAN };
    if (!ctl_cmd_move(q, tgt)) {
        snprintf(msg, n, "#ERROR: cola de ordenes llena, intenta de nuevo.");
        return MOTION_ERR_COLA;
    }
    snprintf(msg, n, "#INFO: moviendo a q1 = %.2f deg, q2 = %.2f deg...", a, b);
    return MOTION_OK;
}
