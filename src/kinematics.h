/*
 * CINEMÁTICA SCARA PLANAR (2 articulaciones rotacionales, z fija)
 *
 * Convención (documentar igual en Notion):
 *   - q1: ángulo del eslabón 1 respecto al eje +X de la base, positivo antihorario.
 *   - q2: ángulo del eslabón 2 RELATIVO al eslabón 1, positivo antihorario.
 *   - Posición del efector:
 *         x = L1*cos(q1) + L2*cos(q1+q2)
 *         y = L1*sin(q1) + L2*sin(q1+q2)
 *   - z no depende de q1 ni de q2 (el eje prismático no se usa por ahora).
 *
 * Cinemática inversa (geométrica, ley de cosenos):
 *         r2 = x^2 + y^2
 *         c2 = (r2 - L1^2 - L2^2) / (2*L1*L2)         -> existe solución solo si |c2| <= 1
 *         q2 = +/- acos(c2)                            -> dos soluciones: "codo" (+) o (-)
 *         q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2))
 *
 * Singularidades: det(J) = L1*L2*sin(q2). Se anula en q2 = 0 (brazo extendido, borde exterior
 * del espacio de trabajo) y en q2 = +/-180° (brazo plegado, borde interior). Cerca de ellas
 * una velocidad pequeña en el espacio de la tarea exige velocidades articulares enormes y se
 * pierde un grado de libertad. Por eso ik() RECHAZA puntos con |sin(q2)| < sin(SING_MARGIN).
 *
 * Código en C puro, sin dependencias de hardware: se prueba en el PC (tools/host_tests).
 */
#pragma once
#include <stdbool.h>

typedef enum {
    IK_OK = 0,
    IK_INALCANZABLE,     /* fuera del anillo L1-L2 .. L1+L2 */
    IK_SINGULAR,         /* demasiado cerca de q2 = 0 o 180° */
    IK_LIMITES,          /* ninguna de las 2 soluciones respeta los límites articulares */
} ik_status_t;

typedef struct {
    float l1, l2;            /* longitudes de los eslabones [mm] */
    float sing_margin;       /* margen de singularidad [rad] */
} kin_cfg_t;

/* Cinemática directa: q [rad] -> (x, y) [mm] */
void kin_fk(const kin_cfg_t *c, float q1, float q2, float *x, float *y);

/*
 * Cinemática inversa.
 *   elbow_pref: +1 usa q2 >= 0, -1 usa q2 <= 0 (se intenta primero esta).
 *   Si la preferida viola límites articulares se prueba la otra (si falla también -> IK_LIMITES).
 *   qmin/qmax: límites articulares [rad].
 *   q_out: solución; elbow_used: codo realmente usado (+1/-1).
 */
ik_status_t kin_ik(const kin_cfg_t *c, float x, float y, int elbow_pref,
                   const float qmin[2], const float qmax[2],
                   float q_out[2], int *elbow_used);

const char *kin_ik_str(ik_status_t s);
