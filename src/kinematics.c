#include "kinematics.h"
#include <math.h>

void kin_fk(const kin_cfg_t *c, float q1, float q2, float *x, float *y)
{
    *x = c->l1 * cosf(q1) + c->l2 * cosf(q1 + q2);
    *y = c->l1 * sinf(q1) + c->l2 * sinf(q1 + q2);
}

static bool dentro(const float q[2], const float qmin[2], const float qmax[2])
{
    return q[0] >= qmin[0] && q[0] <= qmax[0] && q[1] >= qmin[1] && q[1] <= qmax[1];
}

/* Resuelve una rama (sgn = +1 o -1) dado c2 ya calculado. */
static void rama(const kin_cfg_t *c, float x, float y, float c2, int sgn, float q[2])
{
    float s2 = sgn * sqrtf(fmaxf(0.0f, 1.0f - c2 * c2));
    q[1] = atan2f(s2, c2);
    q[0] = atan2f(y, x) - atan2f(c->l2 * s2, c->l1 + c->l2 * c2);
    /* normaliza q1 a (-pi, pi] */
    if (q[0] > (float)M_PI)   q[0] -= 2.0f * (float)M_PI;
    if (q[0] <= -(float)M_PI) q[0] += 2.0f * (float)M_PI;
}

ik_status_t kin_ik(const kin_cfg_t *c, float x, float y, int elbow_pref,
                   const float qmin[2], const float qmax[2],
                   float q_out[2], int *elbow_used)
{
    float r2 = x * x + y * y;
    float c2 = (r2 - c->l1 * c->l1 - c->l2 * c->l2) / (2.0f * c->l1 * c->l2);

    if (c2 > 1.0f || c2 < -1.0f) return IK_INALCANZABLE;

    /* |sin(q2)| = sqrt(1 - c2^2) debe superar el margen de singularidad */
    float s2abs = sqrtf(1.0f - c2 * c2);
    if (s2abs < sinf(c->sing_margin)) return IK_SINGULAR;

    int pref = (elbow_pref >= 0) ? +1 : -1;
    float q[2];

    rama(c, x, y, c2, pref, q);
    if (dentro(q, qmin, qmax)) {
        q_out[0] = q[0]; q_out[1] = q[1];
        if (elbow_used) *elbow_used = pref;
        return IK_OK;
    }
    rama(c, x, y, c2, -pref, q);
    if (dentro(q, qmin, qmax)) {
        q_out[0] = q[0]; q_out[1] = q[1];
        if (elbow_used) *elbow_used = -pref;
        return IK_OK;
    }
    return IK_LIMITES;
}

const char *kin_ik_str(ik_status_t s)
{
    switch (s) {
    case IK_OK:           return "ok";
    case IK_INALCANZABLE: return "inalcanzable (fuera del espacio de trabajo)";
    case IK_SINGULAR:     return "singular (muy cerca de q2=0 o 180)";
    case IK_LIMITES:      return "viola limites articulares en ambos codos";
    }
    return "?";
}
