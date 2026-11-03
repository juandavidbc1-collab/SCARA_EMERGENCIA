#include "screw.h"
#include <math.h>

float screw_duty_para_torque(const screw_model_t *m, float torque)
{
    return (torque / m->kt + m->i0) * m->r / m->vbus * m->pwm_max;
}

float screw_vueltas_por_s(const screw_model_t *m, float duty)
{
    const float w = (duty / m->pwm_max * m->vbus - m->r * m->i0) / m->ke;
    return w > 0.0f ? w / (2.0f * (float)M_PI) : 0.0f;
}

float screw_profundidad_mm(const screw_model_t *m, float vueltas) { return vueltas * m->paso_mm; }

const char *screw_result_str(screw_result_t r)
{
    static const char *const n[] = { "ninguno", "sin_medir", "cancelado" };
    return r <= SCREW_R_CANCELADO ? n[r] : "?";
}
