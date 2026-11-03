/*
 * ATORNILLADO SIN ENCODER. No hay corriente, velocidad ni vueltas medidas: solo un PWM y un tiempo, sacados del modelo de M3.
 *
 * El PWM es el que, con el motor calado, da el torque pedido: en teoría no se puede pasar de ahí. El tiempo es el que
 * tardan las vueltas pedidas en vacío a ese PWM, con margen. Nada de esto comprueba que el tornillo haya quedado bien.
 */
#pragma once

typedef struct {
    float r, ke, kt, i0;        /* motor, referido al eje de salida */
    float vbus, pwm_max;
    float paso_mm;              /* avance de la rosca por vuelta */
    float torque_rosca;         /* [N·m] lo que cuesta girar la rosca: solo lo usa el simulador */
} screw_model_t;

typedef enum { SCREW_IDLE = 0, SCREW_APRIETE = 3, SCREW_FIN = 4 } screw_phase_t;   /* mismos índices que SCREW_PHASES en scara_link.py */
typedef enum { SCREW_R_NINGUNO = 0, SCREW_R_SIN_MEDIR, SCREW_R_CANCELADO } screw_result_t;

float screw_duty_para_torque(const screw_model_t *m, float torque);   /* PWM con el que el motor cala en ese torque */
float screw_vueltas_por_s(const screw_model_t *m, float duty);        /* en vacío, según el modelo */
float screw_profundidad_mm(const screw_model_t *m, float vueltas);
const char *screw_result_str(screw_result_t r);
