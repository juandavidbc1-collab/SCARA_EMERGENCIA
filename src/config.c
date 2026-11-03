#include "config.h"
#include <math.h>

#define DEG(x) ((x) * (float)M_PI / 180.0f)

/* Longitudes de los eslabones (centro de eje a centro de eje): L1 = 120 mm, L2 = 70 mm
 * -> alcance de 50 a 190 mm. */
const kin_cfg_t KIN = { .l1 = 120.0f, .l2 = 70.0f, .sing_margin = DEG(5.0f) };

/*
 * Motor 3: Pololu 37D 50:1 a 12 V, referido al eje de salida. Hoja de datos: 200 rpm y 0,2 A en vacío; 5,5 A y 21 kg·cm
 * calado (valores extrapolados por el fabricante). Sin medir en este robot.
 *   R = 12 / 5,5;  Ke = (12 - R I0) / w0;  Kt = T_calado / (I_calado - I0)   (Kt < Ke: pérdidas del reductor)
 * torque_rosca: lo que cuesta girar la rosca antes de que asiente la cabeza. Supuesto.
 */
const screw_model_t M3_MODEL = {
    .r = 2.1818f, .ke = 0.5522f, .kt = 0.3886f, .i0 = 0.20f, .vbus = 12.0f, .pwm_max = PWM_MAX,
    .paso_mm = 0.5f, .torque_rosca = 0.03f,
};

/*
 * Signos y pines: los mismos del firmware principal. q_home y rango son [MEDIR]: aquí están los del simulador, que usa los
 * mismos topes que el del firmware principal.
 */
const joint_cfg_t JC[NJ] = {
    {
        .name = "J1 (eslabon 1)",
        .pwm = PIN_M1_PWM, .in1 = PIN_M1_IN1, .in2 = PIN_M1_IN2,
        /* topes cruzados: el final de carrera izquierdo (extremo + de q) esta en el conector FC1_MIN */
        .fc_min = PIN_FC1_MAX, .fc_max = PIN_FC1_MIN,
        .motor_sign = -1,
        .q_min = DEG(-120.0f), .q_max = DEG(120.0f), .q_home = DEG(-108.0f), .rango = DEG(228.0f),
    },
    {
        .name = "J2 (eslabon 2)",
        .pwm = PIN_M2_PWM, .in1 = PIN_M2_IN1, .in2 = PIN_M2_IN2,
        .fc_min = PIN_FC2_MAX, .fc_max = PIN_FC2_MIN,
        .motor_sign = -1,
        .q_min = DEG(-140.0f), .q_max = DEG(140.0f), .q_home = DEG(-143.0f), .rango = DEG(286.0f),
    },
};
