/*
 * CONFIGURACIÓN CENTRAL DEL ROBOT SCARA, FIRMWARE DE EMERGENCIA SIN ENCODERS.
 *
 * Mismos pines, sentidos y PWM que el firmware principal. Lo marcado con [MEDIR] es lo que Home no puede saber sin encoder.
 *
 * >>> Todo lo marcado con  [CAMBIAR]  son valores PROVISIONALES: hay que medirlos en el robot. <<<
 * Pines y datos eléctricos vienen del esquema v5 (documento de contexto).
 */
#pragma once
#include <stdint.h>
#if defined(__has_include)
#  if __has_include("sdkconfig.h")
#    include "sdkconfig.h"          /* CONFIG_SCARA_COMMS_UROS (src/Kconfig.projbuild) */
#  endif
#endif
#include "driver/gpio.h"
#include "kinematics.h"

#define NJ 2                       /* articulaciones controladas: eslabón 1 y eslabón 2 */

/* ------------------------------------------------------------------ PINES */
#define PIN_STBY      GPIO_NUM_17  /* habilita U5/U6; R10 lo mantiene en 0 al arrancar */

#define PIN_M1_PWM    GPIO_NUM_2
#define PIN_M1_IN1    GPIO_NUM_16
#define PIN_M1_IN2    GPIO_NUM_4
#define PIN_M2_PWM    GPIO_NUM_19
#define PIN_M2_IN1    GPIO_NUM_5
#define PIN_M2_IN2    GPIO_NUM_18
/* Motor 3 (atornillador): NO se usa por ahora. Sus pines se dejan como salidas en 0 para que
 * las entradas de U6 no queden flotando. */
#define PIN_M3_PWM    GPIO_NUM_21
#define PIN_M3_IN1    GPIO_NUM_23
#define PIN_M3_IN2    GPIO_NUM_22
/* Servo MG90S en J13 (donde iba el ACS712): pin 1 = 5 V, pin 2 = señal, pin 3 = GND. La señal pasa por R40 (10 k) en serie
 * hacia GPIO32; R41 (22 k) y C14 (100 nF) quedan a GND. Si el servo no responde o tiembla, puentea R40. */
#define PIN_SERVO     GPIO_NUM_32

#define PIN_FC1_MIN   GPIO_NUM_36
#define PIN_FC1_MAX   GPIO_NUM_39
#define PIN_FC2_MIN   GPIO_NUM_34
#define PIN_FC2_MAX   GPIO_NUM_35

/* Interruptor normalmente abierto a GND con pull-up -> presionado = 0 (1 si usas NC) */
#define FC_PRESIONADO_NIVEL 0

/* ------------------------------------------------------------------ GEOMETRÍA (cinemática) */
extern const kin_cfg_t KIN;
#define Z_FIJO_MM     0.0f   /* [CAMBIAR] altura a la que trabaja el efector (eje z fijo por ahora) */
#define Z_TOL_MM      5.0f   /* se rechaza una consigna cuya z difiera más que esto de Z_FIJO */
#define Z_CHECK       1      /* 0 = ignora z de la consigna */
#define ELBOW_DEFAULT (+1)   /* codo preferido: +1 (q2>=0) o -1 (q2<=0) */

/* ------------------------------------------------------------------ ARTICULACIONES */
typedef struct {
    const char *name;
    gpio_num_t pwm, in1, in2, fc_min, fc_max;
    int   motor_sign;    /* +1/-1: hace que u>0 (duty positivo) AUMENTE q (se calibra con "pulse") */
    float q_min, q_max;  /* límites por software [rad] hasta que se haga Home */
    /* Sin encoder, estos dos números son lo único que convierte los tiempos de Home en ángulos. [MEDIR] (README) */
    float q_home;        /* [rad] ángulo donde queda el eje al terminar "ref" (punto de Home) */
    float rango;         /* [rad] desde q_home hasta donde se pulsa el final de carrera MAX */
} joint_cfg_t;
extern const joint_cfg_t JC[NJ];
#include "screw.h"
extern const screw_model_t M3_MODEL;

/* ------------------------------------------------------------------ CONTROL */
#define CONTROL_HZ        1000
#define PWM_FREQ_HZ       20000
#define PWM_MAX           1023
#define DUTY_MAX          650        /* tope de PWM en lazo cerrado (~63 %): protege U5 (<=0.8 A por canal) */
#define DUTY_SLEW_PER_MS  3          /* rampa de PWM: 0 -> PWM_MAX en 0.34 s (regla de seguridad >= 0.3 s) */

/* Manual (teclado) */
#if defined(CONFIG_SCARA_COMMS_UROS)
/* Con ROS 2 los latidos viajan por XRCE-DDS (mas latencia/jitter que el serial de texto): mas margen. */
#define DEADMAN_TIMEOUT_MS  300
#else
#define DEADMAN_TIMEOUT_MS  150      /* sin latido del PC en este tiempo -> se detiene (cae el USB, se cuelga el PC). Soltar el
                                      * espacio detiene de inmediato porque el PC envía "d 0 0 0" al soltar. */
#endif
#define JOG_FRENO_MS        120      /* freno al soltar una flecha */

/* Seguridad */
#define FC_ANTIREBOTE_MS  5          /* filtro por software (además de los 100 nF) */
#define FALLA_FRENO_MS    150

/* Home */
#define HOME_DUTY         170        /* búsqueda del final de carrera (~17 %); valor calibrado en el robot real */
#define HOME_DUTY_SLOW    140        /* retroceso hasta liberar el final de carrera (~14 %) */
/* Orden en que se referencian las articulaciones (índices 0-based). [CAMBIAR] según colisiones. */
#define HOME_ORDER_INIT   { 1, 0 }
/* El Home recorre cada eje de tope a tope midiendo TIEMPOS y al final devuelve el robot a donde estaba (estimado).
 * Los límites por software quedan este margen dentro del recorrido. */
#define HOME_MARGEN_RAD   0.052f     /* ~3 grados */

/* Prueba de sentido ("pulse"): sin encoder, el resultado lo ves tú */
#define PULSE_MS          350

/* ------------------------------------------------------------------ MOVIMIENTO POR TIEMPO
 * tiempo = grados / velocidad + tiempo perdido al arrancar. Home mide ambos en cada sentido. */
#define PULSO_MS           600       /* mide el tiempo que se pierde al arrancar. Con pulsos cortos sale menor que el de un movimiento largo */
#define PAUSA_FRENO_MS     200       /* freno después de cada tramo */
#define T_TRAVESIA_MS      25000     /* más que esto de tope a tope: el eje está trabado o el final de carrera no funciona */
#define T_SALIR_FC_MS      3000
#define APROX_DEG          20.0f     /* el último tramo siempre va hacia + y mide esto: repite el error y deja el juego del mismo lado */
#define TRAMO_MIN_DEG      20.0f     /* con menos de unos 300 ms encendido, el motor no alcanza a perder todo el tiempo de arranque */
#define MOVER_MIN_DEG      0.1f      /* un movimiento menor no se hace */
#define DESVIO_MAX_DEG     8.0f      /* "ref": más diferencia que esto es que el brazo se perdió */
#define DESVIO_PRIMER_REF_MAX_DEG 15.0f  /* el primer "ref" tras Home incluye la distancia fija hasta el final de carrera */
/* Antes de Home, solo para mostrar algo. Y PWM al que el motor apenas se mueve, para estimar la velocidad a otro PWM. */
#define VEL_NOMINAL_DEG_S  66.7f
#define T0_NOMINAL_S       0.10f
#define PWM_SIN_MOVER      56.0f     /* del modelo del motor, sin medir en el robot */
#define VEL_MIN_DEG_S      5.0f
#define VEL_MAX_DEG_S      400.0f
#define VEL_DIFERENCIA_MAX 0.25f     /* más diferencia entre un sentido y otro: revisa la mecánica */
/* Corrige la velocidad medida. Lo calcula el comando "ajuste"; cópialo aquí para que no se pierda al reiniciar. */
#ifndef J1_AJUSTE
#define J1_AJUSTE          1.0f
#endif
#ifndef J2_AJUSTE
#define J2_AJUSTE          1.0f
#endif

#define TELE_PERIODO_MS   100

/* Motor 3 (atornillador). Arranca desactivado; se activa con "motor 3 on". */
#define MOTORES_INIT      { true, true }
#define TOOL_ACTIVO_INIT  false
#define TOOL_DUTY_MAX     DUTY_MAX
#define TOOL_MOTOR_SIGN   (+1)       /* [CAMBIAR] tras probar el sentido: duty > 0 debe apretar */
#define TOOL_MAX_ON_MS    10000      /* sin renovar "tool" en este tiempo, M3 se apaga (comunicación perdida) */

/* Modelo de M3 (src/config.c) y atornillado. Tornillo supuesto M3 x 0,5 de 6 mm: [CAMBIAR] según el tornillo real. */
#define SCREW_VUELTAS_META   12.0f   /* vueltas hasta que asienta la cabeza */
#define SCREW_TORQUE_META    0.40f   /* N·m: apriete final */
#define SCREW_MARGEN_T       1.25f   /* tiempo de más sobre el calculado, por si va más lento */

/* Servo MG90S: 50 Hz, pulso de 500 a 2400 us para 0 a 180 grados. [CAMBIAR] los ángulos al montar el dispensador. */
#define SERVO_HZ             50
#define SERVO_US_MIN         500.0f
#define SERVO_US_MAX         2400.0f
#define SERVO_GRADOS_MAX     180.0f
#define SERVO_CERRADO_DEG    0.0f
#define SERVO_ABIERTO_DEG    90.0f
