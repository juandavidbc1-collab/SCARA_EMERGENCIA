/*
 * ÓRDENES DE MOVIMIENTO COMPARTIDAS por todas las interfaces (texto por serial y micro-ROS).
 *
 * Valida la consigna (z fija, alcance, límites articulares, singularidades), calcula la
 * cinemática inversa en C y encola el movimiento en el lazo de control. Así el protocolo de
 * texto y ROS 2 usan EXACTAMENTE la misma lógica y los mismos mensajes.
 *
 * El texto devuelto en msg empieza por "#INFO:" (aceptada) o "#ERROR:" (rechazada) y NO
 * lleva salto de línea.
 */
#pragma once
#include <stddef.h>

typedef enum {
    MOTION_OK = 0,
    MOTION_ERR_Z,          /* z fuera de la altura fija */
    MOTION_ERR_IK,         /* inalcanzable / singular / sin solución dentro de límites */
    MOTION_ERR_LIMITES,    /* ángulo fuera de límites (goto en ángulos) */
    MOTION_ERR_COLA,       /* el lazo de control no aceptó la orden */
    MOTION_ERR_MOTOR,      /* goto con J1 o J2 desactivado */
} motion_res_t;

int  motion_get_elbow(void);            /* codo preferido: +1 (q2 >= 0) o -1 (q2 <= 0) */
void motion_set_elbow(int e);

motion_res_t motion_goto_xyz(float x, float y, float z, char *msg, size_t n);
motion_res_t motion_goto_ang(float q1_deg, float q2_deg, char *msg, size_t n);
