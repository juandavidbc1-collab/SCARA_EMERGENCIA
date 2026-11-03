/*
 * NÚCLEO DE CONTROL SIN ENCODERS: lazo de 1 kHz (tarea FreeRTOS de alta prioridad en el núcleo 1). Misma interfaz y mismos
 * modos que el firmware principal, para que el panel, RViz y los comandos sirvan igual.
 *
 * Lo que cambia: sin medición no hay PID ni RST. Cada eje se enciende el tiempo calculado y se frena. q es una ESTIMACIÓN.
 * La única medición real son los finales de carrera, así que un eje trabado no se nota hasta el siguiente "ref".
 *
 * Máquina de estados:
 *
 *   OFF ──home──▶ HOMING ──(ok)──▶ MOVE(vuelve a donde estaba) ──▶ HOLD
 *    ▲  ▲                                      ▲                │
 *    │  └─ off/stop ───────────────────────────┼── goto/ang ─▶ MOVE ──(llegó)──▶ HOLD
 *    │                                         │
 *    └──── clear ──── FAULT ◀── cualquier falla    OFF/HOLD ◀──▶ JOG (flechas + espacio)
 *
 *  - Toda falla: freno 150 ms, motores libres, STBY = 0 y modo FAULT (hasta "clear").
 *  - JOG solo se mantiene mientras llegan latidos con dead-man = 1 (ver DEADMAN_TIMEOUT_MS).
 *  - MOVE/goto exigen que las articulaciones activas estén referenciadas (Home hecho).
 *  - Paro de emergencia: no pasa por la cola de órdenes. Freno, motores libres, STBY = 0, enclavado en
 *    FAULT (F_ESTOP) hasta "clear". Conserva el Home salvo que interrumpa un Home en curso.
 *  - Motor desactivado: driver libre. Pierde el Home, porque se puede mover a mano y nadie lo mediría. Home solo con los activos.
 *  - Atornillado (M3): por tiempo. El brazo se queda quieto. Paro, falla, detener, apagar o desactivar M3 lo cancelan. Paro y falla cierran el servo.
 *  - HOLD: los ejes frenados (no hay con qué mantener una posición).
 *  - "ref": recorre los ejes de tope a tope. Corrige la posición, vuelve a medir la velocidad y detecta si el brazo se perdió.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "config.h"

typedef enum { M_OFF = 0, M_HOMING, M_HOLD, M_MOVE, M_JOG, M_PULSE, M_FAULT } ctl_mode_t;

typedef enum {
    F_NONE = 0, F_FOLLOW, F_STALL, F_LIMIT, F_HOME_TIMEOUT, F_DIAG_FC, F_ESTOP, F_PERDIDO,   /* F_FOLLOW y F_STALL no se usan: sin encoder no se detectan */
} fault_t;

#define CTRL_POR_TIEMPO 6             /* ctl_status_t.ctrl: índice de CTRLS en scara_link.py */

typedef struct {
    ctl_mode_t mode;
    fault_t    fault;
    bool       homed[NJ];
    float      q[NJ];         /* [rad] ESTIMADO */
    float      qref[NJ];      /* [rad] destino */
    float      duty[NJ];      /* PWM aplicado */
    uint8_t    fc;            /* bit (2*j + side): final de carrera presionado (con antirrebote) */
    uint8_t    en;            /* bit 0 = J1, bit 1 = J2, bit 2 = M3 activos */
    uint8_t    ctrl;          /* siempre CTRL_POR_TIEMPO */
    float      tool_duty;     /* PWM de M3 */
    /* Sin encoder de M3 no se puede calcular corriente, torque, fuerza ni presión: van en 0. */
    float      tool_i, tool_torque, tool_fuerza, tool_presion;
    float      tool_vueltas;  /* ESTIMADAS, del tornillo actual */
    float      tool_prof;     /* [mm] ESTIMADA */
    float      servo_deg;
    uint8_t    screw_fase;    /* screw_phase_t */
} ctl_status_t;

void ctl_init(void);          /* crea colas; no mueve nada */
void ctl_start(void);         /* lanza la tarea del lazo de control */

/* Latido de modo manual: dm = dead-man (espacio presionado); dir = -1/0/+1 por articulación. */
void ctl_cmd_jog(bool dm, int d1, int d2);

/* Órdenes: devuelven false si se rechazan (el motivo sale como evento "#err ..."). */
bool ctl_cmd_home(void);
bool ctl_cmd_move(const float q[NJ], const float tgt_xyz[3]);   /* tgt_xyz = NAN si no aplica */
void ctl_cmd_stop(void);      /* detiene el movimiento y mantiene posición (HOLD) */
void ctl_cmd_off(void);       /* apaga motores (STBY = 0) */
void ctl_cmd_clear(void);     /* borra una falla */
bool ctl_cmd_pulse(int j);
bool ctl_cmd_ref(void);       /* recorre los ejes de tope a tope. Al terminar emite "#LLEGO" */
bool ctl_cmd_ajuste(int j, float pedido_deg, float medido_deg);   /* corrige la velocidad con un ángulo medido a mano */
void ctl_get_mapa(int j, float out[5]);   /* v- v+ [grados/s], arranque- arranque+ [s], factor de ajuste */

void ctl_cmd_estop(void);     /* paro de emergencia: siempre se atiende en el siguiente tick */
bool ctl_cmd_motor(int m, bool on);   /* m: 0 = J1, 1 = J2, 2 = M3 */
bool ctl_motor_enabled(int m);
bool ctl_cmd_tool(float pct);         /* M3 en lazo abierto: -100..100 %. Hay que renovarla antes de TOOL_MAX_ON_MS */
bool ctl_cmd_servo(float deg);        /* 0 a SERVO_GRADOS_MAX */
bool ctl_cmd_screw(float vueltas, float torque);   /* 0 = valor por defecto. Al terminar emite "#TORNILLO: sin_medir ..." */
/* Limites por software vigentes [rad]: los de config.c hasta que se hace Home. */
void ctl_get_limits(int j, float *qmin, float *qmax);

void ctl_get_status(ctl_status_t *s);
const char *ctl_mode_str(ctl_mode_t m);
const char *ctl_fault_str(fault_t f);

/* Cola de mensajes del lazo de control hacia el puerto serie. true si hay mensaje. */
bool ctl_event_pop(char *buf, size_t n, uint32_t wait_ms);
