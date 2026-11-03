/*
 * Biblioteca del simulador: el firmware de emergencia (src/control.c, motion.c, uros_cmd.c) con la planta de sim_core.h.
 * ros2/scara_sim_node.py la carga con ctypes y publica los mismos tópicos que el ESP32.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kinematics.h"
#include "../../src/control.c"
#include "sim_core.h"
#include "motion.h"
#include "uros_cmd.h"

void sim_init(void)
{
    plant_reset(NULL, 10.0, 30.0);
    ctl_init();
}

void sim_step_ms(int n)
{
    for (int k = 0; k < n; k++) { g_now += 1000; planta_1ms(); tick(); }
}

int  sim_cmd(const char *line, char *reply, int n) { return uros_cmd_exec(line, reply, (size_t)n); }
int  sim_goto(float x, float y, float z, char *msg, int n) { return (int)motion_goto_xyz(x, y, z, msg, (size_t)n); }
void sim_jog(int dm, int d1, int d2) { ctl_cmd_jog(dm != 0, d1, d2); }
int  sim_event_pop(char *buf, int n) { return ctl_event_pop(buf, (size_t)n, 0); }
void sim_lock(int j, int on) { if (j >= 0 && j < NJ) g_lock[j] = on != 0; }
void sim_power(int on) { g_power = on != 0; }
void sim_screw_mode(int m) { g_screw_mode = (sim_tornillo_t)m; }
/* Para tools/sim/precision.py: lo que el firmware no ve. */
void sim_seed(unsigned s) { srand(s); }
void sim_fijo(double juego_deg, double rango_err_deg) { PC.juego_deg = juego_deg; for (int j = 0; j < NJ; j++) sw_max_deg[j] += rango_err_deg; }
void sim_cambio(double v_fac, double tc_fac, double ruido) { PC.v_fac = v_fac; PC.ruido = ruido; for (int j = 0; j < NJ; j++) P[j].tc_ahora = PC.tc_fac[j] = tc_fac; }
void sim_real_deg(double *q) { q[0] = P[0].salida_deg; q[1] = P[1].salida_deg; }
double sim_error_mm(double x, double y) { return plant_error_mm(x, y); }
void sim_m3_real(double *d) { d[0] = g_m3_i_real; d[1] = g_m3_torque_real; d[2] = g_m3_w; }

/* Mismos índices que /scara/state en src/uros_comms.c */
void sim_state(float *d)
{
    ctl_status_t s;
    ctl_get_status(&s);
    float x, y; kin_fk(&KIN, s.q[0], s.q[1], &x, &y);
    d[0] = s.q[0] * R2D; d[1] = s.q[1] * R2D; d[2] = x; d[3] = y; d[4] = (float)s.mode; d[5] = (float)s.fault;
    d[6] = s.homed[0]; d[7] = s.homed[1]; d[8] = (float)s.fc; d[9] = s.qref[0] * R2D; d[10] = s.qref[1] * R2D;
    d[11] = s.duty[0]; d[12] = s.duty[1]; d[13] = (float)s.en; d[14] = (float)s.ctrl; d[15] = s.tool_duty;
    d[16] = s.tool_i; d[17] = s.tool_torque; d[18] = s.tool_fuerza; d[19] = s.tool_presion; d[20] = s.tool_vueltas; d[21] = s.tool_prof;
    d[22] = s.servo_deg; d[23] = (float)s.screw_fase;
}
