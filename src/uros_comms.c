/*
 * CAPA micro-ROS (ROS 2 Jazzy) DEL SCARA.  Reemplaza a comms.c (protocolo de texto) cuando
 * CONFIG_SCARA_COMMS_UROS = y; el control, la cinemática y la seguridad NO cambian.
 *
 * Nodo "scara_esp32" en el espacio de nombres "scara":
 *
 *   Suscripciones (PC -> ESP32)
 *     /scara/goto          geometry_msgs/Point      x, y, z [mm]  -> cinemática inversa en C + movimiento
 *     /scara/jog           geometry_msgs/Vector3    x = dead-man (>0.5), y = dir J1, z = dir J2  (-1/0/+1)
 *                          best-effort. Publicar a 25 Hz MIENTRAS se mantiene el dead-man.
 *     /scara/cmd           std_msgs/String          "home", "stop", "off", "clear", "ang a b", "elbow +1",
 *                                                    "gain ...", "pulse 1", "status", "help" (ver uros_cmd.h)
 *
 *   Publicaciones (ESP32 -> PC)
 *     /scara/state         std_msgs/Float32MultiArray   20 Hz, best-effort. Ver STATE_* abajo.
 *     /scara/ee_position   geometry_msgs/Point          5 Hz, best-effort. Posición medida del efector [mm].
 *     /scara/event         std_msgs/String              "#LLEGO ...", "#HOME ...", "#ERROR ...", "#FALLA ..."
 *
 * Seguridad: el dead-man vive en el firmware (DEADMAN_TIMEOUT_MS): si dejan de llegar latidos
 * /scara/jog con dead-man = 1, los motores se detienen solos. Si se pierde el agente micro-ROS se
 * fuerza el jog a cero. Un movimiento punto a punto ya iniciado termina (es acotado y con límites).
 *
 * UART0 es del transporte serial: aquí no se usa printf.
 */
#include "comms.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/vector3.h>

#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
#include "driver/uart.h"
#include "uros_serial_transport.h"
#else
#include <uros_network_interfaces.h>
#endif

#include "config.h"
#include "control.h"
#include "kinematics.h"
#include "motion.h"
#include "uros_cmd.h"

#define NODE_NAME      "scara_esp32"
#define NODE_NS        "scara"
#define STATE_PERIOD_MS 50                  /* 20 Hz */
#define N_HANDLES      4                    /* 3 suscripciones + 1 temporizador */

/* Índices de /scara/state (Float32MultiArray) */
enum { STATE_Q1_DEG = 0, STATE_Q2_DEG, STATE_X, STATE_Y, STATE_MODE, STATE_FAULT, STATE_HOMED1, STATE_HOMED2,
       STATE_FC_BITS, STATE_QREF1_DEG, STATE_QREF2_DEG, STATE_DUTY1, STATE_DUTY2,
       STATE_EN_BITS,      /* bit 0 = J1, bit 1 = J2, bit 2 = M3 activos */
       STATE_CTRL,         /* 0 = PID; 1..5 = RST posicional, incremental, enmascarado, PID original, a medida */
       STATE_TOOL,         /* PWM de M3 */
       /* Atornillado. Calculado con el modelo de M3: no hay sensor de corriente. */
       STATE_TOOL_I, STATE_TOOL_TORQUE, STATE_TOOL_FUERZA, STATE_TOOL_PRESION, STATE_TOOL_VUELTAS, STATE_TOOL_PROF,
       STATE_SERVO_DEG, STATE_SCREW_FASE,
       STATE_N };

/* modo: 0 OFF, 1 HOMING, 2 HOLD, 3 MOVE, 4 JOG, 5 PULSE, 6 FAULT  (ctl_mode_t) */

#define RAD2DEG (180.0f / (float)M_PI)

/* ------------------------------------------------------------------ entidades y mensajes */
static rcl_allocator_t       allocator;
static rclc_support_t        support;
static rcl_init_options_t    init_options;
static rcl_node_t            node;
static rcl_publisher_t       pub_state, pub_ee, pub_event;
static rcl_subscription_t    sub_goto, sub_jog, sub_cmd;
static rcl_timer_t           timer;
static rclc_executor_t       executor;

static std_msgs__msg__Float32MultiArray msg_state;
static float                            state_data[STATE_N];
static geometry_msgs__msg__Point        msg_ee, msg_goto_in;
static geometry_msgs__msg__Vector3      msg_jog_in;
static std_msgs__msg__String            msg_event, msg_cmd_in;
static char                             event_buf[192], cmd_buf[96];

enum { C_OPTS = 1 << 0, C_SUPPORT = 1 << 1, C_NODE = 1 << 2, C_PUB_STATE = 1 << 3, C_PUB_EE = 1 << 4,
       C_PUB_EVENT = 1 << 5, C_SUB_GOTO = 1 << 6, C_SUB_JOG = 1 << 7, C_SUB_CMD = 1 << 8,
       C_TIMER = 1 << 9, C_EXEC = 1 << 10 };
static unsigned created = 0;
static volatile bool g_connected = false;
static int64_t t_conectado_us = 0;
static bool avisar_conexion = false;
#define ESPERA_DESCUBRIR_US 2000000LL   /* un evento publicado antes de que el panel descubra el publicador nuevo se pierde */

/* ------------------------------------------------------------------ publicar eventos */
static void send_event(const char *s)
{
    if (!g_connected) return;
    size_t k = strnlen(s, sizeof event_buf - 1);
    memcpy(event_buf, s, k);
    event_buf[k] = 0;
    msg_event.data.data = event_buf;
    msg_event.data.size = k;
    msg_event.data.capacity = sizeof event_buf;
    (void)rcl_publish(&pub_event, &msg_event, NULL);
}

/* ------------------------------------------------------------------ callbacks */
static void goto_cb(const void *in)
{
    const geometry_msgs__msg__Point *p = (const geometry_msgs__msg__Point *)in;
    char m[200];
    (void)motion_goto_xyz((float)p->x, (float)p->y, (float)p->z, m, sizeof m);
    send_event(m);
}

static int sgn(double v) { return (v > 0.5) - (v < -0.5); }

static void jog_cb(const void *in)
{
    const geometry_msgs__msg__Vector3 *v = (const geometry_msgs__msg__Vector3 *)in;
    ctl_cmd_jog(v->x > 0.5, sgn(v->y), sgn(v->z));
}

static void cmd_cb(const void *in)
{
    const std_msgs__msg__String *s = (const std_msgs__msg__String *)in;
    char line[96], reply[200];
    size_t k = 0;
    if (s->data.data) {
        k = strnlen(s->data.data, s->data.capacity < sizeof line ? s->data.capacity : sizeof line - 1);
        memcpy(line, s->data.data, k);
    }
    line[k] = 0;
    (void)uros_cmd_exec(line, reply, sizeof reply);
    send_event(reply);
}

static void timer_cb(rcl_timer_t *t, int64_t last_call_time)
{
    (void)last_call_time;
    if (!t) return;

    ctl_status_t s;
    ctl_get_status(&s);
    float x, y;
    kin_fk(&KIN, s.q[0], s.q[1], &x, &y);

    state_data[STATE_Q1_DEG]    = s.q[0] * RAD2DEG;
    state_data[STATE_Q2_DEG]    = s.q[1] * RAD2DEG;
    state_data[STATE_X]         = x;
    state_data[STATE_Y]         = y;
    state_data[STATE_MODE]      = (float)s.mode;
    state_data[STATE_FAULT]     = (float)s.fault;
    state_data[STATE_HOMED1]    = s.homed[0] ? 1.0f : 0.0f;
    state_data[STATE_HOMED2]    = s.homed[1] ? 1.0f : 0.0f;
    state_data[STATE_FC_BITS]   = (float)s.fc;
    state_data[STATE_QREF1_DEG] = s.qref[0] * RAD2DEG;
    state_data[STATE_QREF2_DEG] = s.qref[1] * RAD2DEG;
    state_data[STATE_DUTY1]     = s.duty[0];
    state_data[STATE_DUTY2]     = s.duty[1];
    state_data[STATE_EN_BITS]   = (float)s.en;
    state_data[STATE_CTRL]      = (float)s.ctrl;
    state_data[STATE_TOOL]      = s.tool_duty;
    state_data[STATE_TOOL_I]       = s.tool_i;
    state_data[STATE_TOOL_TORQUE]  = s.tool_torque;
    state_data[STATE_TOOL_FUERZA]  = s.tool_fuerza;
    state_data[STATE_TOOL_PRESION] = s.tool_presion;
    state_data[STATE_TOOL_VUELTAS] = s.tool_vueltas;
    state_data[STATE_TOOL_PROF]    = s.tool_prof;
    state_data[STATE_SERVO_DEG]    = s.servo_deg;
    state_data[STATE_SCREW_FASE]   = (float)s.screw_fase;
    (void)rcl_publish(&pub_state, &msg_state, NULL);

    /* /scara/ee_position repite x e y de /scara/state: a 5 Hz para liberar el enlace serie */
    static uint8_t div_ee = 0;
    if (++div_ee >= 4) {
        div_ee = 0;
        msg_ee.x = x; msg_ee.y = y; msg_ee.z = Z_FIJO_MM;
        (void)rcl_publish(&pub_ee, &msg_ee, NULL);
    }

    if (esp_timer_get_time() - t_conectado_us < ESPERA_DESCUBRIR_US) return;
    if (avisar_conexion) { avisar_conexion = false; send_event("#INFO: micro-ROS conectado"); }
    char ev[160];
    for (int i = 0; i < 4 && ctl_event_pop(ev, sizeof ev, 0); i++) send_event(ev);
}

/* ------------------------------------------------------------------ creación / destrucción */
#define RC(x) do { if ((x) != RCL_RET_OK) return false; } while (0)

static bool create_entities(void)
{
    created = 0;
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RC(rcl_init_options_init(&init_options, allocator));
    created |= C_OPTS;
#if !defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    if (rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options) != RMW_RET_OK) return false;
#endif
    RC(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    created |= C_SUPPORT;

    node = rcl_get_zero_initialized_node();
    RC(rclc_node_init_default(&node, NODE_NAME, NODE_NS, &support));
    created |= C_NODE;

    RC(rclc_publisher_init_best_effort(&pub_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "state"));
    created |= C_PUB_STATE;
    RC(rclc_publisher_init_best_effort(&pub_ee, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "ee_position"));
    created |= C_PUB_EE;
    RC(rclc_publisher_init_default(&pub_event, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "event"));
    created |= C_PUB_EVENT;

    RC(rclc_subscription_init_default(&sub_goto, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "goto"));
    created |= C_SUB_GOTO;
    RC(rclc_subscription_init_best_effort(&sub_jog, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "jog"));
    created |= C_SUB_JOG;
    RC(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cmd"));
    created |= C_SUB_CMD;

    timer = rcl_get_zero_initialized_timer();
    RC(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(STATE_PERIOD_MS), timer_cb, true));
    created |= C_TIMER;

    executor = rclc_executor_get_zero_initialized_executor();
    RC(rclc_executor_init(&executor, &support.context, N_HANDLES, &allocator));
    created |= C_EXEC;
    RC(rclc_executor_add_timer(&executor, &timer));
    RC(rclc_executor_add_subscription(&executor, &sub_goto, &msg_goto_in, &goto_cb, ON_NEW_DATA));
    RC(rclc_executor_add_subscription(&executor, &sub_jog, &msg_jog_in, &jog_cb, ON_NEW_DATA));
    RC(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd_in, &cmd_cb, ON_NEW_DATA));
    return true;
}

static void destroy_entities(void)
{
    g_connected = false;
    if (created & C_SUPPORT) {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);   /* no esperar a un agente caído */
    }
    if (created & C_EXEC)      (void)rclc_executor_fini(&executor);
    if (created & C_TIMER)     (void)rcl_timer_fini(&timer);
    if (created & C_SUB_CMD)   (void)rcl_subscription_fini(&sub_cmd, &node);
    if (created & C_SUB_JOG)   (void)rcl_subscription_fini(&sub_jog, &node);
    if (created & C_SUB_GOTO)  (void)rcl_subscription_fini(&sub_goto, &node);
    if (created & C_PUB_EVENT) (void)rcl_publisher_fini(&pub_event, &node);
    if (created & C_PUB_EE)    (void)rcl_publisher_fini(&pub_ee, &node);
    if (created & C_PUB_STATE) (void)rcl_publisher_fini(&pub_state, &node);
    if (created & C_NODE)      (void)rcl_node_fini(&node);
    if (created & C_SUPPORT)   (void)rclc_support_fini(&support);
    if (created & C_OPTS)      (void)rcl_init_options_fini(&init_options);
    created = 0;
}

/* ------------------------------------------------------------------ tarea principal */
typedef enum { ST_WAIT_AGENT, ST_AGENT_AVAILABLE, ST_CONNECTED, ST_DISCONNECTED } ustate_t;

static void prepare_static_messages(void)
{
    memset(&msg_state, 0, sizeof msg_state);
    msg_state.data.data = state_data;
    msg_state.data.size = STATE_N;
    msg_state.data.capacity = STATE_N;

    memset(&msg_cmd_in, 0, sizeof msg_cmd_in);
    msg_cmd_in.data.data = cmd_buf;
    msg_cmd_in.data.size = 0;
    msg_cmd_in.data.capacity = sizeof cmd_buf;

    memset(&msg_event, 0, sizeof msg_event);
    msg_event.data.data = event_buf;
    msg_event.data.capacity = sizeof event_buf;
}

static void uros_task(void *arg)
{
    (void)arg;
#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    static size_t uart_port = UART_NUM_0;
    rmw_uros_set_custom_transport(true, (void *)&uart_port, scara_serial_open, scara_serial_close,
                                  scara_serial_write, scara_serial_read);
#endif
    prepare_static_messages();

    ustate_t st = ST_WAIT_AGENT;
    int64_t t_ping = 0;
    for (;;) {
        switch (st) {
        case ST_WAIT_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) st = ST_AGENT_AVAILABLE;
            else vTaskDelay(pdMS_TO_TICKS(400));
            break;

        case ST_AGENT_AVAILABLE:
            if (create_entities()) {
                g_connected = true;
                t_ping = esp_timer_get_time();
                t_conectado_us = t_ping;
                avisar_conexion = true;
                st = ST_CONNECTED;
            } else {
                destroy_entities();
                st = ST_WAIT_AGENT;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case ST_CONNECTED:
            if (esp_timer_get_time() - t_ping > 1000000LL) {              /* ¿sigue el agente? (1 vez por segundo) */
                t_ping = esp_timer_get_time();
                if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) { st = ST_DISCONNECTED; break; }
            }
            (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            vTaskDelay(1);
            break;

        case ST_DISCONNECTED:
            ctl_cmd_jog(false, 0, 0);                                    /* sin PC no hay movimiento manual */
            destroy_entities();
            st = ST_WAIT_AGENT;
            break;
        }
    }
}

void comms_start(void)
{
#if !defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    /* núcleo 0 (el lazo de control va fijo en el núcleo 1) */
    xTaskCreatePinnedToCore(uros_task, "uros", CONFIG_SCARA_UROS_STACK, NULL, CONFIG_SCARA_UROS_PRIO, NULL, 0);
}
