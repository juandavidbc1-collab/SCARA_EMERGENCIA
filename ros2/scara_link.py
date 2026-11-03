#!/usr/bin/env python3
"""
Enlace del panel con el robot (real o simulado).

Lo que hace rápido al panel: ROS 2 corre en su propio hilo. Los mensajes se reciben al llegar, sin esperar a que la
ventana se redibuje, y el botón de paro publica directamente. La ventana solo lee la última copia.

El latido del dead-man también va en ese hilo, para que no sufra por la ventana. A cambio hay un seguro: si la ventana
deja de responder 150 ms, el latido suelta el dead-man aunque las teclas sigan pulsadas.
"""
import collections
import threading
import time
from dataclasses import dataclass

JOG_HZ = 25
MODES = ("OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT")
FAULTS = ("ninguna", "error de seguimiento", "motor trabado", "final de carrera inesperado", "tiempo de Home agotado",
          "MIN y MAX a la vez", "paro de emergencia", "se perdió la referencia")
CTRLS = ("?", "?", "?", "?", "?", "?", "Por tiempo (sin encoder)")   # índice: CTRL_POR_TIEMPO en src/control.h
SCREW_PHASES = ("Sin atornillar", "Rápido", "Lento", "Apriete", "Terminado")
F_ESTOP = 6
ST_MODE_FAULT = 6

@dataclass
class RobotState:
    q1: float
    q2: float
    x: float
    y: float
    mode: str
    estop: bool
    faulted: bool
    fault_text: str
    home1: bool
    home2: bool
    fc_j1_min: bool
    fc_j1_max: bool
    fc_j2_min: bool
    fc_j2_max: bool
    ref1: float
    ref2: float
    duty1: float
    duty2: float
    j1_on: bool
    j2_on: bool
    m3_on: bool
    ctrl: str
    tool: float
    tool_i: float = 0.0        # [A]; este y los siguientes salen del modelo de M3: no hay sensor de corriente
    tool_torque: float = 0.0   # [N·m]
    tool_fuerza: float = 0.0   # [N]
    tool_presion: float = 0.0  # [MPa]
    tool_vueltas: float = 0.0
    tool_prof: float = 0.0     # [mm]
    servo: float = 0.0         # [grados]
    screw_fase: str = SCREW_PHASES[0]


def decode_state(d):
    """/scara/state -> RobotState, o None si el mensaje está incompleto. Índices en src/uros_comms.c."""
    if d is None or len(d) < 13:
        return None
    m, f = int(d[4]), int(d[5])
    faulted = m == ST_MODE_FAULT
    estop = faulted and f == F_ESTOP
    mode = "PARO" if estop else "FALLA" if faulted else (MODES[m] if 0 <= m < len(MODES) else "?")
    en = int(d[13]) if len(d) > 13 else 3
    c = int(d[14]) if len(d) > 14 else -1
    fc = int(d[8])
    extra = {}
    if len(d) >= 24:
        p = int(d[23])
        extra = dict(tool_i=d[16], tool_torque=d[17], tool_fuerza=d[18], tool_presion=d[19], tool_vueltas=d[20], tool_prof=d[21],
                     servo=d[22], screw_fase=SCREW_PHASES[p] if 0 <= p < len(SCREW_PHASES) else "?")
    return RobotState(q1=d[0], q2=d[1], x=d[2], y=d[3], mode=mode, estop=estop, faulted=faulted,
                      fault_text=FAULTS[f] if 0 <= f < len(FAULTS) else "desconocida",
                      home1=bool(d[6]), home2=bool(d[7]),
                      fc_j1_min=bool(fc & 1), fc_j1_max=bool(fc & 2), fc_j2_min=bool(fc & 4), fc_j2_max=bool(fc & 8),
                      ref1=d[9], ref2=d[10], duty1=d[11], duty2=d[12],
                      j1_on=bool(en & 1), j2_on=bool(en & 2), m3_on=bool(en & 4),
                      ctrl=CTRLS[c] if 0 <= c < len(CTRLS) else "?", tool=d[15] if len(d) > 15 else 0.0, **extra)


class Deadman:
    """Decide qué latido enviar. Sin dead-man no se publica nada, para no gastar el enlace serie del robot."""

    def __init__(self, gui_timeout=0.15, release_repeats=3):
        self.gui_timeout, self.release_repeats = gui_timeout, release_repeats
        self.keys = frozenset()
        self.t_gui = 0.0
        self.release_left = 0

    def set_keys(self, keys):
        self.keys = frozenset(keys)

    def gui_beat(self, now):
        self.t_gui = now

    def next_message(self, now):
        """(dm, d1, d2) a publicar en este ciclo, o None si no hay nada que decir."""
        k = self.keys
        if "space" in k and now - self.t_gui <= self.gui_timeout:
            self.release_left = self.release_repeats
            return (1.0, float(("right" in k) - ("left" in k)), float(("up" in k) - ("down" in k)))
        if self.release_left > 0:                 # soltar se repite: /scara/jog es best-effort y puede perderse
            self.release_left -= 1
            return (0.0, 0.0, 0.0)
        return None


class ScaraLink:
    """Nodo ROS 2 en un hilo propio. Todos sus métodos se pueden llamar desde la ventana sin bloquearla."""

    def __init__(self, node_name="scara_panel"):
        import rclpy
        from geometry_msgs.msg import Point, Vector3
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float32MultiArray, String

        self._Point, self._String, self._Vector3 = Point, String, Vector3
        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=500)
        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.pub_cmd = self.node.create_publisher(String, "/scara/cmd", rel)
        self.pub_goto = self.node.create_publisher(Point, "/scara/goto", rel)
        self.pub_jog = self.node.create_publisher(Vector3, "/scara/jog", best)
        self.pub_inject = self.node.create_publisher(String, "/scara_sim/inject", rel)
        self.node.create_subscription(Float32MultiArray, "/scara/state", self._on_state, best)
        self.node.create_subscription(String, "/scara/event", self._on_event, rel)
        self.node.create_timer(1.0 / JOG_HZ, self._on_jog_timer)

        self.deadman = Deadman()
        self.deadman.gui_beat(time.monotonic())
        self._last_jog = (0.0, 0.0, 0.0)
        self._state, self._t_state = None, 0.0
        self._rate, self._count, self._t_rate = 0.0, 0, time.monotonic()
        self._events = collections.deque(maxlen=5000)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._exec.spin()
        except Exception:                         # al cerrar, el contexto puede desaparecer durante la espera
            pass

    def _on_state(self, m):
        now = time.monotonic()
        self._state, self._t_state = list(m.data), now    # asignación atómica: la ventana lee la última completa
        self._count += 1
        if now - self._t_rate >= 1.0:
            self._rate, self._count, self._t_rate = self._count / (now - self._t_rate), 0, now

    def _on_event(self, m):
        self._events.append(m.data)

    def _on_jog_timer(self):
        msg = self.deadman.next_message(time.monotonic())
        if msg is None:
            self._last_jog = (0.0, 0.0, 0.0)
            return
        self._last_jog = msg
        self.pub_jog.publish(self._Vector3(x=msg[0], y=msg[1], z=msg[2]))

    # --- llamadas desde la ventana
    def command(self, text):
        self.pub_cmd.publish(self._String(data=text))

    def goto(self, x, y, z):
        self.pub_goto.publish(self._Point(x=float(x), y=float(y), z=float(z)))

    def inject(self, text):
        self.pub_inject.publish(self._String(data=text))

    def snapshot(self):
        """(último estado o None, segundos desde que llegó, mensajes por segundo)."""
        return self._state, time.monotonic() - self._t_state, self._rate

    def pop_events(self, n):
        out = []
        while self._events and len(out) < n:
            out.append(self._events.popleft())
        return out

    def set_keys(self, keys):
        self.deadman.set_keys(keys)

    def gui_beat(self):
        self.deadman.gui_beat(time.monotonic())

    def jog_active(self):
        return self._last_jog

    def close(self):
        self.deadman.set_keys(())
        try:
            for _ in range(3):                    # suelta el dead-man antes de irse
                self.pub_jog.publish(self._Vector3(x=0.0, y=0.0, z=0.0))
            self._exec.shutdown(timeout_sec=0.5)
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
