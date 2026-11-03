#!/usr/bin/env python3
"""
Cliente ROS 2 (Jazzy) del SCARA: consigna (x, y, z) desde Python hacia el ESP32 vía micro-ROS.

Requisitos:  Ubuntu con ROS 2 Jazzy;  source /opt/ros/jazzy/setup.bash;  el agente micro-ROS corriendo
(ver UBUNTU_MICROROS.md).  No necesita instalar nada más.

    python3 ros2/scara_client.py home                    Home (espera a que termine)
    python3 ros2/scara_client.py goto 120 80 [0]         consigna en el espacio de la tarea [mm]
    python3 ros2/scara_client.py ang 20 90               consigna articular [grados]
    python3 ros2/scara_client.py cmd "gain"              cualquier comando de /scara/cmd (ver 'help')
    python3 ros2/scara_client.py stop | off | clear
    python3 ros2/scara_client.py state                   imprime una muestra de /scara/state
    python3 ros2/scara_client.py monitor                 imprime eventos y posición hasta Ctrl+C

Tópicos (nodo del ESP32: /scara/...):
    /scara/goto (geometry_msgs/Point)   /scara/jog (geometry_msgs/Vector3, best-effort)
    /scara/cmd (std_msgs/String)        /scara/state (Float32MultiArray)
    /scara/ee_position (Point)          /scara/event (std_msgs/String)
"""
import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String

STATE_FIELDS = ["q1_deg", "q2_deg", "x_mm", "y_mm", "modo", "falla", "home1", "home2", "fc_bits",
                "qref1_deg", "qref2_deg", "duty1", "duty2"]
MODES = ["OFF", "HOMING", "HOLD", "MOVE", "JOG", "PULSE", "FAULT"]
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)

# Qué evento da por terminada una orden:
#   movimientos (goto / ang / home): "#LLEGO" (llegó) o un error/falla. Los "#INFO", "#OK" y "#HOME" son intermedios.
#   órdenes instantáneas (stop, off, clear, cmd): la primera respuesta del intérprete.
MOVE_DONE = ("#LLEGO", "#ERROR", "#FALLA", "#err")
QUICK_DONE = ("#OK", "#INFO", "#ERROR", "#FALLA", "#err")


class ScaraClient(Node):
    def __init__(self):
        super().__init__("scara_client")
        self.pub_goto = self.create_publisher(Point, "/scara/goto", RELIABLE)
        self.pub_cmd = self.create_publisher(String, "/scara/cmd", RELIABLE)
        self.events = []
        self.state = None
        self.ee = None
        self.create_subscription(String, "/scara/event", lambda m: self.events.append(m.data), RELIABLE)
        self.create_subscription(Float32MultiArray, "/scara/state", self._on_state, BEST_EFFORT)
        self.create_subscription(Point, "/scara/ee_position", self._on_ee, BEST_EFFORT)

    def _on_state(self, m):
        self.state = list(m.data)

    def _on_ee(self, m):
        self.ee = (m.x, m.y, m.z)

    def spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_robot(self, timeout=8.0):
        """Espera a que el ESP32 esté conectado (suscriptores en /scara/goto y /scara/cmd, y llegue estado)."""
        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state is not None and self.pub_goto.get_subscription_count() > 0:
                return True
        return False

    def wait_event(self, timeout, terminal=MOVE_DONE, verbose=True):
        """Imprime los eventos que llegan; devuelve el que termina la orden (o None si vence el tiempo)."""
        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            while self.events:
                ev = self.events.pop(0)
                if verbose:
                    print(ev)
                if ev.startswith(terminal):
                    return ev
        return None

    def send_goto(self, x, y, z):
        m = Point(x=float(x), y=float(y), z=float(z))
        self.pub_goto.publish(m)

    def send_cmd(self, text):
        self.pub_cmd.publish(String(data=text))


def fmt_state(s):
    d = dict(zip(STATE_FIELDS, s))
    mode = MODES[int(d["modo"])] if 0 <= int(d["modo"]) < len(MODES) else "?"
    return ("[%-6s] q1=%8.2f q2=%8.2f deg | x=%7.1f y=%7.1f mm | home %d%d | falla %d | topes 0b%s" %
            (mode, d["q1_deg"], d["q2_deg"], d["x_mm"], d["y_mm"], d["home1"], d["home2"],
             d["falla"], format(int(d["fc_bits"]), "04b")))


def main():
    ap = argparse.ArgumentParser(description="Cliente ROS 2 del SCARA (micro-ROS)")
    ap.add_argument("accion", choices=["home", "goto", "ang", "cmd", "stop", "off", "clear", "state", "monitor"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--timeout", type=float, default=40.0, help="segundos máximos de espera del movimiento")
    a = ap.parse_args()

    rclpy.init()
    node = ScaraClient()
    try:
        if not node.wait_for_robot():
            print("No llega estado del ESP32. ¿Está corriendo el agente micro-ROS y el robot conectado?", file=sys.stderr)
            return 2
        node.events.clear()          # descarta eventos viejos

        if a.accion == "goto":
            if len(a.args) not in (2, 3):
                ap.error("goto necesita x y [z]")
            x, y = a.args[0], a.args[1]
            z = a.args[2] if len(a.args) == 3 else 0.0
            node.send_goto(x, y, z)
            ev = node.wait_event(a.timeout)
        elif a.accion == "home":
            node.send_cmd("home")
            ev = node.wait_event(max(a.timeout, 90.0))
        elif a.accion == "ang":
            if len(a.args) != 2:
                ap.error("ang necesita q1 q2")
            node.send_cmd("ang %s %s" % (a.args[0], a.args[1]))
            ev = node.wait_event(a.timeout)
        elif a.accion == "cmd":
            node.send_cmd(" ".join(a.args))
            ev = node.wait_event(5.0, QUICK_DONE)
        elif a.accion in ("stop", "off", "clear"):
            node.send_cmd(a.accion)
            ev = node.wait_event(5.0, QUICK_DONE)
        elif a.accion == "state":
            node.spin_for(0.3)
            print(fmt_state(node.state))
            return 0
        else:  # monitor
            print("Ctrl+C para salir")
            last = 0.0
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                while node.events:
                    print("\n" + node.events.pop(0))
                if node.state and time.time() - last > 0.25:
                    last = time.time()
                    sys.stdout.write("\r" + fmt_state(node.state) + "   ")
                    sys.stdout.flush()
            return 0

        if ev is None:
            print("(sin respuesta final en el tiempo esperado)", file=sys.stderr)
            return 1
        return 1 if ev.startswith(("#ERROR", "#FALLA", "#err")) else 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
