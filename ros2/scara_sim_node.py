#!/usr/bin/env python3
"""
Simulador del robot: se hace pasar por el ESP32 en ROS 2.  Corre el código C real del firmware de emergencia (lazo de 1 kHz,
Home por tiempos, seguridad) contra un modelo de los motores, y publica los mismos tópicos con la misma QoS que src/uros_comms.c.
El panel y RViz no distinguen el robot real del simulado.

    python3 ros2/scara_sim_node.py [--velocidad 1]

Inyección de fallas por /scara_sim/inject (std_msgs/String):
    lock J | free J     traba o libera el eje J (1 o 2)
    power 0 | power 1   corta o devuelve los 12 V de motores (lo que hace el paro físico)
    screw normal | atasco | barrido | no_cae    cómo se porta el siguiente tornillo (el firmware no lo nota: no mide)
"""
import argparse
import ctypes
import os
import subprocess
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
LIB = os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so")
BEST = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
N_STATE = 24
SCREW_MODES = ("normal", "atasco", "barrido", "no_cae")      # mismo orden que sim_tornillo_t en tools/sim/sim_core.h
STEP_S = 0.025                       # el temporizador corre a 40 Hz
STATE_EVERY = 2                      # /scara/state a 20 Hz, como el ESP32
Z_FIJO_MM = 0.0


def load_lib():
    """Compila siempre: es rápido y evita correr el simulador con un firmware viejo."""
    subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
    lib = ctypes.CDLL(LIB)
    lib.sim_goto.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_char_p, ctypes.c_int]
    lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.sim_state.argtypes = [ctypes.POINTER(ctypes.c_float)]
    lib.sim_init()
    return lib


class SimNode(Node):
    def __init__(self, lib, speed):
        super().__init__("scara_esp32", namespace="scara")
        self.lib, self.speed = lib, speed
        self.buf = ctypes.create_string_buffer(240)
        self.state = (ctypes.c_float * N_STATE)()
        self.t_last = time.monotonic()
        self.credit = 0.0
        self.div_state = self.div_ee = 0
        self.pub_state = self.create_publisher(Float32MultiArray, "/scara/state", BEST)
        self.pub_ee = self.create_publisher(Point, "/scara/ee_position", BEST)
        self.pub_event = self.create_publisher(String, "/scara/event", REL)
        self.create_subscription(Point, "/scara/goto", self.on_goto, REL)
        self.create_subscription(Vector3, "/scara/jog", self.on_jog, BEST)
        self.create_subscription(String, "/scara/cmd", self.on_cmd, REL)
        self.create_subscription(String, "/scara_sim/inject", self.on_inject, REL)
        self.create_timer(STEP_S, self.step)
        self.event("#INFO: simulador conectado")

    def event(self, text):
        self.pub_event.publish(String(data=text))

    def reply(self):
        text = self.buf.value.decode("utf-8", "replace")
        if text:
            self.event(text)

    def on_goto(self, m):
        self.lib.sim_goto(m.x, m.y, m.z, self.buf, len(self.buf))
        self.reply()

    @staticmethod
    def _dir(v):
        return (v > 0.5) - (v < -0.5)

    def on_jog(self, m):
        self.lib.sim_jog(int(m.x > 0.5), self._dir(m.y), self._dir(m.z))

    def on_cmd(self, m):
        self.lib.sim_cmd(m.data.encode("utf-8", "replace")[:95], self.buf, len(self.buf))
        self.reply()

    def on_inject(self, m):
        w = m.data.split()
        try:
            if w[0] in ("lock", "free"):
                self.lib.sim_lock(int(w[1]) - 1, int(w[0] == "lock"))
            elif w[0] == "power":
                self.lib.sim_power(int(w[1]))
            elif w[0] == "screw":
                self.lib.sim_screw_mode(SCREW_MODES.index(w[1]))
            else:
                raise ValueError
            self.event("#SIM: " + m.data)
        except (IndexError, ValueError):
            self.event("#ERROR: inyeccion desconocida: " + m.data)

    def step(self):
        now = time.monotonic()
        self.credit += min(now - self.t_last, 0.2) * self.speed * 1000.0    # tras una pausa larga no intenta recuperar todo
        self.t_last = now
        n = int(self.credit)
        self.credit -= n
        # Por tramos cortos: los latidos y órdenes solo llegan entre llamadas a este temporizador.
        self.lib.sim_step_ms(n)
        while self.lib.sim_event_pop(self.buf, len(self.buf)):
            self.reply()
        self.div_state += 1
        if self.div_state >= STATE_EVERY:
            self.div_state = 0
            self.lib.sim_state(self.state)
            self.pub_state.publish(Float32MultiArray(data=list(self.state)))
            self.div_ee += 1
            if self.div_ee >= 4:
                self.div_ee = 0
                self.pub_ee.publish(Point(x=float(self.state[2]), y=float(self.state[3]), z=Z_FIJO_MM))


def main():
    ap = argparse.ArgumentParser(description="Simulador del SCARA (mismo firmware en C, mismos tópicos de ROS 2)")
    ap.add_argument("--velocidad", type=float, default=1.0, help="tiempo simulado por segundo real (1 = tiempo real)")
    a = ap.parse_args()
    rclpy.init()
    node = SimNode(load_lib(), a.velocidad)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:                 # la señal de cierre puede invalidar el contexto mientras espera mensajes
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():               # si la señal de cierre ya cerró el contexto, no hay nada que liberar
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
