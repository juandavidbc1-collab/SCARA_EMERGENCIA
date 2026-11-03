#!/usr/bin/env python3
"""
Prueba de extremo a extremo del simulador sobre ROS 2 real: lanza scara_sim_node y scara_rviz, y maneja el robot
por los mismos tópicos que el panel.   Ejecutar:  python3 tools/sim/test_e2e.py   (con ROS 2 cargado)
"""
import math
import os
import subprocess
import sys
import ctypes
import time
import unittest

import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray, String

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2"))
import scara_sequence as sq  # noqa: E402
BEST = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
HOLD, FAULT, OFF = 2, 6, 0
SIM_SPEED = 10
L1, L2 = 120.0, 70.0


class Client(Node):
    def __init__(self):
        super().__init__("test_e2e")
        self.state, self.js, self.ev, self.markers = None, None, [], None
        self.create_subscription(MarkerArray, "/scara/markers", lambda m: setattr(self, "markers", list(m.markers)), 10)
        self.cmd_p = self.create_publisher(String, "/scara/cmd", REL)
        self.goto_p = self.create_publisher(Point, "/scara/goto", REL)
        self.jog_p = self.create_publisher(Vector3, "/scara/jog", BEST)
        self.inj_p = self.create_publisher(String, "/scara_sim/inject", REL)
        self.create_subscription(Float32MultiArray, "/scara/state", lambda m: setattr(self, "state", list(m.data)), BEST)
        self.create_subscription(String, "/scara/event", lambda m: self.ev.append(m.data), REL)
        self.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", list(m.position)), 10)

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait(self, pred, timeout, what):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if pred():
                return
        raise AssertionError("no ocurrió: %s (estado %s; últimos eventos %s)" % (what, self.state, self.ev[-4:]))

    def cmd(self, text):
        self.cmd_p.publish(String(data=text))


class E2E(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        env = dict(os.environ, ROS_DOMAIN_ID=str(90 + os.getpid() % 100))
        os.environ.update(env)
        cls.procs = [subprocess.Popen([sys.executable, os.path.join(ROOT, "ros2", f)] + args, env=env)
                     for f, args in (("scara_sim_node.py", ["--velocidad", str(SIM_SPEED)]), ("scara_rviz.py", []))]
        cls.addClassCleanup(cls._kill_children)             # también si el arranque falla: no deja procesos huérfanos
        rclpy.init()
        cls.addClassCleanup(rclpy.shutdown)
        cls.n = Client()
        cls.n.wait(lambda: cls.n.state is not None, 20, "llega /scara/state")
        for _ in range(10):                       # da tiempo a que se descubran los suscriptores
            cls.n.spin(0.1)
        cls.n.cmd("home")
        cls.n.wait(lambda: cls.n.state[6] == 1 and cls.n.state[7] == 1 and cls.n.state[4] == HOLD, 40, "Home completo")
        cls.n.spin(0.3)

    @classmethod
    def _kill_children(cls):
        for p in cls.procs:
            p.terminate()
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    def setUp(self):
        self.n.ev.clear()

    def test_01_state_has_the_new_fields(self):
        s = self.n.state
        self.assertEqual(len(s), 24)
        self.assertEqual(int(s[13]) & 3, 3, "J1 y J2 activos")
        self.assertEqual(int(s[13]) & 4, 0, "M3 desactivado al arrancar")
        self.assertEqual(int(s[14]), 6, "control por tiempo")

    def test_02_goto_reaches_target_and_twin_follows(self):
        self.n.goto_p.publish(Point(x=120.0, y=80.0, z=0.0))
        self.n.wait(lambda: any(e.startswith("#LLEGO") for e in self.n.ev), 15, "#LLEGO")
        self.n.spin(0.3)
        s = self.n.state
        self.assertAlmostEqual(s[2], 120.0, delta=1.0)
        self.assertAlmostEqual(s[3], 80.0, delta=1.0)
        q1, q2 = self.n.js
        x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
        y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
        self.assertAlmostEqual(x, 120.0, delta=1.5, msg="el gemelo digital (/joint_states) sigue al robot")
        self.assertAlmostEqual(y, 80.0, delta=1.5)

    def test_03_estop_latches_and_clear_keeps_home(self):
        self.n.cmd("estop")
        self.n.wait(lambda: self.n.state[4] == FAULT and self.n.state[5] == 6, 3, "paro en FAULT")
        self.n.goto_p.publish(Point(x=100.0, y=60.0, z=0.0))
        self.n.spin(0.5)
        self.assertEqual(self.n.state[4], FAULT, "enclavado")
        self.n.cmd("clear")
        self.n.wait(lambda: self.n.state[4] == OFF, 3, "rearme")
        self.assertTrue(self.n.state[6] and self.n.state[7], "conserva el Home")

    def test_04_motor_toggle_is_reported(self):
        self.n.cmd("motor 3 on")
        self.n.wait(lambda: int(self.n.state[13]) & 4, 3, "M3 activo")
        self.n.cmd("tool 50")
        self.n.wait(lambda: abs(self.n.state[15]) > 100, 3, "M3 gira")
        self.n.cmd("motor 3 off")
        self.n.wait(lambda: not int(self.n.state[13]) & 4 and self.n.state[15] == 0, 3, "M3 desactivado y parado")

    def test_05_jog_needs_deadman_and_stops_without_heartbeat(self):
        q0 = self.n.state[0]
        for _ in range(15):
            self.n.jog_p.publish(Vector3(x=0.0, y=1.0, z=0.0))
            self.n.spin(0.04)
        self.assertAlmostEqual(self.n.state[0], q0, delta=0.05, msg="sin dead-man no se mueve")
        for _ in range(25):
            self.n.jog_p.publish(Vector3(x=1.0, y=1.0, z=0.0))
            self.n.spin(0.04)
        self.assertGreater(self.n.state[0], q0 + 2.0, "con dead-man se mueve")
        self.n.wait(lambda: self.n.state[4] == HOLD, 3, "sin latidos se detiene")

    def test_06_rviz_bridge_shows_reference_marker(self):
        self.n.spin(0.3)
        self.assertIsNotNone(self.n.markers, "llegan marcadores del gemelo digital")
        ids = {m.id for m in self.n.markers}
        self.assertIn(12, ids, "marcador de la referencia")

    def test_06b_rviz_shows_boards_and_holes(self):
        self.n.spin(0.3)
        by_id = {m.id: m for m in self.n.markers}
        self.assertEqual(len(by_id[20].points), 4, "las 4 placas")
        self.assertEqual(len(by_id[21].points), 16, "los 16 huecos")
        self.assertAlmostEqual(by_id[20].points[3].y, -0.150, places=3, msg="placa D en el marco de la base")

    def test_06c_full_process_screws_all_holes(self):
        self._run_process_until_done()
        self.assertEqual(self.runner.state, "done", self.runner.reason)
        self.assertEqual(len(self.runner.results), 16)
        self.assertTrue(all(r["resultado"] == "sin_medir" for r in self.runner.results), self.runner.results)
        for r in self.runner.results:
            self.assertAlmostEqual(r["vueltas"], 12.0, delta=0.01, msg=r["hueco"])
        self.assertLess(max(self.errors), 1.0, "error de posición al soltar el tornillo [mm]: %s" % self.errors)
        self.n.wait(lambda: abs(self.n.state[2] - self.seq.process.origin[0]) < 2.0, 5, "vuelve al origen")

    def test_06e_rehearsal_moves_m3_down_and_up_without_screws(self):
        duties, servos = [], []
        n_ev = len(self.n.ev)
        self._run_process_until_done(sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"), ensayo=True), lambda s: (duties.append(s[15]), servos.append(s[22])))
        self.assertEqual(self.runner.state, "done", self.runner.reason)
        self.assertEqual([r["resultado"] for r in self.runner.results], ["ensayado"] * 16)
        self.assertFalse([e for e in self.n.ev[n_ev:] if e.startswith("#TORNILLO")])
        self.assertEqual(max(servos), 0.0, "el servo no se abre")
        self.assertGreater(max(duties), 200.0, "baja")
        self.assertLess(min(duties), -200.0, "sube")

    def _run_process_until_done(self, seq=None, watch=None):
        self.seq = seq or sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"))
        self.n.cmd("motor 3 on")
        self.n.wait(lambda: int(self.n.state[13]) & 4, 3, "M3 activo")
        self.errors = []
        # el reloj va tan rápido como el simulador: si no, las subidas de M3 tardarían 10 veces más de lo que pasa en el robot
        self.runner = sq.SequenceRunner(self.seq.steps, 1, self.n.cmd, lambda: time.monotonic() * SIM_SPEED)
        seen = len(self.n.ev)
        self.runner.start()
        last = None
        t0 = time.time()
        while self.runner.running and time.time() - t0 < 150:
            rclpy.spin_once(self.n, timeout_sec=0.005)
            for e in self.n.ev[seen:]:
                self.runner.on_event(e)
            seen = len(self.n.ev)
            self.runner.tick()
            if watch:
                watch(self.n.state)
            step = self.runner.steps[self.runner.index] if self.runner.running else None
            if step and step is not last:
                if step.get("text") == "servo abrir":
                    g = self.runner.steps[self.runner.index - 2]
                    self.errors.append(math.hypot(self.n.state[2] - g["x"], self.n.state[3] - g["y"]))
                last = step

    def test_07_locked_axis_is_found_when_recorrecting(self):
        self.n.cmd("ang 60 40")
        self.n.wait(lambda: any(e.startswith("#LLEGO") for e in self.n.ev), 10, "#LLEGO")
        self.n.ev.clear()
        self.n.inj_p.publish(String(data="lock 1"))
        self.n.spin(0.2)
        self.n.cmd("ref")
        self.n.wait(lambda: self.n.state[4] == FAULT, 20, "eje trabado -> FAULT al recorrer")
        self.assertFalse(self.n.state[6], "pierde el Home")
        self.n.inj_p.publish(String(data="free 1"))
        self.n.cmd("clear")
        self.n.wait(lambda: self.n.state[4] == OFF, 3, "clear")


class RehearsalDrift(unittest.TestCase):
    """Sin ROS, con el firmware real: M3 debe volver a la misma altura tras los 16 huecos."""

    def test_m3_returns_to_the_start_after_all_holes(self):
        subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
        lib = ctypes.CDLL(os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so"))
        lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_state.argtypes = [ctypes.POINTER(ctypes.c_float)]
        lib.sim_m3_real.argtypes = [ctypes.POINTER(ctypes.c_double)]
        buf, st, real = ctypes.create_string_buffer(240), (ctypes.c_float * 24)(), (ctypes.c_double * 3)()
        events = []

        def cmd(text):
            lib.sim_cmd(text.encode(), buf, len(buf))

        def run(ms):
            lib.sim_step_ms(ms)
            while lib.sim_event_pop(buf, len(buf)):
                events.append(buf.value.decode())

        lib.sim_init()
        cmd("motor 3 on")
        run(20)
        cmd("home")
        for _ in range(4000):
            run(20)
            lib.sim_state(st)
            if st[6] and st[7] and st[4] == 2:
                break
        now = [0.0]
        seq = sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"), ensayo=True)
        r = sq.SequenceRunner(seq.steps, 1, cmd, lambda: now[0])
        r.start()
        seen, angle = 0, 0.0
        while r.running and now[0] < 1500:
            run(1)
            now[0] += 0.001
            for e in events[seen:]:
                r.on_event(e)
            seen = len(events)
            r.tick()
            lib.sim_m3_real(real)
            angle += real[2] * 0.001
        self.assertEqual(r.state, "done", r.reason)
        self.assertLess(abs(angle) / (2 * math.pi), 0.05, "vueltas de M3 que no volvieron")


class M3Height(unittest.TestCase):
    def test_a_screw_still_tightens_with_m3_above_its_starting_point(self):
        subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
        lib = ctypes.CDLL(os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so"))
        lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
        buf = ctypes.create_string_buffer(240)
        events = []

        def cmd(text):
            lib.sim_cmd(text.encode(), buf, len(buf))

        def run(ms):
            lib.sim_step_ms(ms)
            while lib.sim_event_pop(buf, len(buf)):
                events.append(buf.value.decode())

        lib.sim_init()
        cmd("motor 3 on")
        run(20)
        for _ in range(4):                  # 20 s subiendo: el ángulo de M3 queda negativo
            cmd("tool -35")
            run(5000)
        cmd("tool 0")
        run(500)
        cmd("servo abrir")
        run(400)
        cmd("servo cerrar")
        run(600)
        cmd("screw 12 0.4")
        for _ in range(60):
            run(1000)
            if any(e.startswith("#TORNILLO") for e in events):
                break
        self.assertEqual(len([e for e in events if e.startswith("#TORNILLO")]), 1, events)
        lib.sim_m3_real.argtypes = [ctypes.POINTER(ctypes.c_double)]
        real = (ctypes.c_double * 3)()
        lib.sim_m3_real(real)
        self.assertGreater(real[1], 0.2, "par real sobre el tornillo [N·m]")

    def test_m3_comes_back_up_after_each_real_screw(self):
        subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
        lib = ctypes.CDLL(os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so"))
        lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_state.argtypes = [ctypes.POINTER(ctypes.c_float)]
        lib.sim_m3_real.argtypes = [ctypes.POINTER(ctypes.c_double)]
        buf, st, real = ctypes.create_string_buffer(240), (ctypes.c_float * 24)(), (ctypes.c_double * 3)()
        events = []

        def cmd(text):
            lib.sim_cmd(text.encode(), buf, len(buf))

        def run(ms):
            lib.sim_step_ms(ms)
            while lib.sim_event_pop(buf, len(buf)):
                events.append(buf.value.decode())

        lib.sim_init()
        cmd("motor 3 on")
        run(20)
        cmd("home")
        for _ in range(4000):
            run(20)
            lib.sim_state(st)
            if st[6] and st[7] and st[4] == 2:
                break
        now = [0.0]
        seq = sq.load_sequence_full(os.path.join(ROOT, "ros2", "secuencia.yaml"))
        r = sq.SequenceRunner(seq.steps, 1, cmd, lambda: now[0])
        r.start()
        seen, angle = 0, 0.0
        while r.running and now[0] < 1500:
            run(5)
            now[0] += 0.005
            for e in events[seen:]:
                r.on_event(e)
            seen = len(events)
            r.tick()
            lib.sim_m3_real(real)
            angle += real[2] * 0.005
        self.assertEqual(r.state, "done", r.reason)
        self.assertEqual(len(r.results), 16)
        turns = abs(angle) / (2 * math.pi)
        print("  vueltas de M3 que no volvieron en 16 huecos: %.2f (%.2f mm), proceso %.0f s" % (turns, turns * 0.5, now[0]))
        self.assertLess(turns, 8.0, "menos de media vuelta por hueco")


if __name__ == "__main__":
    unittest.main(verbosity=1)
