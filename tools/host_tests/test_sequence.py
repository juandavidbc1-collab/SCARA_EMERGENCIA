import math
"""Pruebas de la secuencia repetitiva, sin ROS.  Ejecutar:  python3 -m unittest tools/host_tests/test_sequence.py"""
import os
import sys
import tempfile
import textwrap
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
import scara_sequence as sq  # noqa: E402

ARRIVED = "#LLEGO: objetivo x=1 y=2"


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def make(steps, repeat=1, timeout=20.0):
    sent, clock = [], Clock()
    return sq.SequenceRunner(steps, repeat, sent.append, clock, step_timeout=timeout), sent, clock


GOTO_A = {"kind": "goto", "x": 120.0, "y": 80.0}
GOTO_B = {"kind": "goto", "x": 100.0, "y": -60.0}


class TestRunner(unittest.TestCase):
    def test_runs_steps_in_order_and_waits_for_arrival(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        self.assertEqual(sent, ["goto 120 80"])
        r.tick()
        self.assertEqual(sent, ["goto 120 80"], "no manda el siguiente punto antes de llegar")
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(sent, ["goto 120 80", "goto 100 -60"])
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(r.state, "done")

    def test_repeats_the_requested_number_of_cycles(self):
        r, sent, _ = make([GOTO_A], repeat=3)
        r.start()
        for _ in range(3):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(len(sent), 3)
        self.assertEqual(r.state, "done")
        self.assertEqual(r.cycles_done, 3)

    def test_zero_repeat_runs_until_stopped(self):
        r, sent, _ = make([GOTO_A], repeat=0)
        r.start()
        for _ in range(50):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(r.state, "running")
        self.assertEqual(len(sent), 51)
        r.stop()
        self.assertEqual(r.state, "stopped")
        self.assertEqual(sent[-1], "stop")

    def test_wait_step_uses_the_clock(self):
        r, sent, clk = make([{"kind": "wait", "secs": 2.0}, GOTO_A])
        r.start()
        clk.t += 1.9
        r.tick()
        self.assertEqual(sent, [])
        clk.t += 0.2
        r.tick()
        self.assertEqual(sent, ["goto 120 80"])

    def test_tool_step_runs_for_its_time_renews_and_stops(self):
        r, sent, clk = make([{"kind": "tool", "pct": 40.0, "secs": 12.0}])
        r.start()
        self.assertEqual(sent, ["tool 40"])
        clk.t += sq.TOOL_RENEW_S + 0.1
        r.tick()
        self.assertEqual(sent, ["tool 40", "tool 40"], "renueva antes de que M3 se apague solo")
        clk.t += 12.0
        r.tick()
        self.assertEqual(sent[-1], "tool 0")
        self.assertEqual(r.state, "done")

    def test_fault_aborts_and_sends_nothing_more(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        r.on_event("#FALLA: motor trabado")
        for _ in range(5):
            r.on_event(ARRIVED)
            r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent, ["goto 120 80"])
        self.assertIn("trabado", r.reason)

    def test_rejected_point_aborts(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.start()
        r.on_event("#ERROR: no se puede ir a (120, 80)")
        r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent, ["goto 120 80"])

    def test_timeout_aborts_and_stops_the_robot(self):
        r, sent, clk = make([GOTO_A], timeout=5.0)
        r.start()
        clk.t += 5.1
        r.tick()
        self.assertEqual(r.state, "aborted")
        self.assertEqual(sent[-1], "stop")
        self.assertIn("tiempo", r.reason)

    def test_abort_during_tool_step_turns_the_tool_off(self):
        r, sent, _ = make([{"kind": "tool", "pct": 40.0, "secs": 5.0}])
        r.start()
        r.on_event("#ERROR: algo")
        self.assertEqual(sent[-1], "tool 0")

    def test_old_arrival_before_start_is_ignored(self):
        r, sent, _ = make([GOTO_A, GOTO_B])
        r.on_event(ARRIVED)
        r.start()
        r.tick()
        self.assertEqual(sent, ["goto 120 80"])

    def test_stop_when_idle_sends_nothing(self):
        r, sent, _ = make([GOTO_A])
        r.stop()
        self.assertEqual(sent, [])

    def test_cycle_time_is_recorded(self):
        r, _, clk = make([GOTO_A], repeat=2)
        r.start()
        clk.t += 3.0
        r.on_event(ARRIVED)
        r.tick()
        self.assertEqual(r.cycle_times, [3.0])


class TestLoad(unittest.TestCase):
    def load(self, text):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(textwrap.dedent(text))
        self.addCleanup(os.unlink, f.name)
        return sq.load_sequence(f.name)

    def test_valid_file(self):
        steps, repeat = self.load("""
            repeticiones: 4
            pasos:
              - ir: [120, 80]
              - esperar: 0.5
              - angulos: [20, 90]
              - m3: {potencia: 40, tiempo: 1.5}
        """)
        self.assertEqual(repeat, 4)
        self.assertEqual([s["kind"] for s in steps], ["goto", "wait", "ang", "tool"])
        self.assertEqual(steps[3]["pct"], 40)

    def test_unreachable_point_names_the_step(self):
        with self.assertRaisesRegex(sq.SequenceError, "paso 2.*alcance"):
            self.load("""
                pasos:
                  - ir: [120, 80]
                  - ir: [400, 0]
            """)

    def test_singular_point_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "singular"):
            self.load("pasos:\n  - ir: [190, 0]\n")

    def test_unknown_step_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "bailar"):
            self.load("pasos:\n  - bailar: 3\n")

    def test_empty_sequence_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "pasos"):
            self.load("repeticiones: 1\npasos: []\n")

    def test_tool_power_out_of_range_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "potencia"):
            self.load("pasos:\n  - m3: {potencia: 150, tiempo: 1}\n")

    def test_negative_repeat_is_rejected(self):
        with self.assertRaisesRegex(sq.SequenceError, "repeticiones"):
            self.load("repeticiones: -1\npasos:\n  - esperar: 1\n")

    def test_bad_yaml_is_reported_not_raised_raw(self):
        with self.assertRaises(sq.SequenceError):
            self.load("pasos: [\n")

    def test_shipped_example_is_valid(self):
        path = os.path.join(os.path.dirname(os.path.abspath(sq.__file__)), "secuencia.yaml")
        steps, _ = sq.load_sequence(path)
        self.assertGreaterEqual(len(steps), 2)


ROS2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2")
PROCESO = """
repeticiones: 2
proceso:
  base_cm: [15, 0]
  origen_cm: [14.5, -20]
  codo: -1
  placas_cm: {A: [29.1421, -5], D: [15, -15]}
  lado_mm: 38
  hueco_borde_mm: 3.5
  servo: {abrir_s: 0.4, caida_s: 0.6}
  tornillo: {vueltas: 12, torque_nm: 0.4}
"""


ASSIGNED = os.path.join(ROS2, "secuencia.yaml")


class TestProcess(unittest.TestCase):
    def _load(self, text):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(text)
        self.addCleanup(os.unlink, f.name)
        return sq.load_sequence_full(f.name)

    def test_assigned_file_has_sixteen_holes_on_four_boards(self):
        seq = sq.load_sequence_full(os.path.join(ROS2, "secuencia.yaml"))
        self.assertEqual(len(seq.process.holes), 16)
        self.assertEqual(sorted(seq.process.boards), ["A", "B", "C", "D"])
        self.assertEqual(sum(s["kind"] == "screw" for s in seq.steps), 16)
        self.assertEqual(seq.repeat, 1)

    def test_assigned_points_are_measured_from_the_base(self):
        seq = sq.load_sequence_full(os.path.join(ROS2, "secuencia.yaml"))
        self.assertAlmostEqual(seq.process.boards["D"][0], 0.0, places=6)
        self.assertAlmostEqual(seq.process.boards["D"][1], -150.0, places=6)
        self.assertAlmostEqual(seq.process.boards["A"][0], 141.421, places=3)
        self.assertAlmostEqual(seq.process.boards["A"][1], -50.0, places=6)

    def test_holes_are_at_the_measured_offset_from_the_board_center(self):
        seq = self._load(PROCESO)
        h = {x["label"]: x for x in seq.process.holes}
        self.assertAlmostEqual(h["A-H1"]["x"], 141.421 - 15.5, places=3)
        self.assertAlmostEqual(h["A-H1"]["y"], -50.0 - 15.5, places=6)
        self.assertAlmostEqual(h["A-H4"]["x"], 141.421 + 15.5, places=3)
        self.assertAlmostEqual(h["A-H4"]["y"], -50.0 + 15.5, places=6)

    def test_process_order_is_move_drop_then_screw(self):
        seq = self._load(PROCESO)
        kinds = [s["kind"] for s in seq.steps]
        first_hole = kinds.index("screw")
        self.assertEqual(kinds[:3], ["cmd", "ref", "goto"], "codo, recorrer los ejes y origen")
        self.assertEqual(kinds[3:first_hole + 1], ["goto", "wait", "cmd", "wait", "cmd", "wait", "screw"])
        self.assertEqual(seq.steps[5]["text"], "servo abrir")
        self.assertEqual(seq.steps[7]["text"], "servo cerrar")
        self.assertEqual(kinds[-1], "goto", "termina en el origen")
        self.assertEqual((seq.steps[-1]["x"], seq.steps[-1]["y"]), (seq.steps[2]["x"], seq.steps[2]["y"]))

    def test_m3_goes_up_after_each_screw_before_the_arm_moves(self):
        seq = sq.load_sequence_full(ASSIGNED)
        for i, s in enumerate(seq.steps):
            if s["kind"] == "screw":
                pause, up, settle, after = seq.steps[i + 1:i + 5]
                self.assertEqual(pause["kind"], "wait")
                self.assertEqual((up["kind"], up["pct"], up["secs"], up["label"]), ("tool", -35.0, 18.5, s["label"]))
                self.assertEqual(settle["kind"], "wait")
                self.assertIn(after["kind"], ("goto", "ref"))

    def test_axes_are_recorrected_at_the_start_and_every_four_holes(self):
        kinds = [s["kind"] for s in sq.load_sequence_full(ASSIGNED).steps]
        refs = [kinds[:i].count("screw") for i, k in enumerate(kinds) if k == "ref"]
        self.assertEqual(refs, [0, 4, 8, 12])

    def test_rehearsal_also_recorrects_the_axes(self):
        kinds = [s["kind"] for s in sq.load_sequence_full(ASSIGNED, ensayo=True).steps]
        self.assertEqual(kinds.count("ref"), 4)

    def test_unreachable_origin_is_moved_and_reported(self):
        seq = self._load(PROCESO)
        g = seq.steps[2]
        self.assertAlmostEqual(math.hypot(g["x"], g["y"]), sq.ORIGEN_RADIO_MM, places=6)
        self.assertAlmostEqual(math.atan2(g["y"], g["x"]), math.atan2(-200.0, -5.0), places=6)
        self.assertTrue(any("origen" in n for n in seq.notes), seq.notes)

    def test_unreachable_hole_is_rejected_with_its_name(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._load(PROCESO.replace("base_cm: [15, 0]", "base_cm: [14.5, -20]").replace("origen_cm: [14.5, -20]", "origen_cm: [22.5, -5]"))
        self.assertIn("A-H", str(e.exception))

    def test_joint_limits_are_checked_for_the_requested_elbow(self):
        with self.assertRaises(sq.SequenceError) as e:
            self._load(PROCESO.replace("codo: -1", "codo: 1"))
        self.assertIn("D-H", str(e.exception))
        self.assertIn("codo", str(e.exception))

    def test_keeps_one_elbow_in_every_hole(self):
        seq = sq.load_sequence_full(os.path.join(ROS2, "secuencia.yaml"))
        for h in seq.process.holes:
            self.assertIsNone(sq.point_problem(h["x"], h["y"], elbow=seq.process.elbow), h["label"])

    def test_old_format_still_loads(self):
        steps, repeat = sq.load_sequence(self._write_old())
        self.assertEqual(repeat, 3)
        self.assertEqual(steps[0]["kind"], "goto")

    def _write_old(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write("repeticiones: 3\npasos:\n  - ir: [120, 80]\n  - servo: {accion: abrir, tiempo: 0.5}\n  - atornillar: {vueltas: 8, torque: 0.3}\n  - codo: -1\n")
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_step_forms_in_pasos(self):
        steps, _ = sq.load_sequence(self._write_old())
        self.assertEqual(steps[1], {"kind": "cmd", "text": "servo abrir", "secs": 0.5, "desc": "Servo: abrir"})
        self.assertEqual((steps[2]["kind"], steps[2]["vueltas"], steps[2]["torque"]), ("screw", 8.0, 0.3))
        self.assertEqual(steps[3]["text"], "elbow -1")


class TestRehearsal(unittest.TestCase):
    def _load(self, extra=""):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(PROCESO + extra)
        self.addCleanup(os.unlink, f.name)
        return sq.load_sequence_full(f.name, ensayo=True)

    def test_visits_the_same_holes_in_the_same_order(self):
        seq = self._load()
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(PROCESO)
        self.addCleanup(os.unlink, f.name)
        real = sq.load_sequence_full(f.name)
        goto = lambda st: [(s["x"], s["y"]) for s in st if s["kind"] == "goto"]
        self.assertEqual(goto(seq.steps), goto(real.steps))

    def test_never_touches_servo_or_screw(self):
        seq = self._load()
        self.assertFalse([s for s in seq.steps if s["kind"] == "screw"])
        self.assertFalse([s for s in seq.steps if s["kind"] == "cmd" and "servo" in s["text"]])

    def test_each_hole_goes_down_then_up_for_the_same_time(self):
        seq = self._load()
        tools = [s for s in seq.steps if s["kind"] == "tool"]
        self.assertEqual(len(tools), 2 * len(seq.process.holes))
        down, up = tools[0], tools[1]
        self.assertEqual((down["label"], up["label"]), ("A-H1", "A-H1"))
        self.assertEqual((down["pct"], up["pct"]), (40.0, -40.0))
        self.assertEqual((down["secs"], up["secs"]), (3.5, 3.5))

    def test_stops_at_the_bottom_before_going_up(self):
        seq = self._load()
        i = next(k for k, s in enumerate(seq.steps) if s["kind"] == "tool")
        pause = seq.steps[i + 1]
        self.assertEqual((pause["kind"], pause["label"]), ("wait", "A-H1"))
        self.assertLess(seq.steps[i + 2]["pct"], 0)

    def test_stop_during_the_pause_at_the_bottom_is_reported(self):
        self.t, self.sent = 0.0, []
        r = sq.SequenceRunner([{"kind": "wait", "secs": 1.0, "label": "A-H1", "desc": "p"}], 1, self.sent.append, lambda: self.t)
        r.start()
        r.stop()
        self.assertTrue(r.tool_left_down)

    def test_stopping_a_screw_warns_that_m3_is_down(self):
        self.t, self.sent = 0.0, []
        r = sq.SequenceRunner([{"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H1", "desc": "s"}], 1, self.sent.append, lambda: self.t)
        r.start()
        r.stop()
        self.assertTrue(r.tool_left_down)

    def test_waits_after_going_up_before_moving(self):
        seq = self._load()
        i = next(k for k, s in enumerate(seq.steps) if s["kind"] == "tool" and s["pct"] < 0)
        self.assertEqual(seq.steps[i + 1]["kind"], "wait")
        self.assertEqual(seq.steps[i + 2]["kind"], "goto")

    def test_time_and_power_come_from_the_file(self):
        seq = self._load("  ensayo: {potencia: 25, bajar_s: 3}\n")
        down = next(s for s in seq.steps if s["kind"] == "tool")
        self.assertEqual((down["pct"], down["secs"]), (25.0, 3.0))

    def test_power_out_of_range_is_rejected(self):
        with self.assertRaises(sq.SequenceError):
            self._load("  ensayo: {potencia: 150, bajar_s: 3}\n")

    def test_normal_load_ignores_the_rehearsal_block(self):
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
            f.write(PROCESO + "  ensayo: {potencia: 25, bajar_s: 3}\n")
        self.addCleanup(os.unlink, f.name)
        self.assertEqual(sum(s["kind"] == "screw" for s in sq.load_sequence_full(f.name).steps), 8)

    def _run(self):
        self.t = 0.0
        self.sent = []
        steps = [{"kind": "tool", "pct": 40.0, "secs": 3.5, "label": "A-H1", "desc": "b"},
                 {"kind": "tool", "pct": -40.0, "secs": 3.5, "label": "A-H1", "fin": True, "desc": "s"},
                 {"kind": "wait", "secs": 1.0, "desc": "w"}]
        r = sq.SequenceRunner(steps, 1, self.sent.append, lambda: self.t)
        r.start()
        return r

    def test_runner_goes_down_up_and_records_the_hole(self):
        r = self._run()
        self.assertEqual(self.sent, ["tool 40"])
        self.t = 3.5; r.tick()
        self.assertEqual(self.sent, ["tool 40", "tool 0", "tool -40"])
        self.assertEqual(r.results, [])
        self.t = 7.0; r.tick()
        self.assertEqual(self.sent[-1], "tool 0")
        self.assertEqual(r.results, [{"hueco": "A-H1", "resultado": "ensayado", "ciclo": 1}])

    def test_interrupted_while_down_is_reported(self):
        r = self._run()
        self.t = 1.0; r.tick()
        r.stop()
        self.assertTrue(r.tool_left_down)
        self.assertEqual(self.sent[-2:], ["tool 0", "stop"], "no sube solo")

    def test_finished_hole_is_not_reported_as_down(self):
        r = self._run()
        self.t = 3.5; r.tick(); self.t = 7.0; r.tick()
        r.stop()
        self.assertFalse(r.tool_left_down)


class TestLimitesFirmware(unittest.TestCase):
    def _punto(self, q1, q2):
        a, b = math.radians(q1), math.radians(q1 + q2)
        return sq.L1_MM * math.cos(a) + sq.L2_MM * math.cos(b), sq.L1_MM * math.sin(a) + sq.L2_MM * math.sin(b)

    def test_j1_uses_the_same_limit_as_the_firmware(self):
        self.assertIsNone(sq.point_problem(*self._punto(116.0, -60.0), elbow=-1))
        self.assertIsNotNone(sq.point_problem(*self._punto(118.0, -60.0), elbow=-1), "el firmware rechaza q1 mayor que 117")


class TestRefStep(unittest.TestCase):
    def setUp(self):
        self.t, self.sent = 0.0, []
        self.r = sq.SequenceRunner([{"kind": "ref", "desc": "r"}, GOTO_A], 1, self.sent.append, lambda: self.t)

    def test_waits_for_arrival(self):
        self.r.start()
        self.assertEqual(self.sent, ["ref"])
        self.t = 20.0; self.r.tick()
        self.assertEqual(self.r.index, 0)
        self.r.on_event("#LLEGO: ejes recorridos"); self.r.tick()
        self.assertEqual(self.r.index, 1)

    def test_has_its_own_timeout(self):
        self.r.start()
        self.t = sq.STEP_TIMEOUT_S + 1; self.r.tick()
        self.assertEqual(self.r.state, "running", "recorrer tarda más que un movimiento")
        self.t = sq.REF_TIMEOUT_S + 1; self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent[-1], "stop")

    def test_lost_arm_aborts(self):
        self.r.start()
        self.r.on_event("#FALLA: J1 se perdio 12 grados")
        self.assertEqual(self.r.state, "aborted")
        self.assertIn("perdio", self.r.reason)

    def test_can_be_written_in_pasos(self):
        self.assertEqual(sq._parse_step({"recorrer": True})["kind"], "ref")


TORNILLO_OK = "#TORNILLO: sin_medir vueltas=12.00 prof=6.00mm avance=100% torque=0.000Nm fuerza=0N presion=0.0MPa corriente=0.00A t=23.2s"


class TestScrewRunner(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.sent = []
        self.steps = [{"kind": "cmd", "text": "servo abrir", "secs": 0.5, "desc": "s"},
                      {"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H1", "desc": "s"},
                      {"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H3", "desc": "s"}]
        self.r = sq.SequenceRunner(self.steps, 1, self.sent.append, lambda: self.t)

    def test_parse(self):
        d = sq.parse_tornillo(TORNILLO_OK)
        self.assertEqual(d["resultado"], "sin_medir")
        self.assertEqual((d["vueltas"], d["prof"], d["avance"], d["t"]), (12.0, 6.0, 100.0, 23.2))
        self.assertIsNone(sq.parse_tornillo("#LLEGO: q1=1"))

    def test_command_step_waits_its_time(self):
        self.r.start()
        self.assertEqual(self.sent, ["servo abrir"])
        self.t = 0.4; self.r.tick()
        self.assertEqual(self.r.index, 0)
        self.t = 0.5; self.r.tick()
        self.assertEqual(self.r.index, 1)
        self.assertEqual(self.sent[-1], "screw 12 0.4")

    def test_screw_ok_records_result_and_continues(self):
        self.r.start(); self.t = 1.0; self.r.tick()
        self.r.on_event(TORNILLO_OK); self.r.tick()
        self.assertEqual(self.r.index, 2)
        self.assertEqual(self.r.results[0]["hueco"], "A-H1")
        self.assertEqual(self.r.results[0]["ciclo"], 1)
        self.assertEqual(self.r.results[0]["resultado"], "sin_medir")
        self.r.on_event(TORNILLO_OK); self.r.tick()
        self.assertEqual(self.r.state, "done")
        self.assertEqual([x["hueco"] for x in self.r.results], ["A-H1", "A-H3"])

    def test_bad_screw_aborts_and_keeps_the_record(self):
        self.r = sq.SequenceRunner(self.steps[1:], 1, self.sent.append, lambda: self.t)
        self.r.start(); self.t = 1.0; self.r.tick()
        n = len(self.sent)
        self.r.on_event(TORNILLO_OK.replace(": sin_medir", ": cancelado")); self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertIn("A-H1", self.r.reason)
        self.assertIn("cancelado", self.r.reason)
        self.assertEqual(self.r.results[0]["resultado"], "cancelado")
        self.assertEqual(len(self.sent), n, "no manda mas ordenes")

    def test_screw_timeout_aborts_and_stops_m3(self):
        self.r.start(); self.t = 1.0; self.r.tick()
        self.t = 1.0 + sq.STEP_TIMEOUT_S + 1; self.r.tick()
        self.assertEqual(self.r.state, "running", "apretar por tiempo tarda más que un movimiento")
        self.t = 1.0 + sq.SCREW_TIMEOUT_S + 1; self.r.tick()
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent[-1], "stop")

    def test_stop_in_servo_step_closes_servo(self):
        self.r.start()
        self.r.stop()
        self.assertEqual(self.sent, ["servo abrir", "servo cerrar", "stop"])

    def test_fault_during_screw_aborts_without_commands(self):
        self.r = sq.SequenceRunner(self.steps[1:], 1, self.sent.append, lambda: self.t)
        self.r.start(); self.t = 1.0; self.r.tick()
        n = len(self.sent)
        self.r.on_event("#FALLA: PARO DE EMERGENCIA.")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(len(self.sent), n)


class TestServoCierra(unittest.TestCase):
    """El servo abre en un paso de 0 s y lo que sigue es la espera: cortar ahí no debe dejarlo soltando tornillos."""
    def setUp(self):
        self.t = 0.0
        self.sent = []
        steps = [sq._cmd("servo abrir", "abrir"), sq._wait(0.4, "cayendo"), sq._cmd("servo cerrar", "cerrar"), sq._wait(0.6, "cayendo")]
        self.r = sq.SequenceRunner(steps, 1, self.sent.append, lambda: self.t)
        self.r.start()
        self.r.tick()                    # ya abrió y está en la espera
        self.assertEqual(self.r.index, 1)
        self.sent.clear()

    def test_stop_during_the_wait_closes_it(self):
        self.r.stop()
        self.assertEqual(self.sent, ["servo cerrar", "stop"])

    def test_fault_during_the_wait_closes_it(self):
        self.r.on_event("#FALLA: algo")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent, ["servo cerrar"])

    def test_error_during_the_wait_closes_it(self):
        self.r.on_event("#ERROR: algo")
        self.assertEqual(self.r.state, "aborted")
        self.assertEqual(self.sent, ["servo cerrar"])

    def test_nothing_extra_once_it_is_closed(self):
        self.t = 0.4; self.r.tick()      # termina la espera y cierra
        self.r.tick()                    # pasa a la segunda espera
        self.assertEqual(self.r.index, 3)
        self.sent.clear()
        self.r.stop()
        self.assertEqual(self.sent, ["stop"])

    def test_an_angle_step_also_counts_as_open(self):
        sent = []
        r = sq.SequenceRunner([sq._parse_step({"servo": {"accion": 90, "tiempo": 0}}), sq._wait(1.0)], 1, sent.append, lambda: 0.0)
        r.start(); r.tick()
        r.stop()
        self.assertEqual(sent, ["servo 90", "servo cerrar", "stop"])


if __name__ == "__main__":
    unittest.main()
