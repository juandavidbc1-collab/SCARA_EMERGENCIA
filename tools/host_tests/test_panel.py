"""Pruebas de la ventana sin pantalla (Qt offscreen) y con un enlace falso.  python3 -m unittest tools/host_tests/test_panel.py"""
import os
import sys
import time
import unittest

os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2"))
from PyQt5 import QtCore, QtTest, QtWidgets  # noqa: E402
import scara_panel as sp  # noqa: E402

APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
HOLD = [10.0, 20.0, 150.0, 60.0, 2.0, 0.0, 1.0, 1.0, 0.0, 10.0, 20.0, 0.0, 0.0, 3.0, 6.0, 0.0]


class FakeLink:
    def __init__(self):
        self.sent, self.gotos, self.keys, self.events, self.injected = [], [], set(), [], []
        self.state = list(HOLD)
        self.age = 0.01
        self.beats = 0

    def command(self, text):
        self.sent.append(text)

    def goto(self, x, y, z):
        self.gotos.append((x, y, z))

    def inject(self, text):
        self.injected.append(text)

    def snapshot(self):
        return (list(self.state) if self.state is not None else None), self.age, 20.0

    def pop_events(self, n):
        out, self.events = self.events[:n], self.events[n:]
        return out

    def set_keys(self, keys):
        self.keys = set(keys)

    def gui_beat(self):
        self.beats += 1

    def jog_active(self):
        return (0.0, 0.0, 0.0)


class PanelCase(unittest.TestCase):
    def setUp(self):
        self.link = FakeLink()
        self.p = sp.ControlPanel(self.link, sim=True)
        self.p.show()
        QtTest.QTest.qWaitForWindowExposed(self.p)
        self.p.tick()
        self.link.sent.clear()

    def tearDown(self):
        self.p.close()

    def key(self, k, press=True):
        (QtTest.QTest.keyPress if press else QtTest.QTest.keyRelease)(self.p, k)

    def test_a_huge_pasted_command_is_cut_before_it_reaches_the_robot(self):
        self.p.cmd_line.setText("x" * 2000)
        self.p.do_cmdline()
        self.assertEqual(len(self.link.sent), 1)
        self.assertLessEqual(len(self.link.sent[0]), 90)

    def test_esc_in_the_command_box_stops_and_gives_the_keyboard_back(self):
        self.p.activateWindow()
        self.p.cmd_line.setFocus()
        APP.processEvents()
        self.assertTrue(self.p.cmd_line.hasFocus())
        QtTest.QTest.keyClick(self.p.cmd_line, QtCore.Qt.Key_Escape)
        self.assertIn("stop", self.link.sent)
        self.assertFalse(self.p.cmd_line.hasFocus())

    def test_estop_button_sends_at_once_from_any_tab(self):
        for i in range(self.p.tabs.count()):
            self.p.tabs.setCurrentIndex(i)
            self.link.sent.clear()
            QtTest.QTest.mouseClick(self.p.btn_estop, QtCore.Qt.LeftButton)
            self.assertEqual(self.link.sent, ["estop"])

    def test_estop_key(self):
        self.key(QtCore.Qt.Key_E)
        self.assertEqual(self.link.sent, ["estop"])

    def test_estop_state_shows_banner_and_rearm(self):
        self.assertFalse(self.p.btn_rearm.isEnabled())
        self.link.state[4], self.link.state[5] = 6.0, 6.0
        self.p.tick()
        self.assertEqual(self.p.lbl_mode.text(), "PARO")
        self.assertTrue(self.p.btn_rearm.isEnabled())
        QtTest.QTest.mouseClick(self.p.btn_rearm, QtCore.Qt.LeftButton)
        self.assertEqual(self.link.sent, ["clear"])

    def test_motor_toggle_sends_and_follows_robot_state(self):
        b = self.p.motor_btn[1]
        self.assertTrue(b.isChecked())
        QtTest.QTest.mouseClick(b, QtCore.Qt.LeftButton)
        self.assertEqual(self.link.sent, ["motor 2 off"])
        self.link.state[13] = 1.0                 # el robot confirma: solo J1
        self.p.tick()
        self.assertFalse(b.isChecked())
        self.assertIn("Desactivado", b.text())

    def test_toggle_returns_if_robot_rejects(self):
        b = self.p.motor_btn[0]
        QtTest.QTest.mouseClick(b, QtCore.Qt.LeftButton)
        for _ in range(3):
            self.p.tick()
        self.p.motor_pending[0] = 0.0             # pasó el tiempo de espera y el robot no cambió
        self.p.tick()
        self.assertTrue(b.isChecked(), "muestra lo que el robot reporta, no lo que se pidió")

    def test_m3_start_blocked_while_disabled_in_robot(self):
        self.assertFalse(self.p.btn_tool.isEnabled())
        self.link.state[13] = 7.0
        self.p.tick()
        self.assertTrue(self.p.btn_tool.isEnabled())

    def test_deadman_keys_reach_link_and_autorepeat_is_ignored(self):
        self.key(QtCore.Qt.Key_Space)
        self.key(QtCore.Qt.Key_Right)
        self.assertEqual(self.link.keys, {"space", "right"})
        self.key(QtCore.Qt.Key_Right, press=False)
        self.assertEqual(self.link.keys, {"space"})

    def test_typing_in_a_box_does_not_drive_robot(self):
        self.p.tabs.setCurrentWidget(self.p.tab_op)
        self.p.edit_x.setFocus()
        QtTest.QTest.keyClick(self.p.edit_x, QtCore.Qt.Key_E)
        QtTest.QTest.keyPress(self.p.edit_x, QtCore.Qt.Key_Space)
        self.assertEqual(self.link.sent, [])
        self.assertEqual(self.link.keys, set())

    def test_window_losing_focus_releases_keys(self):
        self.key(QtCore.Qt.Key_Space)
        self.key(QtCore.Qt.Key_Up)
        self.p.release_all_keys()
        self.assertEqual(self.link.keys, set())

    def test_goto_bad_number_is_reported_not_sent(self):
        self.p.tabs.setCurrentWidget(self.p.tab_op)
        self.p.edit_x.setText("abc")
        self.p.do_goto()
        self.assertEqual(self.link.gotos, [])
        self.assertIn("x", self.p.log.toPlainText())

    def test_ang_nan_is_reported_not_sent(self):
        self.p.edit_q1.setText("nan")
        self.p.edit_q2.setText("0")
        self.p.do_ang()
        self.assertEqual([c for c in self.link.sent if c.startswith("ang")], [])
        self.assertIn("q1", self.p.log.toPlainText())

    def test_goto_inf_is_reported_not_sent(self):
        self.p.edit_x.setText("inf")
        self.p.edit_y.setText("80")
        self.p.do_goto()
        self.assertEqual(self.link.gotos, [])

    def test_goto_accepts_comma_decimal(self):
        self.p.edit_x.setText("120,5")
        self.p.edit_y.setText("80")
        self.p.do_goto()
        self.assertEqual(self.link.gotos[-1][:2], (120.5, 80.0))

    def test_no_data_is_visible(self):
        self.link.state = None
        self.link.age = 99.0
        self.p.tick()
        self.assertIn("SIN DATOS", self.p.lbl_link.text())

    def test_stale_data_is_visible(self):
        self.link.age = 2.0
        self.p.tick()
        self.assertIn("SIN DATOS", self.p.lbl_link.text())

    def test_event_burst_is_drained_and_log_is_bounded(self):
        self.link.events = ["#INFO: %d" % i for i in range(5000)]
        for _ in range(100):
            self.p.tick()
        self.assertEqual(self.link.events, [])
        self.assertLessEqual(self.p.log.blockCount(), sp.LOG_LINES + 1)

    def test_events_reach_running_sequence(self):
        self.p.load_steps([{"kind": "goto", "x": 150.0, "y": 40.0}, {"kind": "goto", "x": 110.0, "y": 90.0}], 1)
        self.p.seq_start()
        self.assertEqual(self.link.sent[-1], "goto 150 40")
        self.link.events = ["#LLEGO: ok"]
        self.p.tick()
        self.assertEqual(self.link.sent[-1], "goto 110 90")

    def test_fault_stops_sequence_and_tells_why(self):
        self.p.load_steps([{"kind": "goto", "x": 150.0, "y": 40.0}, {"kind": "goto", "x": 110.0, "y": 90.0}], 1)
        self.p.seq_start()
        self.link.events = ["#FALLA: motor trabado"]
        self.p.tick()
        self.assertIn("trabado", self.p.lbl_seq.text())
        self.assertEqual(self.link.sent[-1], "goto 150 40")

    def test_sequence_refuses_to_start_without_home(self):
        self.link.state[6] = 0.0
        self.p.tick()
        self.p.load_steps([{"kind": "goto", "x": 150.0, "y": 40.0}], 1)
        self.p.seq_start()
        self.assertFalse(any(s.startswith("goto") for s in self.link.sent))
        self.assertIn("Home", self.p.lbl_seq.text())

    def test_sequence_refuses_to_start_on_stale_data(self):
        self.link.age = 2.0
        self.p.tick()
        self.p.load_steps([{"kind": "goto", "x": 150.0, "y": 40.0}], 1)
        self.link.sent.clear()
        self.p.seq_start()
        self.assertEqual(self.link.sent, [])
        self.assertIn("sin datos", self.p.lbl_seq.text())

    def test_calibration_buttons_send_commands(self):
        self.p.btn_ref.click()
        self.assertEqual(self.link.sent[-1], "ref")
        self.p.btn_mapa.click()
        self.assertEqual(self.link.sent[-1], "mapa")

    def test_ajuste_sends_what_was_measured(self):
        pedido, medido = self.p.cal_edit[2]
        pedido.setText("40")
        medido.setText("41.5")
        self.p.cal_btn[2].click()
        self.assertEqual(self.link.sent[-1], "ajuste 2 40 41.5")

    def test_ajuste_needs_numbers(self):
        self.p.cal_edit[1][1].setText("no")
        self.link.sent.clear()
        self.p.cal_btn[1].click()
        self.assertEqual(self.link.sent, [])

    def test_calibration_results_are_shown(self):
        self.link.events = ["#MAPA: J1 v-=66.7 v+=66.6 t-=106 t+=101 aj=1.0100 | J2 v-=66.7 v+=66.7 t-=81 t+=81 aj=1.0060"]
        self.p.tick()
        self.assertIn("v-=66.7", self.p.lbl_cal.text())
        self.link.events = ["#OK: pon en config.h J1_AJUSTE = 1.0155f"]
        self.p.tick()
        self.assertIn("J1_AJUSTE = 1.0155f", self.p.lbl_cal.text())

    def test_panel_says_it_has_no_encoders(self):
        self.assertIn("sin encoders", self.p.windowTitle())
        self.assertFalse(hasattr(self.p, "ctrl_btn"))
        self.assertFalse(hasattr(self.p, "p_i"), "sin encoder de M3 no hay gráfica de corriente")

    def test_fault_injection_only_in_sim(self):
        self.p.btn_cut.click()
        self.assertEqual(self.link.injected[-1], "power 0")
        real = sp.ControlPanel(FakeLink(), sim=False)
        self.assertFalse(hasattr(real, "btn_cut"))
        real.close()

    def test_arm_view_shows_the_whole_workspace_at_any_window_size(self):
        for w, h in ((1320, 840), (1000, 700), (1700, 1000)):
            self.p.resize(w, h)
            QtTest.QTest.qWait(30)
            (x0, x1), (y0, y1) = self.p.xy.getViewBox().viewRange()
            for lo, hi in ((x0, x1), (y0, y1)):
                self.assertLessEqual(lo, -190.0)
                self.assertGreaterEqual(hi, 190.0)

    def test_footer_clears_after_the_fault_is_gone(self):
        self.link.events = ["#FALLA: motor trabado"]
        self.p.tick()
        self.assertIn("trabado", self.p.lbl_key.text())
        self.p.tick()
        self.assertEqual(self.p.lbl_key.text(), "")

    def test_default_sequence_shows_all_holes_and_boards(self):
        self.assertEqual(self.p.hole_table.rowCount(), 16)
        self.assertEqual(self.p.hole_table.item(0, 0).text(), "A-H1")
        self.assertEqual(len(self.p.xy_holes.getData()[0]), 16)
        self.assertIn("origen", self.p.lbl_seqnote.text())

    def test_sequence_with_screws_needs_m3_active(self):
        self.p.seq_start()
        self.p.tick()
        self.assertIsNone(self.p.runner)
        self.assertIn("M3", self.p.lbl_seq.text())
        self.link.sent.clear()
        st = list(HOLD); st[13] = 7.0
        self.link.state = st
        self.p.tick()
        self.p.seq_start()
        self.p.tick()
        self.assertIsNotNone(self.p.runner)
        self.assertEqual(self.link.sent[0], "elbow -1")

    def test_screw_result_fills_the_hole_table(self):
        st = list(HOLD); st[13] = 7.0
        self.link.state = st
        self.p.tick()
        self.p.load_steps([{"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H1", "desc": "x"},
                           {"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H3", "desc": "y"}], 1)
        self.p.seq_start()
        self.p.tick()
        self.assertEqual(self.p.hole_table.item(0, 1).text(), "Atornillando")
        self.link.events.append("#TORNILLO: sin_medir vueltas=12.00 prof=6.00mm avance=100% torque=0.000Nm fuerza=0N presion=0.0MPa corriente=0.00A t=23.2s")
        self.p.tick()
        self.assertEqual(self.p.hole_table.item(0, 1).text(), "sin_medir")
        self.assertEqual(self.p.hole_table.item(0, 2).text(), "12.00")
        self.assertEqual(self.p.hole_table.item(0, 3).text(), "6.00")
        self.assertEqual(self.p.hole_table.item(0, 4).text(), "23.2")
        self.assertNotIn("Torque N·m", sp.HOLE_COLS)
        self.assertIn("revísalo", self.p.lbl_screw_result.text())
        self.assertEqual(self.p.hole_table.item(1, 1).text(), "Atornillando")

    def test_bad_screw_is_shown_and_stops(self):
        st = list(HOLD); st[13] = 7.0
        self.link.state = st
        self.p.tick()
        self.p.load_steps([{"kind": "screw", "vueltas": 12.0, "torque": 0.4, "label": "A-H1", "desc": "x"}], 1)
        self.p.seq_start()
        self.p.tick()
        self.link.events.append("#TORNILLO: cancelado vueltas=3.00 prof=1.50mm avance=25% torque=0.000Nm fuerza=0N presion=0.0MPa corriente=0.00A t=4.6s")
        self.p.tick()
        self.assertEqual(self.p.hole_table.item(0, 1).text(), "cancelado")
        self.assertIn("ABORTADA", self.p.lbl_seq.text())

    def test_rehearsal_option_reloads_without_servo_or_screw(self):
        self.p.chk_ensayo.click()
        self.assertFalse([s for s in self.p.steps if s["kind"] == "screw"])
        self.assertEqual(self.p.hole_table.rowCount(), 16)
        self.assertEqual(self.p.hole_table.item(0, 1).text(), "Pendiente")
        self.p.chk_ensayo.click()
        self.assertEqual(sum(s["kind"] == "screw" for s in self.p.steps), 16)

    def test_rehearsal_keeps_the_chosen_repetitions(self):
        self.p.spin_repeat.setValue(7)
        self.p.chk_ensayo.click()
        self.assertEqual(self.p.spin_repeat.value(), 7)

    def _start_rehearsal(self):
        st = list(HOLD); st[13] = 7.0
        self.link.state = st
        self.p.tick()
        self.p.chk_ensayo.click()
        self.p.load_steps([{"kind": "tool", "pct": 40.0, "secs": 100.0, "label": "A-H1", "desc": "b"},
                           {"kind": "tool", "pct": -40.0, "secs": 0.0, "label": "A-H1", "fin": True, "desc": "s"}], 1)
        self.link.sent.clear()
        self.p.seq_start()
        self.p.tick()

    def test_rehearsal_needs_m3_active(self):
        self.p.chk_ensayo.click()
        self.p.seq_start()
        self.p.tick()
        self.assertIsNone(self.p.runner)
        self.assertIn("M3", self.p.lbl_seq.text())

    def test_rehearsal_shows_progress_in_the_hole_table(self):
        self._start_rehearsal()
        self.assertEqual(self.link.sent[0], "tool 40")
        self.assertEqual(self.p.hole_table.item(0, 1).text(), "Ensayando")
        self.assertFalse(self.p.chk_ensayo.isEnabled(), "no cambia mientras corre")

    def test_stopping_a_rehearsal_warns_that_m3_may_be_down(self):
        self._start_rehearsal()
        self.p.seq_stop()
        self.p.tick()
        self.assertIn("abajo", self.p.lbl_seq.text())
        self.assertTrue(self.p.chk_ensayo.isEnabled())

    def test_screwdriver_tab_shows_estimates_only(self):
        st = list(HOLD) + [1.25, 0.41, 630.0, 40.1, 11.8, 5.9, 90.0, 3.0]
        self.link.state = st
        self.p.tick()
        self.assertIn("11.80", self.p.lbl_screw_vals.text())
        self.assertNotIn("Corriente", self.p.lbl_screw_vals.text())

    def test_screwdriver_buttons_send_commands(self):
        self.link.sent.clear()
        self.p.btn_servo_open.click()
        self.p.btn_servo_close.click()
        self.p.btn_screw_here.click()
        self.assertEqual(self.link.sent[:3], ["servo abrir", "servo cerrar", "screw 12 0.4"])

    def test_tick_is_fast_with_plots_running(self):
        t0 = time.perf_counter()
        for i in range(120):
            self.link.state[0] = 10.0 + i * 0.1
            self.p.tick()
        ms = (time.perf_counter() - t0) / 120 * 1000
        self.assertLess(ms, 15.0, "un refresco tarda %.1f ms" % ms)


if __name__ == "__main__":
    unittest.main()
