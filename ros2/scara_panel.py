#!/usr/bin/env python3
"""
Panel de control del SCARA, firmware de EMERGENCIA sin encoders. Es el mismo del firmware principal; los ángulos son estimados.

    python3 ros2/scara_panel.py                 con el robot (agente micro-ROS corriendo)
    python3 ros2/scara_panel.py --sim           sin robot: arranca el simulador y RViz
    python3 ros2/scara_panel.py --sim --sin-rviz   solo el simulador

Teclado (con el foco fuera de los cuadros de texto):
    ESPACIO mantenido = dead-man  ·  flechas: J1 izquierda/derecha, J2 arriba/abajo
    E paro de emergencia  ·  H Home  ·  S detener  ·  O apagar  ·  C borrar falla  ·  Esc detener

Requiere:  sudo apt install python3-pyqt5 python3-pyqtgraph
"""
import argparse
import html
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")  # pyqtgraph debe usar el mismo Qt: importar otro aborta el programa
import pyqtgraph as pg  # noqa: E402

import scara_link as sl  # noqa: E402
import scara_sequence as sq  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TICK_MS = 33                  # refresco de la ventana
PLOT_EVERY = 3                # las gráficas a 10 Hz: bastan para verlas y es lo que más cuesta dibujar
PLOT_SECONDS = 10.0
PLOT_POINTS = 240
EVENTS_PER_TICK = 100
LOG_LINES = 800
STALE_S = 0.5                 # sin estado por más que esto, se avisa
TOGGLE_WAIT_S = 1.0           # tiempo que un interruptor muestra lo pedido antes de volver a lo que reporta el robot
TOOL_RENEW_S = 5.0
Z_MM = 0.0
L1, L2 = sq.L1_MM, sq.L2_MM

BG, CARD, CARD2, LINE = "#16181c", "#1f2228", "#272b33", "#363b45"
INK, MUTED = "#eceef2", "#a4a9b4"
SERIES1, SERIES2, SERIES3 = "#3987e5", "#d95926", "#199e70"
GOOD, WARN, CRIT, INFO = "#1f9d55", "#c98500", "#d92d20", "#3987e5"

MODE_COLORS = {"OFF": "#5a606c", "HOMING": INFO, "HOLD": GOOD, "MOVE": INFO, "JOG": INFO, "PULSE": WARN,
               "FALLA": CRIT, "PARO": CRIT}
MODE_TEXT = {"OFF": "Motores apagados", "HOMING": "Midiendo tiempos", "HOLD": "Frenado",
             "MOVE": "En movimiento", "JOG": "Manual", "PULSE": "Pulso de prueba"}
KEY_NAMES = {QtCore.Qt.Key_Space: "space", QtCore.Qt.Key_Left: "left", QtCore.Qt.Key_Right: "right",
             QtCore.Qt.Key_Up: "up", QtCore.Qt.Key_Down: "down"}
SHORTCUTS = {QtCore.Qt.Key_H: "home", QtCore.Qt.Key_S: "stop", QtCore.Qt.Key_O: "off", QtCore.Qt.Key_C: "clear",
             QtCore.Qt.Key_Escape: "stop"}
HOLE_COLS = ("Hueco", "Estado", "Vueltas est.", "Prof. est. mm", "Tiempo s")
HOLE_FIELDS = ("vueltas", "prof", "t")
HOLE_FMT = {"vueltas": "%.2f", "prof": "%.2f", "t": "%.1f"}
SCREW_DEFAULT = (12, 0.4)     # lo mismo que SCREW_VUELTAS_META y SCREW_TORQUE_META en src/config.h

QSS = f"""
QWidget {{ background: {BG}; color: {INK}; font-size: 13px; }}
QLabel {{ background: transparent; }}
QLabel[muted="true"] {{ color: {MUTED}; }}
QFrame#card {{ background: {CARD}; border: 1px solid {LINE}; border-radius: 8px; }}
QLabel#cardTitle {{ font-weight: 600; font-size: 13px; }}
QPushButton {{ background: {CARD2}; border: 1px solid {LINE}; border-radius: 6px; padding: 7px 12px; }}
QPushButton:hover {{ background: #323742; }}
QPushButton:pressed {{ background: #3b414e; }}
QPushButton:disabled {{ color: #6a6f7a; background: {CARD}; }}
QPushButton:checked {{ background: {INFO}; border-color: {INFO}; color: white; }}
QPushButton#estop {{ background: {CRIT}; border: 2px solid #ff8a80; color: white; font-size: 17px; font-weight: 700;
    padding: 12px 22px; border-radius: 10px; }}
QPushButton#estop:hover {{ background: #ef3b2d; }}
QPushButton#estop:pressed {{ background: #a81c12; }}
QPushButton#motorOn {{ background: {GOOD}; border-color: {GOOD}; color: white; font-weight: 600; }}
QPushButton#motorOff {{ background: {CARD2}; color: {MUTED}; font-weight: 600; }}
QLineEdit, QSpinBox {{ background: {BG}; border: 1px solid {LINE}; border-radius: 5px; padding: 5px 7px; }}
QLineEdit:focus, QSpinBox:focus {{ border-color: {INFO}; }}
QTabWidget::pane {{ border: 1px solid {LINE}; border-radius: 8px; top: -1px; }}
QTabBar::tab {{ background: transparent; color: {MUTED}; padding: 9px 14px; }}
QTabBar::tab:selected {{ color: {INK}; border-bottom: 2px solid {INFO}; }}
QPlainTextEdit, QListWidget, QTableWidget {{ background: {BG}; border: 1px solid {LINE}; border-radius: 6px; gridline-color: {LINE}; }}
QHeaderView::section {{ background: {CARD2}; color: {MUTED}; border: none; padding: 4px; }}
QListWidget::item:selected {{ background: {INFO}; color: white; }}
QSlider::groove:horizontal {{ height: 4px; background: {LINE}; border-radius: 2px; }}
QSlider::handle:horizontal {{ width: 16px; margin: -7px 0; background: {INK}; border-radius: 8px; }}
"""


def nofocus(w):
    """Los controles no se quedan con el teclado: si no, ESPACIO pulsaría el último botón usado en vez del dead-man."""
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    return w


def card(title=None):
    f = QtWidgets.QFrame(objectName="card")
    v = QtWidgets.QVBoxLayout(f)
    v.setContentsMargins(12, 10, 12, 12)
    v.setSpacing(8)
    if title:
        v.addWidget(QtWidgets.QLabel(title, objectName="cardTitle"))
    return f, v


def muted(text=""):
    lb = QtWidgets.QLabel(text)
    lb.setProperty("muted", True)
    return lb


class ControlPanel(QtWidgets.QWidget):
    def __init__(self, link, sim=False):
        super().__init__()
        self.link, self.sim = link, sim
        self.setWindowTitle("SCARA · EMERGENCIA, sin encoders" + (" · SIMULACIÓN" if sim else ""))
        self.setStyleSheet(QSS)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.resize(1400, 860)

        self.state = None
        self.keys = set()
        self.ticks = 0
        self.motor_pending = {}
        self.tool_t = 0.0
        self.history, self.hist_i = [], 0
        self.steps, self.repeat = [], 1
        self.runner = None
        self.seq_shown = None
        self._texts = {}
        self._mode_shown = None
        self._footer_on = False

        n = PLOT_POINTS
        self.buf_t = np.full(n, np.nan)
        self.buf = {k: np.full(n, np.nan) for k in ("q1", "q2", "r1", "r2", "d1", "d2")}
        self.buf_i = 0
        self.t0 = time.monotonic()
        self.trail = np.full((200, 2), np.nan)
        self.trail_i = 0
        self.target = None

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)
        root.addWidget(self._build_header())
        body = QtWidgets.QHBoxLayout()
        body.setSpacing(10)
        body.addWidget(self._build_views(), 3)
        body.addWidget(self._build_tabs(), 2)
        root.addLayout(body, 1)
        root.addWidget(self._build_footer())

        self.timer = QtCore.QTimer(self, timerType=QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self.tick)
        self.timer.start(TICK_MS)
        self.setFocus()
        self._load_default_sequence()

    # ------------------------------------------------------------------ construcción
    def _build_header(self):
        f, _ = card()
        h = QtWidgets.QHBoxLayout()
        f.layout().addLayout(h)
        self.lbl_mode = QtWidgets.QLabel("—", alignment=QtCore.Qt.AlignCenter)
        self.lbl_mode.setMinimumWidth(130)
        self.lbl_mode.setStyleSheet(self._pill(MODE_COLORS["OFF"]))
        h.addWidget(self.lbl_mode)

        col = QtWidgets.QVBoxLayout()
        col.setSpacing(2)
        self.lbl_detail = QtWidgets.QLabel("Esperando al robot…")
        self.lbl_detail.setStyleSheet("font-weight: 600; font-size: 14px;")
        self.lbl_pose = muted("")
        self.lbl_pose.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        col.addWidget(self.lbl_detail)
        col.addWidget(self.lbl_pose)
        h.addLayout(col, 1)

        col2 = QtWidgets.QVBoxLayout()
        col2.setSpacing(2)
        self.lbl_link = QtWidgets.QLabel("SIN DATOS", alignment=QtCore.Qt.AlignRight)
        self.lbl_ctrl = muted("")
        self.lbl_ctrl.setAlignment(QtCore.Qt.AlignRight)
        col2.addWidget(self.lbl_link)
        col2.addWidget(self.lbl_ctrl)
        h.addLayout(col2)

        self.btn_rearm = nofocus(QtWidgets.QPushButton("Rearmar"))
        self.btn_rearm.setEnabled(False)
        self.btn_rearm.setToolTip("Borra el paro o la falla (C). Los motores siguen apagados.")
        self.btn_rearm.clicked.connect(lambda: self.send("clear"))
        h.addWidget(self.btn_rearm)
        self.btn_estop = nofocus(QtWidgets.QPushButton("PARO DE EMERGENCIA", objectName="estop"))
        self.btn_estop.setToolTip("Tecla E. Depende de la comunicación: el paro físico de 12 V es el que manda.")
        self.btn_estop.clicked.connect(self.do_estop)
        h.addWidget(self.btn_estop)
        return f

    @staticmethod
    def _pill(color):
        return f"background: {color}; color: white; font-size: 18px; font-weight: 700; padding: 10px 14px; border-radius: 8px;"

    def _plot(self, title, ylabel):
        p = pg.PlotWidget(background=CARD)
        p.setTitle(title, color=MUTED, size="10pt")
        p.setMenuEnabled(False)
        p.setMouseEnabled(False, False)
        p.hideButtons()
        p.showGrid(x=True, y=True, alpha=0.12)
        p.setLabel("left", ylabel, color=MUTED)
        p.setClipToView(True)
        for ax in ("left", "bottom"):
            p.getAxis(ax).setPen(LINE)
            p.getAxis(ax).setTextPen(MUTED)
        nofocus(p)
        return p

    def _build_views(self):
        w = QtWidgets.QWidget()
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setSpacing(8)

        self.xy = pg.PlotWidget(background=CARD)
        nofocus(self.xy)
        self.xy.setTitle("Vista superior [mm]", color=MUTED, size="10pt")
        self.xy.setToolTip("Clic: lleva ese punto a los cuadros de «Ir a una posición». No mueve el robot.")
        self.xy.setAspectLocked(True)
        self.xy.setMenuEnabled(False)
        self.xy.setMouseEnabled(False, False)
        self.xy.hideButtons()
        self.xy.showGrid(x=True, y=True, alpha=0.10)
        self._fit_xy()
        self.xy.getViewBox().sigResized.connect(self._fit_xy)
        for ax in ("left", "bottom"):
            self.xy.getAxis(ax).setPen(LINE)
            self.xy.getAxis(ax).setTextPen(MUTED)
        th = np.linspace(0, 2 * math.pi, 180)
        ring = pg.mkPen(color=LINE, width=1.5)
        for r in (L1 + L2, abs(L1 - L2)):
            self.xy.plot(r * np.cos(th), r * np.sin(th), pen=ring)
        self.xy_boards = pg.PlotCurveItem(pen=pg.mkPen(color=SERIES3, width=1.5), connect="finite")
        self.xy.addItem(self.xy_boards)
        self.xy_holes = pg.ScatterPlotItem(size=7, pen=None)
        self.xy.addItem(self.xy_holes)
        self.xy_trail = self.xy.plot([], [], pen=pg.mkPen(color=SERIES3, width=1.5))
        self.xy_ref = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(WARN), pen=None)
        self.xy_target = pg.ScatterPlotItem(size=14, symbol="x", brush=pg.mkBrush(SERIES3), pen=pg.mkPen(SERIES3))
        self.xy.addItem(self.xy_ref)
        self.xy.addItem(self.xy_target)
        self.xy_arm = pg.PlotCurveItem(pen=pg.mkPen(color=SERIES1, width=7), antialias=True)
        self.xy.addItem(self.xy_arm)
        self.xy_joints = pg.ScatterPlotItem(size=11, brush=pg.mkBrush(INK), pen=None)
        self.xy.addItem(self.xy_joints)
        self.xy.scene().sigMouseClicked.connect(self._on_xy_click)
        g.addWidget(self.xy, 0, 0, 3, 1)

        self.p_q1 = self._plot("J1: estimado y destino", "grados")
        self.p_q2 = self._plot("J2: estimado y destino", "grados")
        self.p_d = self._plot("PWM aplicado", "cuentas")
        ref_pen = pg.mkPen(color=MUTED, width=1, style=QtCore.Qt.DashLine)
        self.c_r1 = self.p_q1.plot([], [], pen=ref_pen, name="Destino")
        self.c_q1 = self.p_q1.plot([], [], pen=pg.mkPen(color=SERIES1, width=2), name="Estimado")
        self.c_r2 = self.p_q2.plot([], [], pen=ref_pen, name="Destino")
        self.c_q2 = self.p_q2.plot([], [], pen=pg.mkPen(color=SERIES1, width=2), name="Estimado")
        self.p_d.addLegend(offset=(6, 4), labelTextColor=MUTED, brush=None)
        self.c_d1 = self.p_d.plot([], [], pen=pg.mkPen(color=SERIES1, width=2), name="J1")
        self.c_d2 = self.p_d.plot([], [], pen=pg.mkPen(color=SERIES2, width=2), name="J2")
        self.p_q2.setXLink(self.p_q1)
        self.p_d.setXLink(self.p_q1)
        g.addWidget(self.p_q1, 0, 1)
        g.addWidget(self.p_q2, 1, 1)
        g.addWidget(self.p_d, 2, 1)
        g.setColumnStretch(0, 6)
        g.setColumnStretch(1, 5)
        return w

    def _fit_xy(self, *_):
        """Con la relación de aspecto fija, hay que pedir el rango de nuevo cada vez que cambia el tamaño."""
        r = L1 + L2 + 15
        self.xy.getViewBox().setRange(xRange=(-r, r), yRange=(-r, r), padding=0)

    def _build_tabs(self):
        self.tabs = nofocus(QtWidgets.QTabWidget())
        self.tab_op = self._tab_operate()
        self.tab_motors = self._tab_motors()
        self.tab_screw = self._tab_screwdriver()
        self.tab_seq = self._tab_sequence()
        self.tab_ctrl = self._tab_control()
        self.tab_log = self._tab_log()
        self.tabs.addTab(self.tab_op, "Operar")
        self.tabs.addTab(self.tab_motors, "Motores")
        self.tabs.addTab(self.tab_screw, "Atornillador")
        self.tabs.addTab(self.tab_seq, "Secuencia")
        self.tabs.addTab(self.tab_ctrl, "Calibración")
        self.tabs.addTab(self.tab_log, "Registro")
        return self.tabs

    def _button(self, text, slot, tip=None):
        b = nofocus(QtWidgets.QPushButton(text))
        b.clicked.connect(slot)
        if tip:
            b.setToolTip(tip)
        return b

    def _edit(self, text="", width=70):
        e = QtWidgets.QLineEdit(text)
        e.setMaximumWidth(width)
        return e

    def _tab_operate(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(10)

        f, c = card("Acciones")
        g = QtWidgets.QGridLayout()
        for i, (txt, cmd, tip) in enumerate((("Home", "home", "Tecla H"), ("Detener", "stop", "Tecla S. Mantiene la posición."),
                                             ("Apagar motores", "off", "Tecla O"), ("Borrar falla", "clear", "Tecla C"))):
            g.addWidget(self._button(txt, lambda _=False, k=cmd: self.send(k), tip), 0, i)
        c.addLayout(g)
        v.addWidget(f)

        f, c = card("Ir a una posición (mm)")
        h = QtWidgets.QHBoxLayout()
        self.edit_x, self.edit_y = self._edit("120"), self._edit("80")
        for lb, e in (("x", self.edit_x), ("y", self.edit_y)):
            h.addWidget(muted(lb))
            h.addWidget(e)
            e.returnPressed.connect(self.do_goto)
        h.addWidget(self._button("Ir", self.do_goto))
        h.addStretch()
        h.addWidget(muted("Codo"))
        self.elbow_group = QtWidgets.QButtonGroup(self)
        for val, txt in ((1, "+1"), (-1, "−1")):
            b = nofocus(QtWidgets.QPushButton(txt, checkable=True))
            b.setChecked(val == 1)
            b.clicked.connect(lambda _=False, x=val: self.send("elbow %+d" % x))
            self.elbow_group.addButton(b)
            h.addWidget(b)
        c.addLayout(h)
        v.addWidget(f)

        f, c = card("Ir a ángulos (grados)")
        h = QtWidgets.QHBoxLayout()
        self.edit_q1, self.edit_q2 = self._edit("20"), self._edit("90")
        for lb, e in (("q1", self.edit_q1), ("q2", self.edit_q2)):
            h.addWidget(muted(lb))
            h.addWidget(e)
            e.returnPressed.connect(self.do_ang)
        h.addWidget(self._button("Ir", self.do_ang))
        h.addStretch()
        c.addLayout(h)
        v.addWidget(f)

        f, c = card("Mover con el teclado")
        self.lbl_jog = QtWidgets.QLabel("")
        self.lbl_jog.setStyleSheet("font-weight: 600;")
        c.addWidget(self.lbl_jog)
        tip = muted("Mantén ESPACIO (dead-man) y usa las flechas: ← → mueven J1, ↑ ↓ mueven J2. Al soltar se detiene.\n"
                    "Si estás escribiendo en un cuadro, el teclado no mueve el robot: pulsa Esc o haz clic afuera.")
        tip.setWordWrap(True)
        c.addWidget(tip)
        v.addWidget(f)
        v.addStretch()
        return w

    def _tab_motors(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(10)
        self.motor_btn, self.motor_info = {}, {}
        desc = (("J1", "Eslabón 1"), ("J2", "Eslabón 2"), ("M3", "Atornillador"))
        for j, (name, what) in enumerate(desc):
            f, c = card()
            h = QtWidgets.QHBoxLayout()
            col = QtWidgets.QVBoxLayout()
            col.setSpacing(2)
            col.addWidget(QtWidgets.QLabel(f"{name} · {what}", objectName="cardTitle"))
            self.motor_info[j] = muted("")
            col.addWidget(self.motor_info[j])
            h.addLayout(col, 1)
            b = nofocus(QtWidgets.QPushButton("Activado", checkable=True, objectName="motorOn"))
            b.setChecked(j < 2)
            b.setMinimumWidth(120)
            b.clicked.connect(lambda checked, m=j: self.do_motor(m, checked))
            self.motor_btn[j] = b
            h.addWidget(b)
            c.addLayout(h)
            if j == 2:
                h2 = QtWidgets.QHBoxLayout()
                self.tool_slider = nofocus(QtWidgets.QSlider(QtCore.Qt.Horizontal))
                self.tool_slider.setRange(-100, 100)
                self.tool_slider.setValue(40)
                self.lbl_tool = muted("40 %")
                self.lbl_tool.setMinimumWidth(48)
                self.tool_slider.valueChanged.connect(self._on_tool_slider)
                self.btn_tool = nofocus(QtWidgets.QPushButton("Girar", checkable=True))
                self.btn_tool.setEnabled(False)
                self.btn_tool.clicked.connect(self.do_tool)
                h2.addWidget(muted("Potencia"))
                h2.addWidget(self.tool_slider, 1)
                h2.addWidget(self.lbl_tool)
                h2.addWidget(self.btn_tool)
                c.addLayout(h2)
            v.addWidget(f)
        note = muted("Un eje desactivado queda libre y pierde el Home: sin encoder nadie mediría si lo mueven. Home solo referencia "
                     "los ejes activos, «Ir a x, y» necesita J1 y J2, e «Ir a ángulos» mueve solo los activos.")
        note.setWordWrap(True)
        v.addWidget(note)
        if self.sim:
            f, c = card("Simulación: provocar fallas")
            h = QtWidgets.QHBoxLayout()
            self.btn_cut = self._button("Cortar 12 V", lambda: self.link.inject("power 0"), "Lo que hace el paro físico")
            h.addWidget(self.btn_cut)
            h.addWidget(self._button("Devolver 12 V", lambda: self.link.inject("power 1")))
            for j in (1, 2):
                b = nofocus(QtWidgets.QPushButton(f"Trabar J{j}", checkable=True))
                b.clicked.connect(lambda on, k=j: self.link.inject(("lock %d" if on else "free %d") % k))
                h.addWidget(b)
            c.addLayout(h)
            v.addWidget(f)
        v.addStretch()
        return w

    def _tab_screwdriver(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(10)

        f, c = card("Estimado por tiempo")
        self.lbl_screw_vals = QtWidgets.QLabel("Sin datos")
        self.lbl_screw_vals.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        c.addWidget(self.lbl_screw_vals)
        self.lbl_screw_result = QtWidgets.QLabel("")
        self.lbl_screw_result.setWordWrap(True)
        self.lbl_screw_result.setStyleSheet("font-weight: 600;")
        c.addWidget(self.lbl_screw_result)
        note = muted("Sin encoder de M3 no se mide nada: ni corriente, ni torque, ni si el tornillo quedó bien. M3 gira un tiempo "
                     "calculado a un PWM con el que, según el modelo, no se pasa del torque pedido. Revisa los tornillos a mano.")
        note.setWordWrap(True)
        c.addWidget(note)
        v.addWidget(f)

        f, c = card("Probar a mano")
        h = QtWidgets.QHBoxLayout()
        h.addWidget(muted("Servo"))
        self.btn_servo_open = self._button("Abrir", lambda: self.send("servo abrir"), "Suelta un tornillo")
        self.btn_servo_close = self._button("Cerrar", lambda: self.send("servo cerrar"))
        h.addWidget(self.btn_servo_open)
        h.addWidget(self.btn_servo_close)
        h.addStretch()
        h.addWidget(muted("Vueltas"))
        self.edit_turns = self._edit(str(SCREW_DEFAULT[0]), 50)
        h.addWidget(self.edit_turns)
        h.addWidget(muted("N·m"))
        self.edit_torque = self._edit(str(SCREW_DEFAULT[1]), 50)
        h.addWidget(self.edit_torque)
        self.btn_screw_here = self._button("Atornillar aquí", self.do_screw, "Hay que activar M3 en la pestaña Motores")
        h.addWidget(self.btn_screw_here)
        c.addLayout(h)
        v.addWidget(f)

        v.addStretch()
        return w

    def _tab_sequence(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(10)
        f, c = card("Proceso repetitivo")
        h = QtWidgets.QHBoxLayout()
        self.lbl_seqfile = muted("")
        self.lbl_seqfile.setWordWrap(True)
        h.addWidget(self.lbl_seqfile, 1)
        h.addWidget(self._button("Abrir…", self._seq_open))
        h.addWidget(self._button("Recargar", lambda: self._seq_load(self.seq_path)))
        c.addLayout(h)
        self.lbl_seqnote = muted("")
        self.lbl_seqnote.setWordWrap(True)
        c.addWidget(self.lbl_seqnote)
        self.chk_ensayo = nofocus(QtWidgets.QCheckBox("Ensayo: sin servo ni tornillo, M3 solo baja y sube en cada hueco"))
        self.chk_ensayo.clicked.connect(self._seq_toggle_rehearsal)
        c.addWidget(self.chk_ensayo)
        self.seq_list = nofocus(QtWidgets.QListWidget())
        c.addWidget(self.seq_list, 1)
        self.hole_table = nofocus(QtWidgets.QTableWidget(0, len(HOLE_COLS)))
        self.hole_table.setHorizontalHeaderLabels(HOLE_COLS)
        self.hole_table.verticalHeader().setVisible(False)
        self.hole_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hole_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.hole_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        c.addWidget(self.hole_table, 2)
        h = QtWidgets.QHBoxLayout()
        h.addWidget(muted("Repeticiones (0 = sin fin)"))
        self.spin_repeat = QtWidgets.QSpinBox()
        self.spin_repeat.setRange(0, 9999)
        h.addWidget(self.spin_repeat)
        h.addStretch()
        self.btn_seq_start = self._button("Iniciar", self.seq_start)
        self.btn_seq_stop = self._button("Detener", self.seq_stop)
        self.btn_seq_stop.setEnabled(False)
        h.addWidget(self.btn_seq_start)
        h.addWidget(self.btn_seq_stop)
        c.addLayout(h)
        self.lbl_seq = QtWidgets.QLabel("Sin ejecutar.")
        self.lbl_seq.setWordWrap(True)
        self.lbl_seq.setStyleSheet("font-weight: 600;")
        c.addWidget(self.lbl_seq)
        v.addWidget(f, 1)
        return w

    def _tab_control(self):
        """Sin encoder no hay controlador que ajustar: esta pestaña es para medir el robot y corregir la velocidad."""
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        v.setSpacing(10)
        f, c = card("Control: por tiempo, sin encoders")
        note = muted("Sin medición no hay PID ni RST. Cada eje se enciende el tiempo calculado con lo que midió Home, y se frena. "
                     "Los ángulos son estimados. Un eje trabado no se nota hasta el siguiente recorrido o hasta pisar un final de carrera.")
        note.setWordWrap(True)
        c.addWidget(note)
        h = QtWidgets.QHBoxLayout()
        self.btn_ref = self._button("Recorrer ejes", lambda: self.send("ref"), "De tope a tope: corrige la posición y la velocidad. Unos 16 s")
        self.btn_mapa = self._button("Ver lo medido", lambda: self.send("mapa"), "Velocidades y tiempos de arranque de cada eje")
        h.addWidget(self.btn_ref)
        h.addWidget(self.btn_mapa)
        h.addStretch()
        c.addLayout(h)
        self.lbl_cal = QtWidgets.QLabel("")
        self.lbl_cal.setWordWrap(True)
        self.lbl_cal.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self.lbl_cal.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        c.addWidget(self.lbl_cal)
        v.addWidget(f)

        f, c = card("Corregir la velocidad con un transportador")
        note = muted("1. Pulsa «Recorrer ejes».  2. Pide un ángulo lejano en Operar, por ejemplo 60 y 40.  3. Mide dónde quedó cada eje.  "
                     "4. Escribe aquí los dos valores y pulsa Ajustar.  5. Copia a src/config.h el valor que aparece arriba, "
                     "porque el ajuste se pierde al reiniciar.")
        note.setWordWrap(True)
        c.addWidget(note)
        g = QtWidgets.QGridLayout()
        g.addWidget(muted("Pedido °"), 0, 1, QtCore.Qt.AlignCenter)
        g.addWidget(muted("Medido °"), 0, 2, QtCore.Qt.AlignCenter)
        self.cal_edit, self.cal_btn = {}, {}
        for j in (1, 2):
            g.addWidget(muted(f"J{j}"), j, 0)
            self.cal_edit[j] = (self._edit("", 70), self._edit("", 70))
            g.addWidget(self.cal_edit[j][0], j, 1)
            g.addWidget(self.cal_edit[j][1], j, 2)
            self.cal_btn[j] = self._button("Ajustar", lambda _=False, k=j: self.do_ajuste(k))
            g.addWidget(self.cal_btn[j], j, 3)
        g.setColumnStretch(4, 1)
        c.addLayout(g)
        v.addWidget(f)
        v.addStretch()
        return w

    def _tab_log(self):
        w = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(w)
        self.log = nofocus(QtWidgets.QPlainTextEdit(readOnly=True))
        self.log.setMaximumBlockCount(LOG_LINES)
        self.log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        v.addWidget(self.log, 1)
        h = QtWidgets.QHBoxLayout()
        self.cmd_line = QtWidgets.QLineEdit(placeholderText="Comando, por ejemplo: status  (help los lista)")
        self.cmd_line.setMaxLength(90)      # /scara/cmd admite poco más; un texto enorme puede colgar el enlace
        self.cmd_line.returnPressed.connect(self.do_cmdline)
        self.cmd_line.installEventFilter(self)
        h.addWidget(self.cmd_line, 1)
        h.addWidget(self._button("Enviar", self.do_cmdline))
        v.addLayout(h)
        return w

    def _build_footer(self):
        self.lbl_key = muted("")
        return self.lbl_key

    # ------------------------------------------------------------------ registro y órdenes
    def add_log(self, text, color=None):
        safe = html.escape(text)
        self.log.appendHtml(f'<span style="color:{color}">{safe}</span>' if color else f"<span>{safe}</span>")

    def send(self, text):
        self.link.command(text)
        self.add_log("> " + text, INFO)

    def do_estop(self):
        self.link.command("estop")                # lo primero: antes de tocar nada de la ventana
        self.add_log("> estop", CRIT)
        if self.runner and self.runner.running:
            self.runner.abort("paro de emergencia")
        self._tool_stop_local()

    def _numbers(self, edits, names):
        try:
            values = [float(e.text().replace(",", ".")) for e in edits]
            if all(math.isfinite(v) for v in values):     # float() también acepta "nan" e "inf"
                return values
        except ValueError:
            pass
        self.add_log("Escribe números válidos en: " + ", ".join(names), CRIT)
        self.tabs.setCurrentWidget(self.tab_log)
        return None

    def do_goto(self):
        v = self._numbers((self.edit_x, self.edit_y), ("x", "y"))
        if v:
            self.target = tuple(v)
            self.link.goto(v[0], v[1], Z_MM)
            self.add_log("> goto %g %g" % tuple(v), INFO)
            self.setFocus()

    def do_ang(self):
        v = self._numbers((self.edit_q1, self.edit_q2), ("q1", "q2"))
        if v:
            self.target = (L1 * math.cos(math.radians(v[0])) + L2 * math.cos(math.radians(v[0] + v[1])),
                           L1 * math.sin(math.radians(v[0])) + L2 * math.sin(math.radians(v[0] + v[1])))
            self.send("ang %g %g" % tuple(v))
            self.setFocus()

    def do_motor(self, m, on):
        self.motor_pending[m] = time.monotonic() + TOGGLE_WAIT_S
        self._show_motor(m, on)
        self.send("motor %d %s" % (m + 1, "on" if on else "off"))

    def _show_motor(self, m, on):
        b = self.motor_btn[m]
        b.setChecked(on)
        wanted = "motorOn" if on else "motorOff"
        if b.objectName() != wanted:
            b.setObjectName(wanted)
            b.setStyle(b.style())
        b.setText("Activado" if on else "Desactivado")

    def _on_tool_slider(self, v):
        self.lbl_tool.setText("%d %%" % v)
        if self.btn_tool.isChecked():
            self.send("tool %d" % v)
            self.tool_t = time.monotonic()

    def do_tool(self, on):
        self.send("tool %d" % (self.tool_slider.value() if on else 0))
        self.tool_t = time.monotonic()

    def do_screw(self):
        v = self._numbers((self.edit_turns, self.edit_torque), ("vueltas", "N·m"))
        if v:
            self.send("screw %g %g" % tuple(v))
            self.setFocus()

    def _tool_stop_local(self):
        self.btn_tool.setChecked(False)

    def do_ajuste(self, j):
        v = self._numbers(self.cal_edit[j], ("pedido", "medido"))
        if v:
            self.send("ajuste %d %g %g" % (j, v[0], v[1]))
            self.setFocus()

    def do_cmdline(self):
        t = self.cmd_line.text().strip()
        if t:
            self.send(t)
            if not self.history or self.history[-1] != t:
                self.history.append(t)
            self.hist_i = len(self.history)
            self.cmd_line.clear()

    def eventFilter(self, obj, ev):
        if obj is self.cmd_line and ev.type() == QtCore.QEvent.KeyPress and self.history:
            if ev.key() in (QtCore.Qt.Key_Up, QtCore.Qt.Key_Down):
                self.hist_i = max(0, min(len(self.history), self.hist_i + (-1 if ev.key() == QtCore.Qt.Key_Up else 1)))
                self.cmd_line.setText(self.history[self.hist_i] if self.hist_i < len(self.history) else "")
                return True
        return super().eventFilter(obj, ev)

    def _on_xy_click(self, ev):
        p = self.xy.getPlotItem().vb.mapSceneToView(ev.scenePos())
        self.edit_x.setText("%.0f" % p.x())
        self.edit_y.setText("%.0f" % p.y())
        self.tabs.setCurrentWidget(self.tab_op)

    # ------------------------------------------------------------------ secuencia
    def _load_default_sequence(self):
        self.seq_path = os.path.join(HERE, "secuencia.yaml")
        self._seq_load(self.seq_path)

    def _seq_open(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Abrir secuencia", HERE, "Secuencia (*.yaml *.yml)")
        if path:
            self.seq_path = path
            self._seq_load(path)

    def _seq_load(self, path):
        self.lbl_seqfile.setText(path)
        try:
            seq = sq.load_sequence_full(path, ensayo=self.chk_ensayo.isChecked())
        except sq.SequenceError as e:
            self.lbl_seq.setText("No se pudo cargar: %s" % e)
            self.steps = []
            self.seq_list.clear()
            self.hole_table.setRowCount(0)
            return
        self.load_steps(seq.steps, seq.repeat)
        self.lbl_seqnote.setText(" ".join(seq.notes))
        self._show_process(seq.process)
        self.chk_ensayo.setEnabled(seq.process is not None)

    def _seq_toggle_rehearsal(self):
        repeat = self.spin_repeat.value()
        self._seq_load(self.seq_path)
        self.spin_repeat.setValue(repeat)

    def load_steps(self, steps, repeat):
        self.steps, self.repeat = steps, repeat
        self.spin_repeat.setValue(repeat)
        self.seq_list.clear()
        for i, s in enumerate(steps, start=1):
            self.seq_list.addItem("%d. %s" % (i, s.get("desc") or s["kind"]))
        self.lbl_seq.setText("%d pasos cargados." % len(steps))
        self.hole_rows = {}
        labels = []
        for k, s in enumerate(steps, start=1):
            if s["kind"] == "screw" or s.get("fin"):
                labels.append(s.get("label") or "paso %d" % k)
        self.hole_table.setRowCount(len(labels))
        for row, label in enumerate(labels):
            self.hole_rows[label] = row
            self.hole_table.setItem(row, 0, QtWidgets.QTableWidgetItem(label))
            self._hole_row(row, "Pendiente", {})
        self.results_shown = 0
        self.hole_active = None

    def _hole_row(self, row, status, res):
        vals = [status] + [HOLE_FMT[k] % res[k] if k in res else "" for k in HOLE_FIELDS]
        for col, text in enumerate(vals, start=1):
            item = QtWidgets.QTableWidgetItem(text)
            if col == 1 and status not in ("Pendiente", "Atornillando", "Ensayando"):
                item.setForeground(QtGui.QColor("#4ade80" if status in sq.RESULTADOS_BUENOS else "#ff8a80"))
            self.hole_table.setItem(row, col, item)

    def _show_process(self, process):
        self.process = process
        if process is None:
            self.xy_boards.setData([], [])
            self.xy_holes.setData([], [])
            return
        half = process.side / 2.0
        xs, ys = [], []
        for cx, cy in process.boards.values():
            xs += [cx - half, cx + half, cx + half, cx - half, cx - half, np.nan]
            ys += [cy - half, cy - half, cy + half, cy + half, cy - half, np.nan]
        self.xy_boards.setData(np.array(xs), np.array(ys))
        self._paint_holes()

    def _paint_holes(self):
        p = getattr(self, "process", None)
        if p is None:
            return
        results = {r["hueco"]: r["resultado"] for r in (self.runner.results if self.runner else [])}
        brushes = [pg.mkBrush(MUTED if h["label"] not in results else GOOD if results[h["label"]] in sq.RESULTADOS_BUENOS else CRIT) for h in p.holes]
        self.xy_holes.setData([h["x"] for h in p.holes], [h["y"] for h in p.holes], brush=brushes)

    def seq_start(self):
        if self.runner and self.runner.running:
            return
        if not self.steps:
            self.lbl_seq.setText("No hay una secuencia cargada.")
            return
        st = self.state
        moves = any(s["kind"] in ("goto", "ang") for s in self.steps)
        screws = any(s["kind"] in ("screw", "tool") for s in self.steps)
        if st is None:
            self.lbl_seq.setText("No se puede iniciar: sin datos del robot.")
            self._texts[id(self.lbl_seq)] = self.lbl_seq.text()
            return
        if st.faulted:
            self.lbl_seq.setText("No se puede iniciar: hay una falla o paro activo.")
            return
        if moves and not ((st.home1 or not st.j1_on) and (st.home2 or not st.j2_on)):
            self.lbl_seq.setText("No se puede iniciar: primero haz Home.")
            return
        if screws and not st.m3_on:
            self.lbl_seq.setText("No se puede iniciar: activa M3 en la pestaña Motores.")
            return
        for label, row in self.hole_rows.items():
            self._hole_row(row, "Pendiente", {})
        self.results_shown, self.hole_active = 0, None
        self.runner = sq.SequenceRunner(self.steps, self.spin_repeat.value(), self.send, time.monotonic)
        self.runner.start()
        self._paint_holes()
        self.btn_seq_start.setEnabled(False)
        self.btn_seq_stop.setEnabled(True)
        self.chk_ensayo.setEnabled(False)

    def seq_stop(self):
        if self.runner:
            self.runner.stop()

    def _update_sequence(self):
        r = self.runner
        if r is None:
            return
        r.tick()
        self._update_holes(r)
        total = str(r.repeat) if r.repeat else "∞"
        last = (" · último ciclo %.1f s" % r.cycle_times[-1]) if r.cycle_times else ""
        text = {"running": "Ciclo %d de %s · paso %d de %d%s" % (r.cycles_done + 1, total, r.index + 1, len(r.steps), last),
                "done": "Terminó: %d ciclos%s" % (r.cycles_done, last),
                "stopped": "Detenida en el ciclo %d, paso %d." % (r.cycles_done + 1, r.index + 1),
                "aborted": "ABORTADA en el paso %d: %s" % (r.index + 1, r.reason)}[r.state]
        if r.tool_left_down:
            text += " M3 puede haber quedado abajo: súbelo a mano antes de mover el brazo."
        self._set(self.lbl_seq, text)
        if r.running:
            if self.seq_list.currentRow() != r.index:
                self.seq_list.setCurrentRow(r.index)
        elif self.seq_shown != r.state or self.btn_seq_stop.isEnabled():
            self.btn_seq_start.setEnabled(True)
            self.btn_seq_stop.setEnabled(False)
            self.chk_ensayo.setEnabled(True)
            self.seq_shown = r.state

    def _update_holes(self, r):
        changed = False
        for res in r.results[self.results_shown:]:
            if res["hueco"] in self.hole_rows:
                self._hole_row(self.hole_rows[res["hueco"]], res["resultado"], res)
            self.results_shown += 1
            changed = True
        step = r.steps[r.index] if r.running else None
        active = None
        if step and (step["kind"] == "screw" or "label" in step):
            active = step.get("label") or "paso %d" % (r.index + 1)
            self.target = None
        elif step and step["kind"] == "goto":
            self.target = (step["x"], step["y"])
        if active != self.hole_active:
            self.hole_active = active
            if active in self.hole_rows:
                self._hole_row(self.hole_rows[active], "Atornillando" if step["kind"] == "screw" else "Ensayando", {})
                self.hole_table.scrollToItem(self.hole_table.item(self.hole_rows[active], 0))
        if changed:
            self._paint_holes()

    # ------------------------------------------------------------------ teclado
    def keyPressEvent(self, ev):
        if ev.isAutoRepeat():
            return
        k = ev.key()
        if k in KEY_NAMES:
            self.keys.add(KEY_NAMES[k])
            self.link.set_keys(self.keys)
        elif k == QtCore.Qt.Key_E:
            self.do_estop()
        elif k in SHORTCUTS:
            self.send(SHORTCUTS[k])
            if k == QtCore.Qt.Key_Escape:
                self.setFocus()             # devuelve el teclado al robot si estaba en un cuadro de texto
        else:
            super().keyPressEvent(ev)

    def keyReleaseEvent(self, ev):
        if ev.isAutoRepeat():
            return
        k = ev.key()
        if k in KEY_NAMES:
            self.keys.discard(KEY_NAMES[k])
            self.link.set_keys(self.keys)
        else:
            super().keyReleaseEvent(ev)

    def mousePressEvent(self, ev):
        self.setFocus()                           # clic afuera de un cuadro: el teclado vuelve al robot
        super().mousePressEvent(ev)

    def release_all_keys(self):
        self.keys.clear()
        self.link.set_keys(self.keys)

    def changeEvent(self, ev):
        if ev.type() == QtCore.QEvent.ActivationChange and not self.isActiveWindow():
            self.release_all_keys()
        super().changeEvent(ev)

    def focusOutEvent(self, ev):
        self.release_all_keys()                   # pasó a un cuadro de texto u otra ventana: se suelta el dead-man
        super().focusOutEvent(ev)

    def closeEvent(self, ev):
        self.timer.stop()
        self.release_all_keys()
        super().closeEvent(ev)

    # ------------------------------------------------------------------ refresco
    def _set(self, label, text):
        """Solo toca el widget si cambió: es lo que evita redibujar la ventana entera 30 veces por segundo."""
        if self._texts.get(id(label)) != text:
            self._texts[id(label)] = text
            label.setText(text)

    def _handle_event(self, ev):
        color = CRIT if ev.startswith(("#ERROR", "#FALLA")) else GOOD if ev.startswith(("#LLEGO", "#HOME: terminado")) else None
        res = sq.parse_tornillo(ev)
        if res:
            color = GOOD if res["resultado"] in sq.RESULTADOS_BUENOS else CRIT
            self._set(self.lbl_screw_result, "Último tornillo: %s · unas %.1f vueltas, %.1f mm · %.1f s · revísalo a mano" % (
                res["resultado"], res["vueltas"], res["prof"], res["t"]))
        self.add_log(ev, color)
        if ev.startswith("#FALLA"):
            self._footer_on = True
            self._set(self.lbl_key, ev)
        if ev.startswith("#MAPA") or "J1_AJUSTE" in ev or "J2_AJUSTE" in ev:
            self._set(self.lbl_cal, ev.split(":", 1)[-1].strip().replace(" | ", "\n"))
        if self.runner:
            self.runner.on_event(ev)

    def tick(self):
        self.link.gui_beat()
        self.ticks += 1
        events = self.link.pop_events(EVENTS_PER_TICK)
        if self._footer_on and not any(ev.startswith("#FALLA") for ev in events):
            self._footer_on = False
            self._set(self.lbl_key, "")
        for ev in events:
            self._handle_event(ev)
        raw, age, rate = self.link.snapshot()
        st = sl.decode_state(raw) if age < STALE_S else None
        self.state = st
        self._update_link(st is not None, rate)
        self._update_keyboard_hint()
        if st is not None:
            self._update_robot(st)
        self._update_sequence()
        self._renew_tool(st)
        if self.ticks % PLOT_EVERY == 0:
            self._draw()

    def _update_link(self, ok, rate):
        self._set(self.lbl_link, ("Conectado · %.0f Hz" % rate) if ok else "SIN DATOS DEL ROBOT")
        wanted = GOOD if ok else CRIT
        if self._texts.get("link_color") != wanted:
            self._texts["link_color"] = wanted
            self.lbl_link.setStyleSheet(f"color: {wanted}; font-weight: 600;")

    def _update_keyboard_hint(self):
        typing = isinstance(QtWidgets.QApplication.focusWidget(), (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox))
        if typing:
            self._set(self.lbl_jog, "Teclado del robot en pausa: estás escribiendo en un cuadro.")
            return
        dm, d1, d2 = self.link.jog_active()
        if dm:
            self._set(self.lbl_jog, "Dead-man ACTIVO · J1 %+d · J2 %+d" % (d1, d2))
        else:
            self._set(self.lbl_jog, "Dead-man suelto: el robot no se mueve con el teclado.")

    def _update_robot(self, st):
        if st.mode != self._mode_shown:
            self._mode_shown = st.mode
            self.lbl_mode.setText(st.mode)
            self.lbl_mode.setStyleSheet(self._pill(MODE_COLORS.get(st.mode, MODE_COLORS["OFF"])))
        if st.estop:
            detail = "Paro de emergencia activo. Revisa el robot y pulsa Rearmar."
        elif st.faulted:
            detail = "Falla: %s. Pulsa Rearmar y repite Home." % st.fault_text
        elif not (st.home1 or not st.j1_on) or not (st.home2 or not st.j2_on):
            detail = MODE_TEXT.get(st.mode, st.mode) + ". Falta hacer Home."
        else:
            detail = MODE_TEXT.get(st.mode, st.mode)
        self._set(self.lbl_detail, detail)
        self._set(self.lbl_pose, "q1 %7.2f°  q2 %7.2f°  x %6.1f  y %6.1f mm" % (st.q1, st.q2, st.x, st.y))
        self._set(self.lbl_ctrl, st.ctrl)
        rearm = st.faulted
        if self.btn_rearm.isEnabled() != rearm:
            self.btn_rearm.setEnabled(rearm)
            self.btn_rearm.setStyleSheet(f"background: {WARN}; color: white; font-weight: 700;" if rearm else "")

        now = time.monotonic()
        on = (st.j1_on, st.j2_on, st.m3_on)
        homes = (st.home1, st.home2)
        fcs = ((st.fc_j1_min, st.fc_j1_max), (st.fc_j2_min, st.fc_j2_max))
        qs, duty = (st.q1, st.q2), (st.duty1, st.duty2)
        for m in range(3):
            pend = self.motor_pending.get(m)
            if pend is not None and (now >= pend or self.motor_btn[m].isChecked() == on[m]):
                del self.motor_pending[m]
            if m not in self.motor_pending and self.motor_btn[m].isChecked() != on[m]:
                self._show_motor(m, on[m])
            if m < 2:
                tope = "tope MIN" if fcs[m][0] else "tope MAX" if fcs[m][1] else "sin tope"
                self._set(self.motor_info[m], "%.1f° · PWM %.0f · %s · %s" % (qs[m], duty[m], "referenciado" if homes[m] else "sin referencia", tope))
            else:
                self._set(self.motor_info[m], "PWM %.0f · servo %.0f°" % (st.tool, st.servo))
        if self.btn_tool.isEnabled() != st.m3_on:
            self.btn_tool.setEnabled(st.m3_on)
            if not st.m3_on:
                self._tool_stop_local()
        if st.faulted and self.btn_tool.isChecked():
            self._tool_stop_local()

        self._set(self.lbl_screw_vals, "%-14s PWM %5.0f  servo %3.0f°\n"
                  "Vueltas  %6.2f     Profundidad %5.2f mm" % (st.screw_fase, st.tool, st.servo, st.tool_vueltas, st.tool_prof))

        i = self.buf_i % PLOT_POINTS
        self.buf_t[i] = now - self.t0
        for k, v in (("q1", st.q1), ("q2", st.q2), ("r1", st.ref1), ("r2", st.ref2), ("d1", st.duty1), ("d2", st.duty2)):
            self.buf[k][i] = v
        self.buf_i += 1
        if st.mode != "OFF" or self.trail_i == 0:
            self.trail[self.trail_i % len(self.trail)] = (st.x, st.y)
            self.trail_i += 1

    def _renew_tool(self, st):
        if self.btn_tool.isChecked() and st is not None and time.monotonic() - self.tool_t > TOOL_RENEW_S:
            self.tool_t = time.monotonic()
            self.link.command("tool %d" % self.tool_slider.value())   # sin registrar: se repite cada pocos segundos

    def _draw(self):
        n = min(self.buf_i, PLOT_POINTS)
        if n < 2:
            return
        end = self.buf_i % PLOT_POINTS
        order = np.arange(end - n, end) % PLOT_POINTS
        t = self.buf_t[order]
        self.c_q1.setData(t, self.buf["q1"][order])
        self.c_r1.setData(t, self.buf["r1"][order])
        self.c_q2.setData(t, self.buf["q2"][order])
        self.c_r2.setData(t, self.buf["r2"][order])
        self.c_d1.setData(t, self.buf["d1"][order])
        self.c_d2.setData(t, self.buf["d2"][order])
        tn = t[-1]
        self.p_q1.setXRange(tn - PLOT_SECONDS, tn, padding=0)

        i = (self.buf_i - 1) % PLOT_POINTS
        a, b = math.radians(self.buf["q1"][i]), math.radians(self.buf["q1"][i] + self.buf["q2"][i])
        ex, ey = L1 * math.cos(a), L1 * math.sin(a)
        tx, ty = ex + L2 * math.cos(b), ey + L2 * math.sin(b)
        self.xy_arm.setData([0, ex, tx], [0, ey, ty])
        self.xy_joints.setData([0, ex, tx], [0, ey, ty])
        ra, rb = math.radians(self.buf["r1"][i]), math.radians(self.buf["r1"][i] + self.buf["r2"][i])
        self.xy_ref.setData([L1 * math.cos(ra) + L2 * math.cos(rb)], [L1 * math.sin(ra) + L2 * math.sin(rb)])
        if self.target:
            self.xy_target.setData([self.target[0]], [self.target[1]])
        m = min(self.trail_i, len(self.trail))
        if m > 1:
            e = self.trail_i % len(self.trail)
            idx = np.arange(e - m, e) % len(self.trail)
            self.xy_trail.setData(self.trail[idx, 0], self.trail[idx, 1])


def start_simulator(with_rviz):
    if with_rviz:
        cmd = ["ros2", "launch", os.path.join(HERE, "scara_rviz.launch.py"), "sim:=true"]
    else:
        cmd = [sys.executable, os.path.join(HERE, "scara_sim_node.py")]
    return subprocess.Popen(cmd, start_new_session=True)


def main():
    ap = argparse.ArgumentParser(description="Panel de control del SCARA")
    ap.add_argument("--sim", action="store_true", help="sin robot: arranca el simulador (y RViz)")
    ap.add_argument("--sin-rviz", action="store_true", help="con --sim: solo el simulador")
    a = ap.parse_args()
    if a.sin_rviz and not a.sim:
        ap.error("--sin-rviz solo tiene sentido con --sim")

    sim_proc = start_simulator(not a.sin_rviz) if a.sim else None
    link = sl.ScaraLink()
    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=False, background=CARD, foreground=MUTED)
    panel = ControlPanel(link, sim=a.sim)
    panel.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    code = 0
    try:
        code = app.exec_()
    finally:
        link.close()
        if sim_proc:
            os.killpg(sim_proc.pid, signal.SIGTERM)
            try:
                sim_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(sim_proc.pid, signal.SIGKILL)
    sys.exit(code)


if __name__ == "__main__":
    main()
