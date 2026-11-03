#!/usr/bin/env python3
"""
Proceso repetitivo: lee una secuencia de un archivo YAML y la ejecuta orden por orden, esperando a que el robot
confirme cada punto. No usa ROS: quien la usa le pasa una función para enviar comandos y le entrega los eventos.

Dos formatos (ver ros2/secuencia.yaml):

    repeticiones: 1
    proceso:                   # atornillar todos los huecos de unas placas; se expande a pasos
      base_cm, origen_cm, codo, placas_cm, lado_mm, hueco_borde_mm, asentar_s, ref_cada, servo, tornillo, ensayo

    repeticiones: 3            # 0 = hasta que se detenga a mano
    pasos:
      - ir: [120, 80]          # x, y [mm] desde la base
      - angulos: [20, 90]      # q1, q2 [grados]
      - esperar: 0.5           # segundos
      - codo: -1
      - servo: {accion: abrir, tiempo: 0.4}     # abrir, cerrar o grados
      - atornillar: {vueltas: 12, torque: 0.4}
      - m3: {potencia: 40, tiempo: 1.5}         # motor 3 sin control: -100..100 %, durante un tiempo [s]
      - recorrer: true         # recorre los ejes de tope a tope: corrige la posición y la velocidad

Si el robot informa una falla o rechaza un punto, la secuencia aborta. Lo único que manda después es apagar M3 y cerrar el
servo, si estaban en marcha. Sin encoders, un tornillo que queda
mal NO se detecta: el resultado es "sin_medir" y hay que revisarlo a mano.

El firmware de emergencia necesita algo más que el principal: recorrer los ejes cada pocos huecos, porque el error se acumula.
Por eso M3 debe estar arriba antes de cada recorrido: hacerlo con la punta abajo rompería algo.

Ensayo: load_sequence_full(path, ensayo=True) recorre los mismos huecos sin servo ni tornillo. En cada uno, M3 baja un
tiempo y sube con la misma potencia durante el mismo tiempo. Solo aplica a los archivos con sección "proceso".
"""
import math
import re
from dataclasses import dataclass, field

import yaml

L1_MM, L2_MM = 120.0, 70.0            # mantener igual que KIN en src/config.c
SING_MARGIN_RAD = math.radians(5.0)
Q1_LIM_DEG = (-105.0, 117.0)          # lo que permite el firmware tras el Home; J1 tiene el tope en -108
Q2_LIM_DEG = (-140.0, 140.0)
ORIGEN_RADIO_MM = 188.0               # alcance máximo fuera de la singularidad, con 2 mm de margen
STEP_TIMEOUT_S = 40.0                 # un movimiento tarda menos; si no, algo anda mal
REF_TIMEOUT_S = 90.0                  # recorrer los ejes tarda unos 16 s. El firmware se cuida solo: esto es por si se corta el enlace
SCREW_TIMEOUT_S = 300.0               # apretar por tiempo: 23 s con los valores normales, mucho más con pocos N·m
TOOL_RENEW_S = 5.0                    # debe ser menor que TOOL_MAX_ON_MS del firmware
ENSAYO_PCT, ENSAYO_S = 40.0, 3.5      # potencia y tiempo de bajada del ensayo si el archivo no los da
SUBIR_PCT, SUBIR_VPS = 35.0, 0.647    # el PWM de apretar (229), y las vueltas por segundo a ese PWM según el modelo de M3
REF_CADA = 4                          # huecos entre correcciones si el archivo no lo dice
RESULTADOS_BUENOS = ("sin_medir", "ensayado")

_TORNILLO = re.compile(r"#TORNILLO: (\w+) vueltas=(-?[\d.]+) prof=(-?[\d.]+)mm avance=(-?[\d.]+)% torque=(-?[\d.]+)Nm "
                       r"fuerza=(-?[\d.]+)N presion=(-?[\d.]+)MPa corriente=(-?[\d.]+)A t=(-?[\d.]+)s")
_TORNILLO_CAMPOS = ("vueltas", "prof", "avance", "torque", "fuerza", "presion", "corriente", "t")


class SequenceError(Exception):
    """Archivo de secuencia no válido. El mensaje está pensado para mostrarlo al usuario."""


@dataclass
class Process:
    """Geometría del proceso, en mm desde la base. Sirve para dibujar las placas."""
    boards: dict
    holes: list
    origin: tuple
    elbow: int
    side: float


@dataclass
class Sequence:
    steps: list
    repeat: int
    process: Process = None
    notes: list = field(default_factory=list)


def parse_tornillo(text):
    """Evento "#TORNILLO: ..." -> diccionario con "resultado" y los números; None si es otro evento."""
    m = _TORNILLO.search(text)
    if not m:
        return None
    return dict(zip(_TORNILLO_CAMPOS, (float(v) for v in m.groups()[1:])), resultado=m.group(1))


def point_problem(x, y, elbow=None):
    """None si el brazo llega a (x, y) [mm]; si no, el motivo. Con elbow también revisa los límites de ese codo."""
    r = math.hypot(x, y)
    if r > L1_MM + L2_MM or r < abs(L1_MM - L2_MM):
        return "(%g, %g) está fuera de alcance: %.0f a %.0f mm" % (x, y, abs(L1_MM - L2_MM), L1_MM + L2_MM)
    c2 = (r * r - L1_MM ** 2 - L2_MM ** 2) / (2.0 * L1_MM * L2_MM)
    q2 = math.acos(max(-1.0, min(1.0, c2)))
    if q2 < SING_MARGIN_RAD or q2 > math.pi - SING_MARGIN_RAD:
        return "(%g, %g) está en una postura singular del brazo" % (x, y)
    if elbow is not None:
        q2 = elbow * q2
        q1 = math.atan2(y, x) - math.atan2(L2_MM * math.sin(q2), L1_MM + L2_MM * math.cos(q2))
        q1 = math.degrees((q1 + math.pi) % (2 * math.pi) - math.pi)
        q2 = math.degrees(q2)
        if not (Q1_LIM_DEG[0] <= q1 <= Q1_LIM_DEG[1] and Q2_LIM_DEG[0] <= q2 <= Q2_LIM_DEG[1]):
            return "(%.1f, %.1f) se sale de los límites con codo %+d: q1 = %.0f, q2 = %.0f grados" % (x, y, elbow, q1, q2)
    return None


def _numbers(value, n, name):
    if not isinstance(value, (list, tuple)) or len(value) != n or not all(isinstance(v, (int, float)) for v in value):
        raise ValueError("%s necesita %d numeros" % (name, n))
    return [float(v) for v in value]


def _parse_step(raw):
    if not isinstance(raw, dict) or len(raw) != 1:
        raise ValueError("cada paso es una sola orden, por ejemplo  - ir: [120, 80]")
    key, val = next(iter(raw.items()))
    if key == "ir":
        x, y = _numbers(val, 2, "ir")
        problem = point_problem(x, y)
        if problem:
            raise ValueError(problem)
        return _goto(x, y)
    if key == "angulos":
        a, b = _numbers(val, 2, "angulos")
        return {"kind": "ang", "a": a, "b": b, "desc": "Ángulos q1 = %g, q2 = %g grados" % (a, b)}
    if key == "esperar":
        if not isinstance(val, (int, float)) or val < 0:
            raise ValueError("esperar necesita un tiempo positivo en segundos")
        return _wait(float(val))
    if key == "m3":
        if not isinstance(val, dict) or "potencia" not in val or "tiempo" not in val:
            raise ValueError("m3 necesita {potencia: %, tiempo: s}")
        pct, secs = val["potencia"], val["tiempo"]
        if not isinstance(pct, (int, float)) or abs(pct) > 100:
            raise ValueError("la potencia de m3 va de -100 a 100 %")
        if not isinstance(secs, (int, float)) or secs <= 0:
            raise ValueError("el tiempo de m3 debe ser positivo")
        return {"kind": "tool", "pct": float(pct), "secs": float(secs),
                "desc": "Motor 3 al %g %% durante %g s" % (pct, secs)}
    if key == "codo":
        if val not in (1, -1):
            raise ValueError("codo es 1 o -1")
        return _cmd("elbow %+d" % val, "Codo %+d" % val)
    if key == "servo":
        if not isinstance(val, dict) or "accion" not in val:
            raise ValueError("servo necesita {accion: abrir | cerrar | grados, tiempo: s}")
        acc, secs = val["accion"], val.get("tiempo", 0.0)
        if not (acc in ("abrir", "cerrar") or (isinstance(acc, (int, float)) and 0 <= acc <= 180)):
            raise ValueError("la acción del servo es abrir, cerrar o de 0 a 180 grados")
        if not isinstance(secs, (int, float)) or secs < 0:
            raise ValueError("el tiempo del servo debe ser positivo")
        return _cmd("servo %s" % acc, "Servo: %s" % acc, float(secs))
    if key == "atornillar":
        if not isinstance(val, dict) or "vueltas" not in val or "torque" not in val:
            raise ValueError("atornillar necesita {vueltas: n, torque: N·m}")
        v, t = val["vueltas"], val["torque"]
        if not isinstance(v, (int, float)) or not 0 < v <= 40 or not isinstance(t, (int, float)) or not 0 < t <= 1.5:
            raise ValueError("atornillar: vueltas de 0 a 40 y torque de 0 a 1.5 N·m")
        return _screw(float(v), float(t))
    if key == "recorrer":
        return _ref()
    raise ValueError("orden desconocida: %s (usa ir, angulos, esperar, codo, servo, atornillar, m3 o recorrer)" % key)


def _goto(x, y, desc=None):
    return {"kind": "goto", "x": x, "y": y, "desc": desc or "Ir a x = %g, y = %g mm" % (x, y)}


def _wait(secs, desc=None):
    return {"kind": "wait", "secs": secs, "desc": desc or "Esperar %g s" % secs}


def _cmd(text, desc, secs=0.0):
    return {"kind": "cmd", "text": text, "secs": secs, "desc": desc}


def _screw(vueltas, torque, label=None):
    s = {"kind": "screw", "vueltas": vueltas, "torque": torque,
         "desc": "Atornillar %s(%g vueltas, %g N·m)" % (label + " " if label else "", vueltas, torque)}
    if label:
        s["label"] = label
    return s


def _ref():
    return {"kind": "ref", "desc": "Recorrer los ejes de tope a tope"}


def _pair(val, name):
    return _numbers(val, 2, name)


def _positive(raw, key, name, default=None):
    val = raw.get(key, default)
    if not isinstance(val, (int, float)) or val < 0:
        raise ValueError("%s debe ser un número positivo" % name)
    return float(val)


def _rehearse_hole(label, pct, secs, settle):
    """M3 baja y sube por tiempo: no hay medición de altura. La espera final deja que se detenga antes de mover el brazo."""
    down = {"kind": "tool", "pct": pct, "secs": secs, "label": label, "desc": "Bajar M3 en %s (%g %%, %g s)" % (label, pct, secs)}
    up = {"kind": "tool", "pct": -pct, "secs": secs, "label": label, "fin": True, "desc": "Subir M3 en %s" % label}
    pause = dict(_wait(settle, "Pausa abajo"), label=label)   # sin esta pausa, M3 pierde altura al invertir el giro
    return [down, pause, up, _wait(settle, "Asentar")]


def _raise_m3(label, pct, secs, settle):
    """Tras apretar, M3 sube por tiempo. Se gira al revés con el mismo PWM: se supone que la punta se suelta sola del tornillo, ver README."""
    pause = dict(_wait(settle, "Pausa abajo"), label=label)
    up = {"kind": "tool", "pct": -pct, "secs": secs, "label": label, "desc": "Subir M3 en %s (%g s)" % (label, secs)}
    return [pause, up, _wait(settle, "Asentar")]


def _expand_process(raw, ensayo=False):
    """Sección "proceso" -> (pasos, Process, notas)."""
    if not isinstance(raw, dict):
        raise ValueError("proceso debe ser una sección con sus datos")
    bx, by = _pair(raw.get("base_cm"), "base_cm")
    elbow = raw.get("codo", -1)
    if elbow not in (1, -1):
        raise ValueError("codo es 1 o -1")
    side, edge = _positive(raw, "lado_mm", "lado_mm"), _positive(raw, "hueco_borde_mm", "hueco_borde_mm")
    if 2 * edge >= side:
        raise ValueError("los huecos no caben: hueco_borde_mm debe ser menor que la mitad de lado_mm")
    placas = raw.get("placas_cm")
    if not isinstance(placas, dict) or not placas:
        raise ValueError("placas_cm necesita al menos una placa, por ejemplo A: [29, -5]")
    servo, tornillo = raw.get("servo") or {}, raw.get("tornillo") or {}
    t_open, t_fall = _positive(servo, "abrir_s", "servo.abrir_s", 0.4), _positive(servo, "caida_s", "servo.caida_s", 0.6)
    settle = _positive(raw, "asentar_s", "asentar_s", 0.3)
    vueltas, torque = _positive(tornillo, "vueltas", "tornillo.vueltas"), _positive(tornillo, "torque_nm", "tornillo.torque_nm")
    bloque = raw.get("ensayo") or {}
    pct, secs = _positive(bloque, "potencia", "ensayo.potencia", ENSAYO_PCT), _positive(bloque, "bajar_s", "ensayo.bajar_s", ENSAYO_S)
    if not 0 < pct <= 100 or secs == 0:
        raise ValueError("ensayo: potencia de 0 a 100 y bajar_s mayor que 0")

    up_pct = _positive(tornillo, "subir_pct", "tornillo.subir_pct", SUBIR_PCT)
    up_s = _positive(tornillo, "subir_s", "tornillo.subir_s", vueltas / SUBIR_VPS)
    if not 0 < up_pct <= 100 or up_s == 0:
        raise ValueError("tornillo: subir_pct de 0 a 100 y subir_s mayor que 0")
    ref_cada = raw.get("ref_cada", REF_CADA)
    if not isinstance(ref_cada, int) or ref_cada < 0:
        raise ValueError("ref_cada es un entero: cada cuántos huecos recorrer los ejes (0 = solo al inicio)")

    notes = []
    ox, oy = _pair(raw.get("origen_cm"), "origen_cm")
    ox, oy = (ox - bx) * 10.0, (oy - by) * 10.0
    r = math.hypot(ox, oy)
    if r > ORIGEN_RADIO_MM:
        notes.append("El origen pedido está a %.1f mm de la base y el brazo alcanza %.0f: se usa (%.1f, %.1f) mm, en la misma dirección."
                     % (r, L1_MM + L2_MM, ox * ORIGEN_RADIO_MM / r, oy * ORIGEN_RADIO_MM / r))
        ox, oy = ox * ORIGEN_RADIO_MM / r, oy * ORIGEN_RADIO_MM / r
    problem = point_problem(ox, oy, elbow)
    if problem:
        raise ValueError("origen: " + problem)

    half = side / 2.0 - edge
    boards, holes = {}, []
    steps = [_cmd("elbow %+d" % elbow, "Codo %+d" % elbow), _ref(), _goto(ox, oy, "Ir al origen")]
    for name, center in placas.items():
        cx, cy = _pair(center, "placa %s" % name)
        cx, cy = (cx - bx) * 10.0, (cy - by) * 10.0
        boards[str(name)] = (cx, cy)
        # H1 y H2 a la izquierda, H3 y H4 a la derecha, como en la imagen; se recorren por el borde para no cruzar la placa.
        for hole, sx, sy in (("H1", -1, -1), ("H3", 1, -1), ("H4", 1, 1), ("H2", -1, 1)):
            label = "%s-%s" % (name, hole)
            x, y = cx + sx * half, cy + sy * half
            problem = point_problem(x, y, elbow)
            if problem:
                raise ValueError("hueco %s: %s" % (label, problem))
            if ref_cada and holes and len(holes) % ref_cada == 0:
                steps.append(_ref())
            holes.append({"label": label, "board": str(name), "x": x, "y": y})
            steps += [_goto(x, y, "Ir a %s (%.1f, %.1f) mm" % (label, x, y)), _wait(settle, "Asentar")]
            if ensayo:
                steps += _rehearse_hole(label, pct, secs, settle)
            else:
                steps += [_cmd("servo abrir", "Servo: abrir"), _wait(t_open, "Tornillo cayendo"),
                          _cmd("servo cerrar", "Servo: cerrar"), _wait(t_fall, "Tornillo cayendo"),
                          _screw(vueltas, torque, label)] + _raise_m3(label, up_pct, up_s, settle)
    steps.append(_goto(ox, oy, "Volver al origen"))
    return steps, Process(boards, holes, (ox, oy), elbow, side), notes


def load_sequence_full(path, ensayo=False):
    """Devuelve un Sequence. Lanza SequenceError con un mensaje claro si el archivo no sirve."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise SequenceError("no se pudo leer %s: %s" % (path, e)) from e
    if not isinstance(data, dict) or not (data.get("pasos") or data.get("proceso")):
        raise SequenceError("el archivo debe tener una sección proceso o una lista de pasos")
    repeat = data.get("repeticiones", 1)
    if not isinstance(repeat, int) or repeat < 0:
        raise SequenceError("repeticiones debe ser un entero mayor o igual a 0")
    if data.get("proceso"):
        try:
            steps, process, notes = _expand_process(data["proceso"], ensayo)
        except ValueError as e:
            raise SequenceError("proceso: %s" % e) from e
        return Sequence(steps, repeat, process, notes)
    if not isinstance(data["pasos"], list):
        raise SequenceError("pasos debe ser una lista")
    steps = []
    for i, raw in enumerate(data["pasos"], start=1):
        try:
            steps.append(_parse_step(raw))
        except ValueError as e:
            raise SequenceError("paso %d: %s" % (i, e)) from e
    return Sequence(steps, repeat)


def load_sequence(path):
    """Devuelve (pasos, repeticiones)."""
    seq = load_sequence_full(path)
    return seq.steps, seq.repeat


def _fmt(v):
    return "%g" % v


class SequenceRunner:
    """Máquina de estados: idle -> running -> done | stopped | aborted. Se llama tick() con frecuencia."""

    def __init__(self, steps, repeat, send, clock, step_timeout=STEP_TIMEOUT_S):
        self.steps, self.repeat, self.send, self.clock = steps, repeat, send, clock
        self.step_timeout = step_timeout
        self.state = "idle"
        self.reason = ""
        self.index = 0
        self.cycles_done = 0
        self.cycle_times = []
        self.results = []                 # un diccionario por tornillo terminado
        self._arrived = False
        self._screw_result = None
        self._servo_open = False
        self._t_step = self._t_cycle = self._t_renew = 0.0

    @property
    def running(self):
        return self.state == "running"

    @property
    def tool_left_down(self):
        """Se cortó con M3 abajo: no se devuelve solo, y el brazo no debería moverse así."""
        if self.state not in ("stopped", "aborted"):
            return False
        return "label" in self.steps[self.index]

    def start(self):
        self.state, self.reason = "running", ""
        self.index = self.cycles_done = 0
        self.cycle_times = []
        self.results = []
        self._servo_open = False
        self._t_cycle = self.clock()
        self._begin_step()

    def stop(self):
        if not self.running:
            return
        self._tool_off_if_needed()
        self.send("stop")
        self.state = "stopped"

    def abort(self, reason):
        """Corta la secuencia sin enviar nada más (por ejemplo, tras un paro de emergencia)."""
        if self.running:
            self.state, self.reason = "aborted", reason

    def on_event(self, text):
        if not self.running:
            return
        if text.startswith("#LLEGO"):
            self._arrived = True
        elif text.startswith("#TORNILLO"):
            self._screw_result = parse_tornillo(text)
        elif text.startswith(("#FALLA", "#ERROR")):
            self._abort(text.split(":", 1)[-1].strip(), send_stop=False)

    def tick(self):
        if not self.running:
            return
        step = self.steps[self.index]
        now = self.clock()
        kind = step["kind"]
        if kind in ("goto", "ang", "ref"):
            if self._arrived:
                self._next_step()
            elif now - self._t_step > (REF_TIMEOUT_S if kind == "ref" else self.step_timeout):
                self._abort("se agotó el tiempo esperando llegar al paso %d" % (self.index + 1), send_stop=True)
        elif kind == "wait":
            if now - self._t_step >= step["secs"]:
                self._next_step()
        elif kind == "cmd":
            if now - self._t_step >= step["secs"]:
                self._next_step()
        elif kind == "screw":
            if self._screw_result is not None:
                self._finish_screw(step, self._screw_result)
            elif now - self._t_step > SCREW_TIMEOUT_S:
                self._abort("se agotó el tiempo atornillando en el paso %d" % (self.index + 1), send_stop=True)
        elif kind == "tool":
            if now - self._t_step >= step["secs"]:
                self.send("tool 0")
                if step.get("fin"):
                    self.results.append({"hueco": step["label"], "resultado": "ensayado", "ciclo": self.cycles_done + 1})
                self._next_step()
            elif now - self._t_renew >= TOOL_RENEW_S:
                self._t_renew = now
                self.send("tool " + _fmt(step["pct"]))

    def _finish_screw(self, step, res):
        where = step.get("label") or "paso %d" % (self.index + 1)
        self.results.append(dict(res, hueco=where, ciclo=self.cycles_done + 1))
        if res["resultado"] in RESULTADOS_BUENOS:
            self._next_step()
        else:
            self._abort("tornillo %s: %s (%.1f vueltas)" % (where, res["resultado"], res["vueltas"]), send_stop=False)

    def _begin_step(self):
        step = self.steps[self.index]
        self._arrived = False
        self._screw_result = None
        self._t_step = self._t_renew = self.clock()
        if step["kind"] == "goto":
            self.send("goto %s %s" % (_fmt(step["x"]), _fmt(step["y"])))
        elif step["kind"] == "ang":
            self.send("ang %s %s" % (_fmt(step["a"]), _fmt(step["b"])))
        elif step["kind"] == "cmd":
            self.send(step["text"])
            if step["text"].startswith("servo "):
                self._servo_open = step["text"] != "servo cerrar"
        elif step["kind"] == "screw":
            self.send("screw %s %s" % (_fmt(step["vueltas"]), _fmt(step["torque"])))
        elif step["kind"] == "tool":
            self.send("tool " + _fmt(step["pct"]))
        elif step["kind"] == "ref":
            self.send("ref")

    def _next_step(self):
        self.index += 1
        if self.index >= len(self.steps):
            now = self.clock()
            self.cycles_done += 1
            self.cycle_times.append(now - self._t_cycle)
            self._t_cycle = now
            self.index = 0
            if self.repeat and self.cycles_done >= self.repeat:
                self.state = "done"
                return
        self._begin_step()

    def _tool_off_if_needed(self):
        step = self.steps[self.index]
        if step["kind"] == "tool":
            self.send("tool 0")
        if self._servo_open:                 # abrir dura 0 s: casi siempre se llega aquí en la espera de después
            self.send("servo cerrar")
            self._servo_open = False

    def _abort(self, reason, send_stop):
        self._tool_off_if_needed()
        if send_stop:
            self.send("stop")
        self.state, self.reason = "aborted", reason
