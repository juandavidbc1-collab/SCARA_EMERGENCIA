#!/usr/bin/env python3
"""
Cuánto se equivoca el brazo al llegar a cada hueco con cosas que Home no vio, usando el firmware real y la secuencia real.
Se calibra con el robot nominal (lo que haría el usuario con un transportador) y luego cambian las condiciones.

    python3 tools/sim/precision.py

Cada fila: mediana y peor error en mm, y cuántas corridas abortaron porque el firmware notó que el brazo se había perdido.
Ojo: lo simulado no incluye la holgura real de la guía ni cuánto se aparta el tornillo; sirve para comparar, no para prometer.
"""
import ctypes
import os
import subprocess
import sys
import tempfile

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2"))
import scara_sequence as sq  # noqa: E402

DT_MS = 5
# nombre, voltaje, fricción, ruido de fricción en cada arranque, juego [grados], error del rango [grados], cambia desde el hueco
ESCENARIOS = (
    ("Nominal", 1.00, 1.00, 0.00, 0.0, 0.0, 0),
    ("Juego del reductor 0,5 grados", 1.00, 1.00, 0.00, 0.5, 0.0, 0),
    ("Rango mal medido 1 grado", 1.00, 1.00, 0.00, 0.0, 1.0, 0),
    ("Voltaje 3 % menor", 0.97, 1.00, 0.00, 0.0, 0.0, 0),
    ("Fricción 10 % mayor", 1.00, 1.10, 0.00, 0.0, 0.0, 0),
    ("Fricción 10 % mayor desde el hueco 7", 1.00, 1.10, 0.00, 0.0, 0.0, 6),
    ("Fricción varía 1 % en cada arranque", 1.00, 1.00, 0.01, 0.0, 0.0, 0),
    ("Fricción varía 2 % en cada arranque", 1.00, 1.00, 0.02, 0.0, 0.0, 0),
    ("Todo junto", 0.97, 1.10, 0.02, 0.5, 1.0, 0),
)
SEMILLAS = 20


def cargar():
    subprocess.run(["sh", os.path.join(ROOT, "tools", "sim", "build.sh")], check=True)
    lib = ctypes.CDLL(os.path.join(ROOT, "tools", "sim", "build", "libscara_sim.so"))
    lib.sim_cmd.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
    lib.sim_event_pop.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.sim_state.argtypes = [ctypes.POINTER(ctypes.c_float)]
    lib.sim_fijo.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.sim_cambio.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
    lib.sim_real_deg.argtypes = [ctypes.POINTER(ctypes.c_double)]
    lib.sim_error_mm.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.sim_error_mm.restype = ctypes.c_double
    return lib


class Robot:
    def __init__(self, lib):
        self.lib, self.buf, self.st, self.events = lib, ctypes.create_string_buffer(240), (ctypes.c_float * 24)(), []

    def cmd(self, text):
        self.lib.sim_cmd(text.encode(), self.buf, len(self.buf))

    def correr(self, ms):
        self.lib.sim_step_ms(ms)
        while self.lib.sim_event_pop(self.buf, len(self.buf)):
            self.events.append(self.buf.value.decode())

    def hasta_quieto(self, segundos):
        for _ in range(int(segundos * 1000 / 20)):
            self.correr(20)
            self.lib.sim_state(self.st)
            if int(self.st[4]) in (0, 2, 6):
                return int(self.st[4])
        raise RuntimeError("no terminó: %s" % self.events[-3:])

    def home(self):
        self.cmd("home")
        self.correr(20)
        return self.hasta_quieto(240)


def calibrar(lib):
    r = Robot(lib)
    lib.sim_init()
    r.correr(5)
    r.home()
    for orden, t in (("ref", 60), ("ang 60 40", 30)):
        r.cmd(orden)
        r.correr(20)
        r.hasta_quieto(t)
    q = (ctypes.c_double * 2)()
    lib.sim_real_deg(q)
    r.cmd("ajuste 1 60 %.1f" % q[0])
    r.cmd("ajuste 2 40 %.1f" % q[1])
    r.correr(5)
    print("Calibración:", [e for e in r.events if "AJUSTE" in e], flush=True)


def corrida(lib, seq, esc, semilla):
    _, v, tc, ruido, juego, rango, desde = esc
    lib.sim_seed(semilla)
    lib.sim_init()
    r = Robot(lib)
    lib.sim_fijo(juego, rango)
    r.correr(5)
    r.cmd("motor 3 on")
    r.correr(5)
    if r.home() != 2:
        return [], "falló Home"
    lib.sim_cambio(*((v, tc, ruido) if not desde else (1.0, 1.0, ruido)))
    huecos = {h["label"]: h for h in seq.process.holes}
    now = [0.0]
    run = sq.SequenceRunner(seq.steps, 1, r.cmd, lambda: now[0])
    run.start()
    errores, medidos, visto = [], set(), len(r.events)
    while run.running and now[0] < 1500:
        r.correr(DT_MS)
        now[0] += DT_MS * 1e-3
        for e in r.events[visto:]:
            run.on_event(e)
        visto = len(r.events)
        run.tick()
        if not run.running:
            break
        paso = run.steps[run.index]
        if paso["kind"] == "tool" and paso.get("pct", 0) > 0 and paso["label"] not in medidos:
            medidos.add(paso["label"])
            h = huecos[paso["label"]]
            errores.append(lib.sim_error_mm(h["x"], h["y"]))
            if desde and len(medidos) == desde + 1:
                lib.sim_cambio(v, tc, ruido)
    return errores, None if run.state == "done" else run.reason


def tabla(lib, ref_cada, escenarios):
    with open(os.path.join(ROOT, "ros2", "secuencia.yaml")) as f:
        data = yaml.safe_load(f)
    data["proceso"]["ref_cada"] = ref_cada
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump(data, f)
    seq = sq.load_sequence_full(f.name, ensayo=True)
    os.unlink(f.name)
    print("\n| Corrige cada %d huecos | Mediana mm | Peor mm | Corridas que abortaron |\n|---|---|---|---|" % ref_cada)
    for esc in escenarios:
        n = SEMILLAS if esc[3] else 1
        errores, fallas = [], []
        for s in range(n):
            e, fallo = corrida(lib, seq, esc, s + 1)
            errores += e
            if fallo:
                fallas.append(fallo)
        errores.sort()
        med, peor = (errores[len(errores) // 2], errores[-1]) if errores else (0.0, 0.0)
        print("| %s | %.2f | %.2f | %d de %d |" % (esc[0], med, peor, len(fallas), n), flush=True)
        if fallas:
            print("  (por ejemplo: %s)" % fallas[0], file=sys.stderr, flush=True)


def main():
    lib = cargar()
    calibrar(lib)
    tabla(lib, 4, ESCENARIOS)
    tabla(lib, 1, [ESCENARIOS[7]])
    tabla(lib, 0, [ESCENARIOS[4]])


if __name__ == "__main__":
    main()
