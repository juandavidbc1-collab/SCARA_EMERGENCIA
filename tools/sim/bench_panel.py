#!/usr/bin/env python3
"""
Mide el panel contra el simulador, sin pantalla:   python3 tools/sim/bench_panel.py [--capturas DIR]

  - Tiempo de cada refresco de la ventana con el robot en movimiento y las gráficas activas.
  - Regularidad del latido del dead-man.
  - Cuánto tarda en procesar una ráfaga de 100 eventos, comparado con el bucle del panel anterior
    (scara_teleop_ros.py: un solo mensaje de ROS por refresco, a 25 Hz).
"""
import argparse
import os
import statistics
import subprocess
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("BENCH_DOMAIN", str(150 + os.getpid() % 50))
os.environ["ROS_DOMAIN_ID"] = os.environ["BENCH_DOMAIN"]   # el mismo para el simulador y los procesos hijos
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "ros2"))
BURST = 100


def burst_process():
    code = ("import rclpy,time\nfrom std_msgs.msg import String\nfrom rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy\n"
            "rclpy.init();n=rclpy.create_node('rafaga')\n"
            "p=n.create_publisher(String,'/scara/event',QoSProfile(reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST,depth=200))\n"
            "time.sleep(2.0)\nprint(time.monotonic(),flush=True)\n"
            "[p.publish(String(data='#BENCH %d'%i)) for i in range(" + str(BURST) + ")]\ntime.sleep(4.0)\n")
    return subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)


def old_loop():
    """El bucle del panel anterior, sin la ventana: un spin_once por refresco."""
    import rclpy
    import scara_teleop_ros as old
    rclpy.init()
    node = old.make_node(rclpy)
    node.echo = False
    time.sleep(0.3)
    bp = burst_process()
    t_start, seen, t_last = float(bp.stdout.readline()), 0, None
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and seen < BURST:
        rclpy.spin_once(node, timeout_sec=0.0)
        node.publish_jog()
        while node.events:
            if node.events.popleft().startswith("#BENCH"):
                seen += 1
                t_last = time.monotonic()
        time.sleep(0.04)
    bp.kill()
    return seen, (t_last - t_start) if t_last else float("nan")


def new_panel(shots):
    import scara_link as sl
    import scara_panel as sp
    from PyQt5 import QtTest, QtWidgets
    app = QtWidgets.QApplication([])
    link = sl.ScaraLink("bench")
    p = sp.ControlPanel(link, sim=True)
    p.timer.stop()
    p.show()
    durations = []

    def run(seconds, until=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            t = time.perf_counter()
            p.tick()
            app.processEvents()
            durations.append((time.perf_counter() - t) * 1000)
            if until and until():
                return True
            QtTest.QTest.qWait(max(1, int(sp.TICK_MS - durations[-1])))
        return False

    def shot(name, tab=None):
        if shots:
            if tab is not None:
                p.tabs.setCurrentWidget(tab)
            run(0.3)
            p.grab().save(os.path.join(shots, name))

    run(1.0)
    link.command("home")
    if not run(40, lambda: p.state and p.state.home1 and p.state.home2 and p.state.mode == "HOLD"):
        raise SystemExit("Home no terminó. Estado: %s\nRegistro:\n%s" % (p.state, p.log.toPlainText()[-800:]))
    durations.clear()

    # movimiento con gráficas
    for x, y in ((150, 40), (60, 120), (110, 90)):
        link.goto(x, y, 0.0)
        p.target = (x, y)
        run(2.2)
    shot("panel_operar.png", p.tab_op)
    moving = list(durations)

    # latido: se mantiene el dead-man y se mide el período real, con la ventana refrescando
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    stamps = []
    best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    probe_ctx_node = rclpy.create_node("sonda")
    probe_ctx_node.create_subscription(Vector3, "/scara/jog", lambda m: stamps.append((time.monotonic(), m.x)), best)
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(probe_ctx_node)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()
    run(0.5)
    p.keys = {"space", "right"}
    link.set_keys(p.keys)
    run(2.0)
    t_stall = time.monotonic()
    time.sleep(0.5)                               # la ventana se congela con las teclas pulsadas
    p.release_all_keys()
    run(0.5)
    armed = [t for t, x in stamps if x > 0.5]
    gaps = [(b - a) * 1000 for a, b in zip(armed, armed[1:]) if b < t_stall]
    last_armed_after_stall = max((t for t in armed if t >= t_stall), default=t_stall) - t_stall

    # ráfaga de eventos
    bp = burst_process()
    t_start = float(bp.stdout.readline())
    t_last, seen = None, 0
    orig = p._handle_event

    def spy(ev):
        nonlocal t_last, seen
        if ev.startswith("#BENCH"):
            seen += 1
            t_last = time.monotonic()
        orig(ev)
    p._handle_event = spy
    run(3.0, lambda: seen >= BURST)
    p._handle_event = orig
    bp.kill()

    link.command("motor 3 on")
    run(0.3)
    shot("panel_motores.png", p.tab_motors)
    p.load_steps(p.steps, p.repeat)
    p.seq_start()
    run(30.0 if shots else 2.0, lambda: bool(p.runner and len(p.runner.results) >= 2))
    run(7.0 if shots else 0.0)             # a mitad del tercer tornillo
    shot("panel_atornillador.png", p.tab_screw)
    shot("panel_secuencia.png", p.tab_seq)
    p.seq_stop()
    run(0.3)
    link.command("mapa")
    run(0.5)
    shot("panel_calibracion.png", p.tab_ctrl)
    p.do_estop()
    run(0.6)
    shot("panel_paro.png", p.tab_op)
    p.close()
    link.close()
    return moving, gaps, last_armed_after_stall, seen, (t_last - t_start) if t_last else float("nan")


def child(mode, shots):
    if mode == "old":
        seen, t = old_loop()
        print("OLD %d %.3f" % (seen, t))
    else:
        moving, gaps, stall, seen, t = new_panel(shots)
        moving.sort()
        print("Panel nuevo")
        print("  refresco: mediana %.1f ms, p95 %.1f ms, máximo %.1f ms (presupuesto %d ms, %d muestras)" % (
            statistics.median(moving), moving[int(len(moving) * 0.95)], moving[-1], 33, len(moving)))
        print("  latido del dead-man: mediana %.1f ms, máximo %.1f ms, objetivo 40 ms (%d muestras)" % (
            statistics.median(gaps), max(gaps), len(gaps)))
        print("  ventana congelada con teclas pulsadas: dead-man suelto a los %.0f ms (seguro: 150 ms)" % (stall * 1000))
        print("  ráfaga de %d eventos: %d procesados en %.2f s" % (BURST, seen, t))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capturas", help="carpeta para guardar capturas de la ventana")
    ap.add_argument("--hijo", choices=["old", "new"], help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.hijo:
        child(a.hijo, a.capturas)
        return
    if a.capturas:
        os.makedirs(a.capturas, exist_ok=True)
    sim = subprocess.Popen([sys.executable, os.path.join(ROOT, "ros2", "scara_sim_node.py"), "--velocidad", "5"])
    try:
        time.sleep(2.0)
        out = subprocess.run([sys.executable, __file__, "--hijo", "old"], capture_output=True, text=True, timeout=60).stdout
        for line in out.splitlines():
            if line.startswith("OLD"):
                _, seen, t = line.split()
                print("Panel anterior\n  ráfaga de %d eventos: %s procesados en %s s" % (BURST, seen, t))
        args = [sys.executable, __file__, "--hijo", "new"] + (["--capturas", a.capturas] if a.capturas else [])
        subprocess.run(args, check=True, timeout=180)
    finally:
        sim.terminate()
        sim.wait(timeout=5)


if __name__ == "__main__":
    main()
