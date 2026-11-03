#!/usr/bin/env python3
"""
Puente para RViz: convierte /scara/state (micro-ROS) en lo que RViz entiende.

  /scara/state (Float32MultiArray)  ->  /joint_states (JointState)   [robot_state_publisher dibuja el robot]
  /scara/goto  (Point, mm)          ->  marcador de la consigna (esfera verde)
  /scara/cmd   ("goto x y")         ->  el mismo marcador, para las consignas que llegan como texto (secuencia)
  /scara/state (referencia)         ->  marcador de la referencia del controlador (esfera amarilla)
  (estado)                          ->  /scara/markers: espacio de trabajo (corona de alcance), estela del efector
  secuencia.yaml, /scara/event      ->  /scara/markers: placas y huecos; cada hueco cambia de color según su tornillo

Espacio de trabajo: corona entre |l1-l2| y l1+l2 (l1, l2 de KIN en src/config.c). No incluye los
límites articulares reales (finales de carrera); es el alcance geométrico de la cinemática inversa.
Lanzar todo junto:  ros2 launch ros2/scara_rviz.launch.py          (agregar sim:=true para usar el simulador)
"""
import math
import os

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

import scara_sequence as sq

L1_MM, L2_MM = 120.0, 70.0          # mantener igual que KIN en src/config.c
FRAME = "base_link"
BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
TRAIL_MAX = 400
SEQ_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "secuencia.yaml")
HOLE_COLORS = {None: (0.55, 0.57, 0.62, 1.0), "sin_medir": (0.12, 0.75, 0.35, 1.0)}
HOLE_BAD = (0.9, 0.18, 0.13, 1.0)


def circle(r_m, n=96):
    return [Point(x=r_m * math.cos(2 * math.pi * k / n), y=r_m * math.sin(2 * math.pi * k / n), z=0.0)
            for k in range(n + 1)]


def forward_mm(q1_deg, q2_deg):
    a, b = math.radians(q1_deg), math.radians(q1_deg + q2_deg)
    return L1_MM * math.cos(a) + L2_MM * math.cos(b), L1_MM * math.sin(a) + L2_MM * math.sin(b)


class RvizBridge(Node):
    def __init__(self):
        super().__init__("scara_rviz")
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/scara/markers", 10)
        self.create_subscription(Float32MultiArray, "/scara/state", self.on_state, BEST_EFFORT)
        self.create_subscription(Point, "/scara/goto", self.on_goto, RELIABLE)
        self.create_subscription(String, "/scara/cmd", self.on_cmd, RELIABLE)
        self.target = None
        self.ref = None
        self.trail = []
        self.q = (0.0, 0.0)
        self.hole_state = {}
        self.create_subscription(String, "/scara/event", self.on_event, RELIABLE)
        try:
            self.process = sq.load_sequence_full(SEQ_FILE).process
        except sq.SequenceError as e:
            self.process = None
            self.get_logger().warning("No se dibujan las placas: %s" % e)
        self.create_timer(0.1, self.publish_markers)

    def on_state(self, m):
        d = m.data
        if len(d) < 2:
            return
        self.q = (d[0], d[1])
        if len(d) > 10:
            self.ref = forward_mm(d[9], d[10])
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["joint1", "joint2"]
        js.position = [math.radians(d[0]), math.radians(d[1])]
        self.pub_js.publish(js)
        x, y = forward_mm(d[0], d[1])
        p = Point(x=x / 1000.0, y=y / 1000.0, z=0.05)
        if not self.trail or math.hypot(p.x - self.trail[-1].x, p.y - self.trail[-1].y) > 0.002:
            self.trail.append(p)
            self.trail = self.trail[-TRAIL_MAX:]

    def on_goto(self, m):
        self.target = (float(m.x) / 1000.0, float(m.y) / 1000.0)

    def on_cmd(self, m):
        w = m.data.split()
        if len(w) >= 3 and w[0] == "goto":
            try:
                self.target = (float(w[1]) / 1000.0, float(w[2]) / 1000.0)
            except ValueError:
                pass

    def on_event(self, m):
        r = sq.parse_tornillo(m.data)
        if not r or not self.process:
            return
        x, y = forward_mm(*self.q)
        near = min(self.process.holes, key=lambda h: math.hypot(h["x"] - x, h["y"] - y))
        self.hole_state[near["label"]] = r["resultado"]

    def process_markers(self):
        """Dibujo de lo que se va a atornillar, en el marco de la base. Un solo marcador por lista: 16 esferas no cuestan más."""
        board_h = 0.002
        b = self.marker(20, Marker.CUBE_LIST, (0.13, 0.42, 0.28, 0.85), (self.process.side / 1000.0, self.process.side / 1000.0, board_h))
        b.points = [Point(x=c[0] / 1000.0, y=c[1] / 1000.0, z=0.0) for c in self.process.boards.values()]
        h = self.marker(21, Marker.SPHERE_LIST, (1, 1, 1, 1), (0.006, 0.006, 0.006))
        h.points = [Point(x=p["x"] / 1000.0, y=p["y"] / 1000.0, z=board_h) for p in self.process.holes]
        h.colors = [ColorRGBA(**dict(zip("rgba", HOLE_COLORS.get(self.hole_state.get(p["label"]), HOLE_BAD)))) for p in self.process.holes]
        o = self.marker(22, Marker.CYLINDER, (0.25, 0.5, 0.95, 0.9), (0.008, 0.008, 0.002))
        o.pose.position.x, o.pose.position.y = self.process.origin[0] / 1000.0, self.process.origin[1] / 1000.0
        return [b, h, o]

    def marker(self, mid, mtype, color, scale):
        k = Marker()
        k.header.frame_id = FRAME
        k.header.stamp = self.get_clock().now().to_msg()
        k.ns, k.id, k.type, k.action = "scara", mid, mtype, Marker.ADD
        k.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
        k.scale.x, k.scale.y, k.scale.z = (float(v) for v in scale)   # ROS exige float, no int
        k.pose.orientation.w = 1.0
        return k

    def publish_markers(self):
        arr = MarkerArray()
        r_out = (L1_MM + L2_MM) / 1000.0
        r_in = abs(L1_MM - L2_MM) / 1000.0
        for i, r in enumerate((r_out, r_in)):
            k = self.marker(i, Marker.LINE_STRIP, (0.2, 0.8, 0.3, 0.9), (0.002, 0.0, 0.0))
            k.points = circle(r)
            arr.markers.append(k)
        t = self.marker(10, Marker.LINE_STRIP, (1.0, 0.2, 0.6, 0.9), (0.002, 0.0, 0.0))
        t.points = list(self.trail)
        arr.markers.append(t)
        if self.target:
            g = self.marker(11, Marker.SPHERE, (0.1, 0.9, 0.1, 0.9), (0.012, 0.012, 0.012))
            g.pose.position.x, g.pose.position.y, g.pose.position.z = float(self.target[0]), float(self.target[1]), 0.05
            arr.markers.append(g)
        if self.ref:
            r = self.marker(12, Marker.SPHERE, (1.0, 0.8, 0.1, 0.9), (0.008, 0.008, 0.008))
            r.pose.position.x, r.pose.position.y, r.pose.position.z = self.ref[0] / 1000.0, self.ref[1] / 1000.0, 0.05
            arr.markers.append(r)
        if self.process:
            arr.markers.extend(self.process_markers())
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = RvizBridge()
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
