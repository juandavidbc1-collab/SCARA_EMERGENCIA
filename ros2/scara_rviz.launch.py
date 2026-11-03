"""ros2 launch ros2/scara_rviz.launch.py [sim:=true] [rviz:=false]
Con el robot: necesita ROS 2 cargado y el agente micro-ROS corriendo.  Con sim:=true arranca el simulador en su lugar."""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    with open(os.path.join(HERE, "scara.urdf")) as f:
        urdf = f.read()
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", description="usar el simulador en lugar del robot"),
        DeclareLaunchArgument("rviz", default_value="true", description="abrir la ventana de RViz"),
        ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "scara_sim_node.py")], output="screen",
                       condition=IfCondition(LaunchConfiguration("sim"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}]),
        ExecuteProcess(cmd=[sys.executable, os.path.join(HERE, "scara_rviz.py")], output="screen"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(HERE, "scara.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
