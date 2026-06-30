"""
bringup.launch.py — composition root del sistema (alto nivel, sin hardware).

Compone:
  * robotfun_description/display.launch.py  → robot_state_publisher + RViz (+ sliders)
  * robotfun_kinematics/kinematics.launch.py → fk_check + (trajectory | ik)

Argumentos:
  model:=primitives|meshes      modelo a visualizar (medidas reales vs piezas reales).
  controller:=trajectory|ik      controlador de alto nivel que publica /joint_command.
  method:=analytic|dls|newton|gradient   método de IK (analytic recomendado).
  approach_deg:=-90              ángulo de aproximación de la pinza (−90 = abajo).
  gui:=true|false               sliders manuales (útil sin ESP32).

Con hardware real, lanzar además:
  ros2 launch robotfun_firmware microros_agent.launch.py
y NO usar gui:=true (el ESP32 ya publica /joint_states).

Ejemplos:
  ros2 launch robotfun_bringup bringup.launch.py
  ros2 launch robotfun_bringup bringup.launch.py model:=meshes controller:=ik
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = DeclareLaunchArgument("model", default_value="primitives")
    controller = DeclareLaunchArgument("controller", default_value="trajectory")
    method = DeclareLaunchArgument("method", default_value="analytic")
    approach_deg = DeclareLaunchArgument("approach_deg", default_value="-90.0")
    gui = DeclareLaunchArgument("gui", default_value="true")

    desc_launch = PathJoinSubstitution(
        [FindPackageShare("robotfun_description"), "launch", "display.launch.py"])
    kin_launch = PathJoinSubstitution(
        [FindPackageShare("robotfun_kinematics"), "launch", "kinematics.launch.py"])

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(desc_launch),
        launch_arguments={
            "model": LaunchConfiguration("model"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )
    kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kin_launch),
        launch_arguments={
            "controller": LaunchConfiguration("controller"),
            "method": LaunchConfiguration("method"),
            "approach_deg": LaunchConfiguration("approach_deg"),
        }.items(),
    )

    return LaunchDescription([
        model, controller, method, approach_deg, gui,
        description, kinematics,
    ])
