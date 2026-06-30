"""
bringup.launch.py — composition root del sistema (alto nivel, sin hardware).

Compone:
  * robotfun_description/display.launch.py  → robot_state_publisher + RViz (+ sliders)
  * robotfun_kinematics/kinematics.launch.py → fk_check + (trajectory | ik)

Argumentos:
  model:=primitives|meshes     modelo a visualizar (comparar real vs DH).
  use_joint4:=false|true       false ⇒ J4 fijo (4 GDL, pick&place); true ⇒ 5 GDL.
  controller:=trajectory|ik     controlador de alto nivel que publica /joint_command.
  gui:=true|false              sliders manuales (útil sin ESP32).

Con hardware real, lanzar además:
  ros2 launch robotfun_firmware microros_agent.launch.py
y NO usar gui:=true (el ESP32 ya publica /joint_states).

Ejemplos:
  ros2 launch robotfun_bringup bringup.launch.py
  ros2 launch robotfun_bringup bringup.launch.py model:=meshes use_joint4:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = DeclareLaunchArgument("model", default_value="primitives")
    use_joint4 = DeclareLaunchArgument("use_joint4", default_value="false")
    controller = DeclareLaunchArgument("controller", default_value="trajectory")
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
            "use_joint4": LaunchConfiguration("use_joint4"),
            "controller": LaunchConfiguration("controller"),
        }.items(),
    )

    return LaunchDescription([
        model, use_joint4, controller, gui,
        description, kinematics,
    ])
