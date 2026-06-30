"""
Lanza la cinemática de alto nivel.

Un único controlador publica en /joint_command a la vez (se elige con ``controller``):
  * controller:=trajectory  → control suave con perfil trapezoidal (recomendado HW).
  * controller:=ik          → IK directa "salto a la pose".
fk_check_node siempre se ejecuta (valida TF ↔ FK publicando /fk_pose).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_joint4 = DeclareLaunchArgument(
        "use_joint4", default_value="false",
        description="false ⇒ J4 fijo (4 GDL, pick&place); true ⇒ J4 activo (5 GDL).")
    use_orientation = DeclareLaunchArgument(
        "use_orientation", default_value="false",
        description="Si true, la IK intenta también resolver orientación.")
    controller = DeclareLaunchArgument(
        "controller", default_value="trajectory",
        description="Controlador de alto nivel: 'trajectory' (suave) o 'ik' (directo).")

    uj4 = LaunchConfiguration("use_joint4")
    ctrl = LaunchConfiguration("controller")
    is_traj = IfCondition(PythonExpression(["'", ctrl, "' == 'trajectory'"]))
    is_ik = IfCondition(PythonExpression(["'", ctrl, "' == 'ik'"]))

    fk_check_node = Node(
        package="robotfun_kinematics", executable="fk_check_node",
        name="fk_check_node", output="screen",
    )
    trajectory_node = Node(
        package="robotfun_kinematics", executable="trajectory_node",
        name="trajectory_node", output="screen",
        parameters=[{"use_joint4": uj4}], condition=is_traj,
    )
    ik_node = Node(
        package="robotfun_kinematics", executable="ik_node", name="ik_node",
        output="screen", condition=is_ik,
        parameters=[{
            "use_joint4": uj4,
            "use_orientation": LaunchConfiguration("use_orientation"),
            "damping": 0.05,
            "orient_weight": 0.3,
        }],
    )

    return LaunchDescription([
        use_joint4, use_orientation, controller,
        fk_check_node, trajectory_node, ik_node,
    ])
