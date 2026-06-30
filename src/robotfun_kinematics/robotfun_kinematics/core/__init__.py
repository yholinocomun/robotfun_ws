"""Núcleo de cinemática puro (sin ROS): modelo DH 4 GDL, FK, Jacobiano, IK y trayectorias."""

from .dh_model import (
    A2, A3, D1, HAND,
    ARM_JOINT_NAMES,
    GRIPPER_JOINT_NAME,
    N_JOINTS,
    ROBOT,
    DHChain,
    build_default_robot,
    dh,
    fkine,
    jacobian_geometric,
    jacobian_position,
)
from .ik_solver import (
    IKResult,
    approach_angle,
    ik_analytic,
    rot_to_quat,
    solve_analytic,
    solve_ik,
    solve_numeric,
)
from .trajectory import joint_trajectory, trapezoidal_profile
from .workspace import WorkspaceLimits, clamp_target, reach, validate_target

__all__ = [
    "A2", "A3", "D1", "HAND", "ARM_JOINT_NAMES", "GRIPPER_JOINT_NAME", "N_JOINTS",
    "ROBOT", "DHChain", "build_default_robot", "dh", "fkine", "jacobian_geometric",
    "jacobian_position", "IKResult", "approach_angle", "ik_analytic", "rot_to_quat",
    "solve_analytic", "solve_ik", "solve_numeric", "joint_trajectory",
    "trapezoidal_profile", "WorkspaceLimits", "clamp_target", "reach", "validate_target",
]
