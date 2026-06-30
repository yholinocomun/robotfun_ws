"""Núcleo de cinemática puro (sin ROS): modelo DH, FK, Jacobiano, IK y trayectorias."""

from .dh_model import (
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
    ACTIVE_ALL,
    ACTIVE_J4_FIXED,
    IKResult,
    IKSolver,
    quat_to_rot,
    rot_to_quat,
    rpy_to_rot,
    solve_ik,
)
from .trajectory import joint_trajectory, trapezoidal_profile

__all__ = [
    "ARM_JOINT_NAMES", "GRIPPER_JOINT_NAME", "N_JOINTS", "ROBOT", "DHChain",
    "build_default_robot", "dh", "fkine", "jacobian_geometric", "jacobian_position",
    "ACTIVE_ALL", "ACTIVE_J4_FIXED", "IKResult", "IKSolver", "quat_to_rot",
    "rot_to_quat", "rpy_to_rot", "solve_ik", "joint_trajectory", "trapezoidal_profile",
]
