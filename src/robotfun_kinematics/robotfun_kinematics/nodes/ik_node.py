#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_node.py
==========

Nodo ROS 2 de ALTO NIVEL (cinemática inversa).

    /target_pose      (geometry_msgs/Pose)        — pose cartesiana deseada
    /gripper_command  (std_msgs/Float32)          — apertura del gripper (rad)
    /joint_states     (sensor_msgs/JointState)    — realimentación (semilla IK)
              |
              v   IK (Newton-Raphson + DLS, capa de dominio)
              |
    /joint_command    (std_msgs/Float32MultiArray)— [q1..q5, gripper] → ESP32

Parámetros
----------
use_joint4    : bool (default False). False ⇒ J4 fijo (4 GDL, pick & place);
                True ⇒ J4 activo (5 GDL, tareas de orientación).
use_orientation : bool (default False). Si True, la IK intenta también orientación.
damping       : lambda del DLS.
orient_weight : peso de la tarea de orientación.

Este nodo es un ADAPTADOR delgado: toda la matemática vive en
``robotfun_kinematics.core`` (Clean Architecture).
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray

from robotfun_kinematics.core import (
    ARM_JOINT_NAMES,
    N_JOINTS,
    ROBOT,
    IKSolver,
    quat_to_rot,
)
from robotfun_kinematics.core.ik_solver import ACTIVE_ALL, ACTIVE_J4_FIXED


class IKNode(Node):
    def __init__(self):
        super().__init__("ik_node")

        self.declare_parameter("use_joint4", False)
        self.declare_parameter("use_orientation", False)
        self.declare_parameter("damping", 0.05)
        self.declare_parameter("orient_weight", 0.3)

        self.use_joint4 = bool(self.get_parameter("use_joint4").value)
        self.use_orientation = bool(self.get_parameter("use_orientation").value)
        damping = float(self.get_parameter("damping").value)
        orient_weight = float(self.get_parameter("orient_weight").value)

        self.solver = IKSolver(robot=ROBOT, damping=damping, orient_weight=orient_weight)
        self.active_mask = ACTIVE_ALL if self.use_joint4 else ACTIVE_J4_FIXED

        # Semilla = última realimentación articular conocida (arranca en HOME).
        self.q_current = np.zeros(N_JOINTS)
        self.gripper_cmd = 0.0

        self.cmd_pub = self.create_publisher(Float32MultiArray, "/joint_command", 10)
        self.create_subscription(Pose, "/target_pose", self.target_cb, 10)
        self.create_subscription(Float32, "/gripper_command", self.gripper_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        self.get_logger().info(
            f"ik_node listo | use_joint4={self.use_joint4} "
            f"(J4 {'activo, 5 GDL' if self.use_joint4 else 'fijo, 4 GDL'}) "
            f"| use_orientation={self.use_orientation}. "
            "Publica una Pose en /target_pose.")

    # -- realimentación: actualiza la semilla de la IK ------------------------
    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        for i, jn in enumerate(ARM_JOINT_NAMES):
            if jn in name_to_pos:
                self.q_current[i] = name_to_pos[jn]

    def gripper_cb(self, msg: Float32):
        self.gripper_cmd = float(msg.data)
        self.publish_command(self.q_current)   # reenvía con la nueva apertura

    # -- objetivo cartesiano → IK → comando ----------------------------------
    def target_cb(self, msg: Pose):
        x_des = np.array([msg.position.x, msg.position.y, msg.position.z])
        R_des = None
        if self.use_orientation:
            R_des = quat_to_rot(msg.orientation.x, msg.orientation.y,
                                msg.orientation.z, msg.orientation.w)

        res = self.solver.solve(x_des, self.q_current, R_des=R_des,
                                active_mask=self.active_mask)
        if not res.ok:
            self.get_logger().warn(
                f"IK no convergió (err={res.error:.4f}). Se publica la mejor "
                "solución; posible objetivo fuera del espacio de trabajo o singularidad.")

        self.q_current = res.q
        self.publish_command(res.q)

        x_chk = ROBOT.fkine(res.q)[0:3, 3]
        self.get_logger().info(
            f"objetivo {np.round(x_des, 4)} → q(deg) {np.round(np.degrees(res.q), 1)} "
            f"| FK={np.round(x_chk, 4)} err={res.error:.2e}")

    def publish_command(self, q):
        out = Float32MultiArray()
        out.data = [float(v) for v in q[:N_JOINTS]] + [float(self.gripper_cmd)]
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
