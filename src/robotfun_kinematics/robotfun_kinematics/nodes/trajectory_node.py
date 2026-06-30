#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trajectory_node.py
==================

Control cinemático con PERFIL TRAPEZOIDAL (movimiento suave en lazo abierto).

Dos modos:
  1) ARTICULAR  : /joint_goal (Float32MultiArray [q1..q5, gripper]) → interpolación
                  trapezoidal sincronizada en el espacio articular.
  2) CARTESIANO : /target_pose (Pose) → recta cartesiana con perfil trapezoidal;
                  en cada paso se integra  q += J^+(q)·dx  (control diferencial
                  real con Jacobiano + pseudo-inversa amortiguada).

Salida: /joint_command (Float32MultiArray) — stream fino [q1..q5, gripper] a
``control_rate`` Hz hacia el ESP32.

Respeta la versión J4 fijo/activo vía el parámetro ``use_joint4`` (en cartesiano
congela la columna de J4 igual que la IK).
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from robotfun_kinematics.core import ARM_JOINT_NAMES, N_JOINTS, ROBOT
from robotfun_kinematics.core.ik_solver import ACTIVE_ALL, ACTIVE_J4_FIXED
from robotfun_kinematics.core.trajectory import joint_trajectory, trapezoidal_profile


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("trajectory_node")

        self.declare_parameter("control_rate", 50.0)   # Hz (fineza del stream)
        self.declare_parameter("v_max", 0.6)           # rad/s
        self.declare_parameter("a_max", 1.2)           # rad/s^2
        self.declare_parameter("v_max_cart", 0.08)     # m/s
        self.declare_parameter("a_max_cart", 0.15)     # m/s^2
        self.declare_parameter("damping", 0.06)        # lambda DLS
        self.declare_parameter("use_joint4", False)

        self.rate = float(self.get_parameter("control_rate").value)
        self.use_joint4 = bool(self.get_parameter("use_joint4").value)
        self.active_idx = np.where(ACTIVE_ALL if self.use_joint4 else ACTIVE_J4_FIXED)[0]

        self.q_arm = np.zeros(N_JOINTS)
        self.gripper = 0.0
        self.have_feedback = False
        self.traj: list[np.ndarray] = []
        self.traj_idx = 0

        self.cmd_pub = self.create_publisher(Float32MultiArray, "/joint_command", 10)
        self.create_subscription(Float32MultiArray, "/joint_goal", self.joint_goal_cb, 10)
        self.create_subscription(Pose, "/target_pose", self.pose_goal_cb, 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.stream_cb)

        self.get_logger().info(
            f"trajectory_node listo (trapezoidal) | use_joint4={self.use_joint4}.\n"
            "  articular : ros2 topic pub /joint_goal std_msgs/msg/Float32MultiArray "
            "\"{data: [q1,q2,q3,q4,q5,gripper]}\"\n"
            "  cartesiano: ros2 topic pub /target_pose geometry_msgs/msg/Pose ...")

    def joint_state_cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        if not self.have_feedback:
            for i, jn in enumerate(ARM_JOINT_NAMES):
                if jn in name_to_pos:
                    self.q_arm[i] = name_to_pos[jn]
            if "gripper" in name_to_pos:
                self.gripper = name_to_pos["gripper"]
            self.have_feedback = True

    # MODO ARTICULAR ----------------------------------------------------------
    def joint_goal_cb(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < N_JOINTS:
            self.get_logger().warn("Se requieren al menos 5 ángulos.")
            return
        q_goal = ROBOT.clamp(np.array(data[:N_JOINTS]))
        grip_goal = float(data[N_JOINTS]) if len(data) > N_JOINTS else self.gripper

        v_max = float(self.get_parameter("v_max").value)
        a_max = float(self.get_parameter("a_max").value)

        q0 = np.hstack((self.q_arm, self.gripper))
        qg = np.hstack((q_goal, grip_goal))
        self.traj = joint_trajectory(q0, qg, v_max, a_max, self.rate)
        self.traj_idx = 0
        self.get_logger().info(
            f"[ARTICULAR] {len(self.traj)} pasos, "
            f"duración={len(self.traj)/self.rate:.2f} s")

    # MODO CARTESIANO (control diferencial) -----------------------------------
    def pose_goal_cb(self, msg: Pose):
        x_goal = np.array([msg.position.x, msg.position.y, msg.position.z])
        v_max = float(self.get_parameter("v_max_cart").value)
        a_max = float(self.get_parameter("a_max_cart").value)
        lam2 = float(self.get_parameter("damping").value) ** 2

        q = self.q_arm.copy()
        x0 = ROBOT.fkine(q)[0:3, 3]
        dist = float(np.linalg.norm(x_goal - x0))
        s_list = trapezoidal_profile(dist, v_max, a_max, self.rate)

        traj = []
        s_prev = 0.0
        for s in s_list:
            dx = (x0 + s * (x_goal - x0)) - (x0 + s_prev * (x_goal - x0))
            J = ROBOT.jacobian_position(q)[:, self.active_idx]   # columnas activas
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(3), dx)
            q[self.active_idx] += dq
            q = ROBOT.clamp(q)
            traj.append(np.hstack((q, self.gripper)))
            s_prev = s

        self.traj = traj
        self.traj_idx = 0
        self.get_logger().info(
            f"[CARTESIANO] {len(traj)} pasos, recta={dist*100:.1f} cm, "
            f"duración={len(traj)/self.rate:.2f} s")

    # Stream fino: un setpoint por tick ---------------------------------------
    def stream_cb(self):
        if self.traj_idx >= len(self.traj):
            return
        point = self.traj[self.traj_idx]
        self.traj_idx += 1
        self.q_arm = point[:N_JOINTS].copy()
        self.gripper = float(point[N_JOINTS])
        out = Float32MultiArray()
        out.data = [float(v) for v in point]
        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
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
