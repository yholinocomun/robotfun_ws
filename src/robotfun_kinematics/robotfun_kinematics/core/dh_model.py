#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dh_model.py
===========

Modelo cinemático del robot **4 GDL (yaw + 3 pitch) + gripper** en convención
**Denavit-Hartenberg estándar** (Spong / Craig "distal"). Capa de dominio
(Clean Architecture): NO depende de ROS y se ejecuta/testea aislada
(`python3 -m robotfun_kinematics.core.dh_model`).

Cambio de diseño respecto al modelo previo (5 GDL)
--------------------------------------------------
Se **eliminó por completo el roll de muñeca** (antiguo joint_4). El antiguo
joint_5 (pitch de muñeca) pasa a ser el nuevo **joint_4**. El robot queda como
un brazo antropomórfico clásico: **1 yaw + 3 pitch coplanares**. Esta estructura
tiene **cinemática inversa ANALÍTICA cerrada** (ver `ik_solver`), lo más eficiente
y exacto posible. El servo del roll se mantiene físicamente fijo a 90° (muerto).

Medidas reales (tabla DH del usuario; metros). Brazo RECTO con un pequeño offset
radial L0 en la base:
    L0   = 0.010     offset radial de la base (a_1)
    L1   = 0.063     base -> eje de pitch del hombro (d_1)
    L2   = 0.120     hombro -> codo (brazo)
    A3   = 0.120     codo -> muñeca (antebrazo, = L3+L4 unificados)
    HAND = 0.110     muñeca -> punta del gripper (L5)

Tabla DH estándar (4 juntas). El vector q se SUMA al offset: θ_i = q_i + θoff_i.

    i | d_i | θ_i      | α_i  | a_i
    --|-----|----------|------|------
    1 | L1  | q1       | +90° | L0      (yaw; α=90 lleva el eje de pitch a horizontal)
    2 | 0   | q2 + 90° |  0°  | L2      (hombro)
    3 | 0   | q3       |  0°  | A3      (codo)
    4 | 0   | q4       |  0°  | HAND    (muñeca)

HOME (q=0): brazo RECTO y vertical, servos a 90°. FK(HOME) → TCP = [0.010, 0, 0.413].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

cos = np.cos
sin = np.sin
pi = np.pi

# ---------------------------------------------------------------------------
# Medidas físicas reales (metros) — tabla DH del usuario. Ajustar si re-mides.
# ---------------------------------------------------------------------------
L0 = 0.010              # offset radial de la base (a_1)
D1 = 0.063              # base -> eje de pitch del hombro (d_1 = L1)
A2 = 0.120              # brazo  (hombro -> codo, = L2)
A3 = 0.120              # antebrazo (codo -> muñeca, = L3+L4 unificados)
HAND = 0.110            # muñeca -> punta del gripper (= L5)

# Offsets θ para que q=0 == HOME (brazo RECTO y vertical).
TH1_OFF = 0.0
TH2_OFF = pi / 2.0
TH3_OFF = 0.0
TH4_OFF = 0.0

#: Nombres canónicos (deben coincidir con el URDF y el firmware).
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4"]
GRIPPER_JOINT_NAME = "gripper"


def dh(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
    """Matriz homogénea DH estándar: ``Rz(theta)·Tz(d)·Tx(a)·Rx(alpha)``."""
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


@dataclass(frozen=True)
class DHChain:
    """Cadena cinemática serie parametrizada por su tabla DH constante."""

    d: np.ndarray
    a: np.ndarray
    alpha: np.ndarray
    theta_offset: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray

    @property
    def n_joints(self) -> int:
        return len(self.d)

    def fkine(self, q, upto: int | None = None, return_frames: bool = False):
        """Cinemática directa. q en radianes; ignora elementos extra (gripper)."""
        q = np.asarray(q, dtype=float).ravel()
        n = self.n_joints if upto is None else upto
        T = np.eye(4)
        frames = []
        for i in range(n):
            T = T @ dh(self.d[i], q[i] + self.theta_offset[i], self.a[i], self.alpha[i])
            frames.append(T.copy())
        return (T, frames) if return_frames else T

    def jacobian_geometric(self, q) -> np.ndarray:
        """Jacobiano geométrico 6 x n (todas las juntas revolutas)."""
        _, frames = self.fkine(q, return_frames=True)
        p_e = frames[-1][0:3, 3]
        J = np.zeros((6, self.n_joints))
        z_prev = np.array([0.0, 0.0, 1.0])
        p_prev = np.array([0.0, 0.0, 0.0])
        for i in range(self.n_joints):
            J[0:3, i] = np.cross(z_prev, p_e - p_prev)
            J[3:6, i] = z_prev
            z_prev = frames[i][0:3, 2]
            p_prev = frames[i][0:3, 3]
        return J

    def jacobian_position(self, q) -> np.ndarray:
        return self.jacobian_geometric(q)[0:3, :]

    def clamp(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.q_min, self.q_max)

    # -- geometría para la IK analítica (sólo válida para esta estructura) ----
    @property
    def base_offset(self) -> float:
        return float(self.a[0])      # L0 (offset radial del hombro respecto al eje de yaw)

    @property
    def shoulder_height(self) -> float:
        return float(self.d[0])      # D1 (= L1)

    @property
    def link_upper(self) -> float:
        return float(self.a[1])      # A2

    @property
    def link_fore(self) -> float:
        return float(self.a[2])      # A3

    @property
    def link_hand(self) -> float:
        return float(self.a[3])      # HAND


# Límites articulares POR JUNTA (rad). Deben coincidir con JOINT_MIN/MAX_DEG y
# SERVO_CENTER_DEG del firmware. Por defecto ±90° (servo centrado). Para MÁS
# espacio de trabajo se pueden hacer ASIMÉTRICOS (sesgar los pitch hacia adelante),
# p. ej. Q_MIN=[-90,-60,-60,-90], Q_MAX=[90,120,120,90] (grados) tras re-montar
# los horns de los servos y ajustar SERVO_CENTER_DEG en el firmware.
Q_MIN_DEG = np.array([-90.0, -90.0, -90.0, -90.0])
Q_MAX_DEG = np.array([90.0, 90.0, 90.0, 90.0])


def build_default_robot() -> DHChain:
    """Construye la cadena DH validada del robot (4 GDL)."""
    return DHChain(
        d=np.array([D1, 0.0, 0.0, 0.0]),
        a=np.array([L0, A2, A3, HAND]),
        alpha=np.array([pi / 2.0, 0.0, 0.0, 0.0]),
        theta_offset=np.array([TH1_OFF, TH2_OFF, TH3_OFF, TH4_OFF]),
        q_min=np.radians(Q_MIN_DEG),
        q_max=np.radians(Q_MAX_DEG),
    )


ROBOT: DHChain = build_default_robot()
N_JOINTS: int = ROBOT.n_joints


def fkine(q, **kw):
    return ROBOT.fkine(q, **kw)


def jacobian_geometric(q):
    return ROBOT.jacobian_geometric(q)


def jacobian_position(q):
    return ROBOT.jacobian_position(q)


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    _, fr = ROBOT.fkine(np.zeros(N_JOINTS), return_frames=True)
    print("FK(HOME) — brazo recto y vertical (tabla DH del usuario):")
    print("  hombro :", np.round(fr[0][:3, 3], 4), " esperado (0.010,0,0.063)")
    print("  codo   :", np.round(fr[1][:3, 3], 4), " esperado (0.010,0,0.183)")
    print("  muñeca :", np.round(fr[2][:3, 3], 4), " esperado (0.010,0,0.303)")
    print("  TCP    :", np.round(fr[3][:3, 3], 4), " esperado (0.010,0,0.413)")
    print(f"alcance máx desde el hombro = A2+A3+HAND = {A2 + A3 + HAND:.4f} m")
    print("θ offsets (deg):", np.round(np.degrees(ROBOT.theta_offset), 2))
