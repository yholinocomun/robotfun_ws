#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dh_model.py
===========

Modelo cinemático del robot de 5 GDL (RRRRR) + gripper, en la convención
**Denavit-Hartenberg estándar** (Spong / Craig "distal"). Es la **capa de
dominio** (Clean Architecture): NO depende de ROS y se puede ejecutar/testear
de forma aislada (`python3 -m robotfun_kinematics.core.dh_model`).

Origen del modelo
-----------------
La tabla DH proviene del trabajo de laboratorio (`frlabsyholi`: dh / fkine /
jacobian) y fue **validada contra el robot físico** en `twin_ws`:
    - HOME vertical (servos a 90° ⇒ q = 0)  →  FK(home) ≈ [0.010, 0, 0.393] m.
    - El comportamiento de cada junta coincide con las fotos del robot.

Estructura física (yaw-pitch-pitch-roll-pitch):
    J1 yaw   (base)
    J2 pitch (hombro)
    J3 pitch (codo, eje || a J2)
    J4 ROLL  (muñeca: eje saliente Z COLINEAL con el antebrazo → longitud en d4)
    J5 PITCH (pinza: revolute que inclina la pinza; la pinza apunta por X5)

Tabla DH estándar (longitudes en metros):
    L0=0.010  L1=0.063  L2=0.120  L3=0.090  L4=0.030  L5=0.090

    i | d_i      | theta_i   | alpha_i | a_i
    --|----------|-----------|---------|-----
    1 | L1       | q1 + 0°   |  +90°   | L0
    2 | 0        | q2 + 90°  |   0°    | L2
    3 | 0        | q3 + 90°  |  +90°   | 0
    4 | L3 + L4  | q4 + 0°   |  +90°   | 0
    5 | 0        | q5 + 90°  |   0°    | L5

Nota J4 (eje saliente / la DOF menos útil para posicionar)
----------------------------------------------------------
El eje Z4 es COLINEAL con el antebrazo, por eso su longitud va en d4 (a lo largo
del eje), no en a4 (perpendicular). Girar J4 es un ROLL del antebrazo.

  * En HOME (y siempre que la punta caiga SOBRE el eje Z4, p. ej. J5≈0) girar J4
    NO mueve la punta → su columna del Jacobiano de POSICIÓN es exactamente 0.
  * Con la muñeca flexionada (J5≠0) la punta queda fuera del eje y J4 SÍ la
    desplaza algo (la barre alrededor del antebrazo).

Aun así, J1 (yaw) + J2,J3 (brazo planar) ya cubren la posición 3D y J5 da el
cabeceo de aproximación: J4 es la DOF **menos útil para posicionar** y, cerca de
HOME, mal-condiciona el Jacobiano. Por eso el solucionador la **congela por
defecto** (ver `ik_solver`, máscara ``ACTIVE_J4_FIXED``); el roll solo importa
para orientar la pinza (futuro, guiado por cámara).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

cos = np.cos
sin = np.sin
pi = np.pi

# ---------------------------------------------------------------------------
# Dimensiones físicas del robot (metros). Ajustar aquí si se re-mide el robot.
# ---------------------------------------------------------------------------
L0 = 0.010
L1 = 0.063
L2 = 0.120
L3 = 0.090
L4 = 0.030
L5 = 0.090

#: Nombres canónicos de las juntas del brazo (deben coincidir EXACTAMENTE con el
#: URDF y con el firmware: joint_1..joint_5; el gripper es un canal aparte).
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
GRIPPER_JOINT_NAME = "gripper"


def dh(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
    """Matriz homogénea DH estándar: ``Rot_z(theta)·Trans_z(d)·Trans_x(a)·Rot_x(alpha)``."""
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
    """
    Cadena cinemática serie parametrizada por su tabla DH constante.

    Encapsular la tabla en un objeto (en vez de variables globales) cumple el
    Principio de Responsabilidad Única y permite, sin tocar el resto del código,
    crear variantes del robot (p. ej. recalibrar L2/L3) o reusar la clase para
    otra cadena. El vector articular ``q`` se SUMA al offset:
    ``theta_i = q_i + theta_offset_i``.
    """

    d: np.ndarray
    a: np.ndarray
    alpha: np.ndarray
    theta_offset: np.ndarray
    q_min: np.ndarray
    q_max: np.ndarray

    @property
    def n_joints(self) -> int:
        return len(self.d)

    # ------------------------------------------------------------------ FK ---
    def fkine(self, q, upto: int | None = None, return_frames: bool = False):
        """
        Cinemática directa.

        q : vector (>= n_joints) en radianes. Si trae el gripper como elemento
            extra, se ignora para la cadena del brazo.
        upto : si se indica, devuelve la pose del frame ``upto`` (1..n_joints).
        return_frames : si True devuelve también [T_0^1, ..., T_0^n].
        """
        q = np.asarray(q, dtype=float).ravel()
        n = self.n_joints if upto is None else upto
        T = np.eye(4)
        frames = []
        for i in range(n):
            theta_i = q[i] + self.theta_offset[i]
            T = T @ dh(self.d[i], theta_i, self.a[i], self.alpha[i])
            frames.append(T.copy())
        if return_frames:
            return T, frames
        return T

    # ----------------------------------------------------------- Jacobiano ---
    def jacobian_geometric(self, q) -> np.ndarray:
        """
        Jacobiano geométrico 6 x n (todas las juntas revolutas):
            J_v_i = z_{i-1} x (p_e - p_{i-1})     (parte lineal)
            J_w_i = z_{i-1}                       (parte angular)
        """
        _, frames = self.fkine(q, return_frames=True)
        p_e = frames[-1][0:3, 3]

        J = np.zeros((6, self.n_joints))
        z_prev = np.array([0.0, 0.0, 1.0])   # frame 0 = base
        p_prev = np.array([0.0, 0.0, 0.0])
        for i in range(self.n_joints):
            J[0:3, i] = np.cross(z_prev, p_e - p_prev)
            J[3:6, i] = z_prev
            z_prev = frames[i][0:3, 2]
            p_prev = frames[i][0:3, 3]
        return J

    def jacobian_position(self, q) -> np.ndarray:
        """Filas lineales del Jacobiano geométrico (3 x n)."""
        return self.jacobian_geometric(q)[0:3, :]

    def clamp(self, q) -> np.ndarray:
        """Satura q a [q_min, q_max]."""
        return np.clip(np.asarray(q, dtype=float), self.q_min, self.q_max)


def build_default_robot() -> DHChain:
    """Construye la cadena DH validada del robot (5 GDL)."""
    n = 5
    return DHChain(
        d=np.array([L1, 0.0, 0.0, L3 + L4, 0.0]),
        a=np.array([L0, L2, 0.0, 0.0, L5]),
        alpha=np.array([pi / 2.0, 0.0, pi / 2.0, pi / 2.0, 0.0]),
        theta_offset=np.array([0.0, pi / 2.0, pi / 2.0, 0.0, pi / 2.0]),
        q_min=np.array([-pi / 2.0] * n),   # el firmware satura a ±90°
        q_max=np.array([pi / 2.0] * n),
    )


#: Instancia por defecto reutilizada por los nodos ROS y los tests.
ROBOT: DHChain = build_default_robot()
N_JOINTS: int = ROBOT.n_joints


# Funciones de conveniencia (API estable a nivel de módulo) -------------------
def fkine(q, **kw):
    return ROBOT.fkine(q, **kw)


def jacobian_geometric(q):
    return ROBOT.jacobian_geometric(q)


def jacobian_position(q):
    return ROBOT.jacobian_position(q)


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    print("Tabla DH del robot (5 GDL). FK(HOME=q0) — esperado ~[0.010, 0, 0.393]:")
    T_home = ROBOT.fkine(np.zeros(N_JOINTS))
    print(T_home)
    print("posición efector final:", np.round(T_home[0:3, 3], 4))
    print("\nColumna J4 del Jacobiano de posición en HOME (debe ser ~0):")
    print(np.round(ROBOT.jacobian_position(np.zeros(N_JOINTS))[:, 3], 6))
    q_bent = np.array([0.0, -0.5, 0.8, 0.0, 0.6])   # muñeca flexionada (J5≠0)
    print("Columna J4 con la muñeca flexionada (J5≠0, ya NO es 0):")
    print(np.round(ROBOT.jacobian_position(q_bent)[:, 3], 6))
