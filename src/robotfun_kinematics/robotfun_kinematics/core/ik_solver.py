#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_solver.py
============

Cinemática **inversa** numérica (capa de dominio, sin ROS) para la cadena DH del
robot. Usa Newton-Raphson con pseudo-inversa amortiguada (Damped Least Squares /
Levenberg-Marquardt):

    dq = J^T (J J^T + lambda^2 I)^-1 · e

El amortiguamiento evita pasos enormes cerca de singularidades (det(J J^T) → 0).

Máscara de juntas activas — las DOS versiones del robot
-------------------------------------------------------
El método ``solve`` admite ``active_mask`` (booleanos por junta). Las juntas
inactivas se **congelan** en su valor semilla y se eliminan sus columnas del
Jacobiano antes del DLS. Esto implementa, con UN solo solucionador, las dos
versiones pedidas:

    * J4 FIJO (por defecto, ``ACTIVE_J4_FIXED = [T,T,T,F,T]``): pick & place.
      J4 es un roll del antebrazo: su columna de posición es 0 en HOME y pequeña
      cerca de él (mal-condiciona el DLS), y es la DOF menos útil para posicionar
      (J1+J2+J3 ya cubren la posición y J5 el cabeceo). Congelarlo da una IK de
      4 GDL bien condicionada y sin deriva de q4. Para pastillas axisimétricas el
      roll de la pinza es irrelevante.

    * J4 ACTIVO (``ACTIVE_ALL = [T,T,T,T,T]``): 5 GDL, para tareas donde la
      cámara fije la orientación/roll del objeto (futuro).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_model import ROBOT, DHChain

cos = np.cos
sin = np.sin

#: Máscaras predefinidas para las dos versiones del robot.
ACTIVE_J4_FIXED = np.array([True, True, True, False, True])
ACTIVE_ALL = np.array([True, True, True, True, True])


# ---------------------------------------------------------------------------
# Utilidades de orientación (independientes de la IK; reutilizables por nodos)
# ---------------------------------------------------------------------------
def rot_error(R_cur: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    """Error de orientación axis-angle (3,) que lleva R_cur → R_des."""
    R_err = R_des @ R_cur.T
    cos_ang = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(cos_ang)
    if ang < 1e-9:
        return np.zeros(3)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) / (2.0 * sin(ang))
    return axis * ang


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Roll-Pitch-Yaw (convención URDF: Rz·Ry·Rx) → matriz de rotación."""
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Cuaternión (x,y,z,w) → matriz de rotación 3x3 (normaliza primero)."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-9:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_to_quat(R: np.ndarray):
    """Matriz de rotación 3x3 → cuaternión (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


# ---------------------------------------------------------------------------
# Resultado de la IK
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    q: np.ndarray          # solución articular (rad), n_joints
    ok: bool               # True si convergió por debajo de la tolerancia
    error: float           # norma del error final
    iterations: int        # iteraciones consumidas


# ---------------------------------------------------------------------------
# Solucionador IK
# ---------------------------------------------------------------------------
class IKSolver:
    """
    Solucionador IK (DLS) configurable e inyectable con cualquier ``DHChain``
    (Dependency Inversion: depende de la abstracción, no de variables globales).
    """

    def __init__(self, robot: DHChain = ROBOT, damping: float = 0.05,
                 max_iter: int = 500, tol: float = 1e-4, step_clip: float = 0.3,
                 orient_weight: float = 0.3):
        self.robot = robot
        self.damping = damping
        self.max_iter = max_iter
        self.tol = tol
        self.step_clip = step_clip
        self.orient_weight = orient_weight

    def solve(self, x_des, q0, *, R_des=None, active_mask=ACTIVE_J4_FIXED,
              respect_limits: bool = True) -> IKResult:
        """
        Parameters
        ----------
        x_des : (3,) posición deseada [X, Y, Z] (m).
        q0    : (n,) semilla articular (rad). Conviene una postura "ready" NO
                singular (el home vertical es singularidad de frontera).
        R_des : (3,3) orientación deseada o None (solo posición; recomendado 5 GDL).
        active_mask : (n,) bool. Juntas False se congelan en su valor de q0.
        respect_limits : satura q a [q_min, q_max] cada iteración.
        """
        robot = self.robot
        n = robot.n_joints
        x_des = np.asarray(x_des, dtype=float).ravel()
        q = np.asarray(q0, dtype=float).ravel().copy()[:n]
        active = np.asarray(active_mask, dtype=bool)
        idx = np.where(active)[0]
        lam2 = self.damping * self.damping
        use_orient = R_des is not None

        err = np.inf
        for it in range(1, self.max_iter + 1):
            T = robot.fkine(q)
            e_pos = x_des - T[0:3, 3]

            if use_orient:
                e_ori = self.orient_weight * rot_error(T[0:3, 0:3], R_des)
                e = np.hstack((e_pos, e_ori))
                J_full = robot.jacobian_geometric(q)
                J_full = np.vstack((J_full[0:3, :], self.orient_weight * J_full[3:6, :]))
            else:
                e = e_pos
                J_full = robot.jacobian_position(q)

            err = float(np.linalg.norm(e))
            if err < self.tol:
                return IKResult(q=q, ok=True, error=err, iterations=it)

            # Solo columnas de juntas ACTIVAS → DLS bien condicionado.
            J = J_full[:, idx]
            m = J.shape[0]
            dq_active = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(m), e)

            # Limitar el tamaño del paso (estabilidad / trayectoria suave).
            norm_dq = np.linalg.norm(dq_active)
            if norm_dq > self.step_clip:
                dq_active *= self.step_clip / norm_dq

            q[idx] += dq_active
            if respect_limits:
                q = robot.clamp(q)

        return IKResult(q=q, ok=False, error=err, iterations=self.max_iter)


def solve_ik(x_des, q0, *, robot: DHChain = ROBOT, use_joint4: bool = False,
             R_des=None, damping: float = 0.05, orient_weight: float = 0.3) -> IKResult:
    """Atajo funcional. ``use_joint4=False`` ⇒ J4 fijo (pick & place)."""
    solver = IKSolver(robot=robot, damping=damping, orient_weight=orient_weight)
    mask = ACTIVE_ALL if use_joint4 else ACTIVE_J4_FIXED
    return solver.solve(x_des, q0, R_des=R_des, active_mask=mask)


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    rng = np.random.default_rng(0)
    q_seed = np.array([0.0, -0.5, 0.8, 0.0, 0.3])   # "ready" no singular
    for label, uj4 in [("J4 FIJO (4 GDL)", False), ("J4 ACTIVO (5 GDL)", True)]:
        print(f"\n=== {label} ===")
        for k in range(5):
            q_true = rng.uniform(-1.0, 1.0, 5)
            if not uj4:
                q_true[3] = 0.0
            x_goal = ROBOT.fkine(q_true)[0:3, 3]
            res = solve_ik(x_goal, q_seed, use_joint4=uj4)
            x_reached = ROBOT.fkine(res.q)[0:3, 3]
            print(f"  caso {k}: ok={res.ok} it={res.iterations} "
                  f"err={res.error:.2e} |x_des-x_alc|={np.linalg.norm(x_goal - x_reached):.2e}")
