#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_solver.py
============

Cinemática **inversa** del robot de 4 GDL (yaw + 3 pitch) + gripper. Capa de
dominio (sin ROS).

La tarea de un brazo de 4 GDL es **posición (3) + ángulo de aproximación (1)** =
4 coordenadas con 4 juntas → sistema cuadrado. Por eso ofrecemos varias vías,
de la más eficiente a la más general:

  1) ANALÍTICA cerrada  (``method="analytic"``, RECOMENDADA)
     yaw directo + 2R planar por ley de cosenos. Exacta, instantánea, da las dos
     ramas (codo arriba/abajo). Es lo más eficiente para esta estructura.

  2) DLS / Levenberg-Marquardt (``method="dls"``, la "mejor" numérica)
     dq = Jᵀ(JJᵀ + λ²I)⁻¹ e. Amortiguada → estable cerca de singularidades.
     Útil si pides sólo posición (deja libre el ángulo) o como respaldo robusto.

  3) Newton / Gauss-Newton (``method="newton"``)
     dq = J⁺ e (pseudo-inversa). Convergencia cuadrática pero frágil en
     singularidades (sin amortiguamiento).

  4) Gradiente / Jacobiano transpuesto (``method="gradient"``)
     dq = α Jᵀ e. El más simple y barato por iteración; converge lento. Didáctico.

Convención del ángulo de aproximación φ
---------------------------------------
φ = ángulo absoluto de la mano en el plano de trabajo (plano vertical que
contiene al brazo tras el yaw), medido desde el eje radial +r:
    φ = +90° → la pinza apunta ARRIBA   (HOME)
    φ =   0° → apunta horizontal
    φ = −90° → apunta ABAJO  (pick & place top-down)
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_model import (
    TH2_OFF, TH3_OFF, TH4_OFF, ROBOT, DHChain,
)

cos = np.cos
sin = np.sin
pi = np.pi


# ---------------------------------------------------------------------------
# Resultado de la IK
# ---------------------------------------------------------------------------
@dataclass
class IKResult:
    q: np.ndarray          # solución articular (rad), n_joints
    ok: bool               # True si es válida (alcanzable y dentro de límites)
    error: float           # norma del error cartesiano final
    method: str            # método usado
    iterations: int = 0    # iteraciones (0 para analítica)
    reason: str = ""       # motivo si ok=False


def approach_angle(q) -> float:
    """Ángulo absoluto φ de la mano en el plano (rad) para una postura q."""
    q = np.asarray(q, dtype=float).ravel()
    return (q[1] + TH2_OFF) + (q[2] + TH3_OFF) + (q[3] + TH4_OFF)


# ---------------------------------------------------------------------------
# 1) IK ANALÍTICA CERRADA
# ---------------------------------------------------------------------------
def _wrap(a: float) -> float:
    return (a + pi) % (2.0 * pi) - pi


def _planar_2r(rw: float, zw: float, sr: float, d1: float, a2: float, a3: float,
               elbow_up: bool):
    """2R planar: hombro (sr,d1) → muñeca (rw,zw). Devuelve (ang2_abs, ang3_abs) o None."""
    dr, dz = rw - sr, zw - d1
    c3 = (dr * dr + dz * dz - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
    if c3 < -1.0 or c3 > 1.0:
        return None
    s3 = np.sqrt(max(0.0, 1.0 - c3 * c3))
    if elbow_up:
        s3 = -s3
    ang_elbow = np.arctan2(s3, c3)
    ang2 = np.arctan2(dz, dr) - np.arctan2(a3 * sin(ang_elbow), a2 + a3 * cos(ang_elbow))
    return ang2, ang2 + ang_elbow


def ik_analytic(x: float, y: float, z: float, phi: float,
                robot: DHChain = ROBOT, elbow_up: bool = False):
    """
    IK cerrada (rama frontal): TCP (x,y,z) + ángulo de aproximación φ (rad).

    φ se mide en el plano vertical que va de la base hacia (x,y): +90°=arriba,
    0°=horizontal hacia afuera, −90°=abajo (pick top-down). q1=atan2(y,x) (el
    robot opera al frente, q1∈±90°). ``elbow_up`` elige la rama del codo.
    Devuelve q=[q1..q4] o None si esa rama no alcanza.
    """
    d1, a2, a3, hand = (robot.shoulder_height, robot.link_upper,
                        robot.link_fore, robot.link_hand)
    sr = robot.base_offset                       # offset radial del hombro (L0)
    q1 = np.arctan2(y, x)
    r = np.hypot(x, y)
    rw = r - hand * cos(phi)                     # muñeca en el plano (r, z)
    zw = z - hand * sin(phi)
    planar = _planar_2r(rw, zw, sr, d1, a2, a3, elbow_up)
    if planar is None:
        return None
    ang2, ang3 = planar
    q2 = ang2 - TH2_OFF
    q3 = _wrap(ang3 - ang2 - TH3_OFF)
    q4 = _wrap(phi - ang3 - TH4_OFF)
    return np.array([q1, q2, q3, q4])


def solve_analytic(x, y, z, phi, robot: DHChain = ROBOT, seed=None) -> IKResult:
    """
    IK analítica: prueba las dos ramas del codo, filtra por límites articulares
    y elige la válida más cercana a ``seed`` (continuidad de movimiento).
    """
    target = np.array([x, y, z])
    candidates = []   # (q, in_limits, err)
    for up in (False, True):
        q = ik_analytic(x, y, z, phi, robot, elbow_up=up)
        if q is None:
            continue
        in_lim = bool(np.all(q >= robot.q_min - 1e-9) and np.all(q <= robot.q_max + 1e-9))
        err = float(np.linalg.norm(robot.fkine(q)[0:3, 3] - target))
        candidates.append((q, in_lim, err))

    valid = [c for c in candidates if c[1] and c[2] < 1e-4]
    if valid:
        if seed is not None:
            s = np.asarray(seed, dtype=float).ravel()[:robot.n_joints]
            q = min(valid, key=lambda c: np.linalg.norm(c[0] - s))[0]
        else:
            q = valid[0][0]
        return IKResult(q=q, ok=True, method="analytic",
                        error=float(np.linalg.norm(robot.fkine(q)[0:3, 3] - target)))

    if not candidates:
        return IKResult(q=np.zeros(robot.n_joints), ok=False, error=float("inf"),
                        method="analytic", reason="objetivo fuera del espacio de trabajo")
    q = robot.clamp(min(candidates, key=lambda c: c[2])[0])
    return IKResult(q=q, ok=False, method="analytic",
                    error=float(np.linalg.norm(robot.fkine(q)[0:3, 3] - target)),
                    reason="solución fuera de límites articulares (¿q1>90°? objetivo detrás)")


# ---------------------------------------------------------------------------
# 2-4) IK NUMÉRICA (gradiente / Newton / DLS) sobre [posición(3); φ(1)]
# ---------------------------------------------------------------------------
def _task_error_and_jacobian(robot, q, x_des, phi_des):
    T = robot.fkine(q)
    e_pos = x_des - T[0:3, 3]
    Jp = robot.jacobian_position(q)                 # 3 x n
    if phi_des is None:                             # sólo posición (redundante)
        return e_pos, Jp
    e_phi = np.array([_wrap(phi_des - approach_angle(q))])   # error angular envuelto
    # gradiente de φ respecto a q: [0,1,1,1,...] (todas las pitch suman)
    Jphi = np.zeros((1, robot.n_joints)); Jphi[0, 1:] = 1.0
    return np.hstack((e_pos, e_phi)), np.vstack((Jp, Jphi))


def solve_numeric(x_des, q0, phi_des=None, robot: DHChain = ROBOT, method="dls",
                  damping=0.05, alpha=0.3, max_iter=200, tol=1e-5,
                  step_clip=0.4, respect_limits=True) -> IKResult:
    """
    IK numérica iterativa. ``method`` ∈ {"dls", "newton", "gradient"}.
    Si ``phi_des`` es None, resuelve sólo posición (la redundancia la fija el método).
    """
    x_des = np.asarray(x_des, dtype=float).ravel()
    q = np.asarray(q0, dtype=float).ravel().copy()[:robot.n_joints]
    lam2 = damping * damping
    err = np.inf
    for it in range(1, max_iter + 1):
        e, J = _task_error_and_jacobian(robot, q, x_des, phi_des)
        err = float(np.linalg.norm(e))
        if err < tol:
            return IKResult(q=q, ok=True, error=err, method=method, iterations=it)

        m = J.shape[0]
        if method == "gradient":                    # Jacobiano transpuesto
            dq = alpha * (J.T @ e)
        elif method == "newton":                    # pseudo-inversa (Gauss-Newton)
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-12 * np.eye(m), e)
        else:                                       # "dls" / Levenberg-Marquardt
            dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(m), e)

        nrm = np.linalg.norm(dq)
        if nrm > step_clip:
            dq *= step_clip / nrm
        q = q + dq
        if respect_limits:
            q = robot.clamp(q)

    return IKResult(q=q, ok=False, error=err, method=method, iterations=max_iter,
                    reason="no convergió (¿fuera de alcance o singularidad?)")


# ---------------------------------------------------------------------------
# Interfaz unificada
# ---------------------------------------------------------------------------
def solve_ik(x_des, q0, *, approach=None, method="analytic", robot: DHChain = ROBOT,
             **kw) -> IKResult:
    """
    Punto de entrada único.
      method="analytic"  → IK cerrada (si `approach` es None usa −90°, top-down);
                           usa q0 como semilla para elegir la rama más cercana.
      method="dls"|"newton"|"gradient" → numérica (usa q0 como semilla).
    """
    x_des = np.asarray(x_des, dtype=float).ravel()
    if method == "analytic":
        phi = -pi / 2.0 if approach is None else approach   # por defecto: top-down
        return solve_analytic(x_des[0], x_des[1], x_des[2], phi, robot, seed=q0)
    return solve_numeric(x_des, q0, phi_des=approach, robot=robot, method=method, **kw)


# ---------------------------------------------------------------------------
# Utilidades de orientación (reutilizadas por nodos)
# ---------------------------------------------------------------------------
def rot_to_quat(R: np.ndarray):
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * s; qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s; qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s; qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s; qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s; qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s; qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s; qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s; qz = 0.25 * s
    return qx, qy, qz, qw


if __name__ == "__main__":
    np.set_printoptions(suppress=True, precision=5)
    rng = np.random.default_rng(0)
    print("Caso de uso real: pick & place sobre una MESA al frente, pinza ABAJO.")
    print("Objetivo cartesiano (x,y,z) + φ=−90°. Comparación de métodos:\n")
    phi = -pi / 2.0                       # pinza apuntando hacia abajo
    # Semilla numérica = postura 'lista para pick' (warm-start, como en operación).
    seed = solve_analytic(0.18, 0.0, 0.10, phi).q
    print(f"  semilla numérica (pick-ready) q={np.round(seed, 3)}")
    stats = {m: [0, 0.0] for m in ["analytic", "dls", "newton", "gradient"]}
    reachable = 0
    N = 200
    for _ in range(N):
        x = rng.uniform(0.10, 0.28)
        y = rng.uniform(-0.15, 0.15)
        z = rng.uniform(0.04, 0.22)
        target = np.array([x, y, z])
        if not solve_analytic(x, y, z, phi).ok:
            continue                      # objetivo no alcanzable apuntando abajo
        reachable += 1
        for m in stats:
            res = solve_ik(target, seed, approach=phi, method=m)
            d = np.linalg.norm(ROBOT.fkine(res.q)[0:3, 3] - target)
            dphi = abs(_wrap(approach_angle(res.q) - phi))
            stats[m][0] += int(d < 1e-3 and dphi < 1e-2)
            stats[m][1] += res.iterations
    print(f"  objetivos alcanzables (de {N} en la mesa): {reachable}\n")
    print(f"  {'método':9s}  aciertos   iters_prom   nota")
    notas = {"analytic": "exacta, 0 iteraciones (RECOMENDADA)",
             "dls": "robusta cerca de singularidades (mejor numérica)",
             "newton": "rápida; sin amortiguar es frágil",
             "gradient": "simple/barata, converge lento"}
    for m, (ok, its) in stats.items():
        print(f"  {m:9s}  {ok:3d}/{reachable:<3d}   {its/max(reachable,1):6.1f}     {notas[m]}")
