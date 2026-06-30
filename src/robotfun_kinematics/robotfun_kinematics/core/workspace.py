#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
workspace.py
============

Restricción del **área de trabajo** del robot (capa de dominio, sin ROS).

Sirve para dos cosas que pediste:
  1) Limitar la IK: rechazar (o recortar) objetivos que el brazo no puede o no
     debe alcanzar, evitando soluciones forzadas/peligrosas.
  2) Acotar el pick & place a una zona segura (la mesa / bandeja de pastillas).

Tres tipos de límite, combinables:
  * ALCANCE radial desde el hombro: r_min ≤ |TCP − hombro| ≤ r_max. El máximo es
    A2+A3+HAND; un mínimo evita la zona singular pegada al cuerpo.
  * CAJA cartesiana (la zona útil): x,y,z ∈ [min,max]. p. ej. la mesa de trabajo.
  * ALTURA mínima z_floor: no bajar de la superficie (no chocar la mesa).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .dh_model import A2, A3, D1, HAND, ROBOT, DHChain


@dataclass
class WorkspaceLimits:
    """Límites del área de trabajo (metros). Valores None = sin ese límite."""

    x_min: float = -0.30
    x_max: float = 0.30
    y_min: float = -0.30
    y_max: float = 0.30
    z_min: float = 0.02       # altura mínima (superficie de la mesa, no bajar más)
    z_max: float = 0.45
    r_min: float = 0.08       # alcance mínimo desde el hombro (evita el cuerpo)
    r_max: float = A2 + A3 + HAND   # alcance máximo físico (≈0.35 m)

    def shoulder(self, robot: DHChain = ROBOT) -> np.ndarray:
        """Centro del hombro (pivote de los pitch) sobre el eje de yaw."""
        return np.array([0.0, 0.0, robot.shoulder_height])


def reach(point, robot: DHChain = ROBOT) -> float:
    """Distancia del punto al hombro (alcance radial)."""
    return float(np.linalg.norm(np.asarray(point, dtype=float) - WorkspaceLimits().shoulder(robot)))


def validate_target(point, limits: WorkspaceLimits = WorkspaceLimits(),
                    robot: DHChain = ROBOT):
    """
    Comprueba si un objetivo cartesiano está dentro del área de trabajo.
    Devuelve (ok: bool, reason: str).
    """
    x, y, z = np.asarray(point, dtype=float).ravel()[:3]
    if not (limits.x_min <= x <= limits.x_max):
        return False, f"x={x:.3f} fuera de [{limits.x_min}, {limits.x_max}]"
    if not (limits.y_min <= y <= limits.y_max):
        return False, f"y={y:.3f} fuera de [{limits.y_min}, {limits.y_max}]"
    if not (limits.z_min <= z <= limits.z_max):
        return False, f"z={z:.3f} fuera de [{limits.z_min}, {limits.z_max}]"
    rr = reach(point, robot)
    if not (limits.r_min <= rr <= limits.r_max):
        return False, f"alcance={rr:.3f} fuera de [{limits.r_min}, {limits.r_max:.3f}]"
    return True, "ok"


def clamp_target(point, limits: WorkspaceLimits = WorkspaceLimits(),
                 robot: DHChain = ROBOT) -> np.ndarray:
    """
    Recorta un objetivo al área de trabajo: primero a la caja, luego al alcance
    máximo (proyectando radialmente hacia el hombro si se pasa de r_max).
    """
    p = np.asarray(point, dtype=float).ravel()[:3].copy()
    p[0] = np.clip(p[0], limits.x_min, limits.x_max)
    p[1] = np.clip(p[1], limits.y_min, limits.y_max)
    p[2] = np.clip(p[2], limits.z_min, limits.z_max)
    sh = limits.shoulder(robot)
    v = p - sh
    rr = np.linalg.norm(v)
    if rr > limits.r_max and rr > 1e-9:
        p = sh + v * (limits.r_max / rr)
    return p
