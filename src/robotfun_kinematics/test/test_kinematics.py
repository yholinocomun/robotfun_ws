#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests del núcleo de cinemática (FK, Jacobiano, IK) — sin ROS."""

import numpy as np
import pytest

from robotfun_kinematics.core import N_JOINTS, ROBOT, solve_ik
from robotfun_kinematics.core.ik_solver import ACTIVE_J4_FIXED


def test_fk_home():
    """FK en HOME (q=0) ≈ [0.010, 0, 0.393] m (validado contra el robot físico)."""
    p = ROBOT.fkine(np.zeros(N_JOINTS))[0:3, 3]
    np.testing.assert_allclose(p, [0.010, 0.0, 0.393], atol=1e-3)


def test_j4_position_column_zero_at_home():
    """La columna de J4 del Jacobiano de POSICIÓN es 0 en HOME y cuando J5=0
    (la punta cae sobre el eje del antebrazo). Con J5≠0 deja de ser 0."""
    # HOME y cualquier postura con J5 = 0 → punta sobre el eje Z4 → columna 0.
    for q5 in (0.0,):
        for _ in range(10):
            q = np.array([0.3, -0.4, 0.6, 0.5, q5])
            assert np.linalg.norm(ROBOT.jacobian_position(q)[:, 3]) < 1e-9
    # Muñeca flexionada (J5 ≠ 0): J4 sí desplaza la punta.
    q_bent = np.array([0.0, -0.5, 0.8, 0.0, 0.6])
    assert np.linalg.norm(ROBOT.jacobian_position(q_bent)[:, 3]) > 1e-3


@pytest.mark.parametrize("use_joint4", [False, True])
def test_ik_roundtrip(use_joint4):
    """FK(IK(x)) ≈ x para objetivos alcanzables, en ambos modos de J4."""
    rng = np.random.default_rng(2)
    q_seed = np.array([0.0, -0.5, 0.8, 0.0, 0.3])   # "ready" no singular
    ok_count = 0
    for _ in range(30):
        q_true = rng.uniform(-1.0, 1.0, N_JOINTS)
        if not use_joint4:
            q_true[3] = 0.0                          # J4 fijo: objetivo sin roll
        x_goal = ROBOT.fkine(q_true)[0:3, 3]
        res = solve_ik(x_goal, q_seed, use_joint4=use_joint4)
        x_reached = ROBOT.fkine(res.q)[0:3, 3]
        if np.linalg.norm(x_goal - x_reached) < 1e-3:
            ok_count += 1
    assert ok_count >= 27   # tolera algún objetivo en singularidad de frontera


def test_ik_keeps_j4_frozen_when_fixed():
    """Con J4 fijo, la solución NO mueve q4 respecto a la semilla."""
    q_seed = np.array([0.0, -0.5, 0.8, 0.0, 0.3])
    x_goal = ROBOT.fkine(np.array([0.4, -0.3, 0.5, 0.0, 0.2]))[0:3, 3]
    res = solve_ik(x_goal, q_seed, use_joint4=False)
    assert abs(res.q[3] - q_seed[3]) < 1e-12


def test_joint_limits_respected():
    """La IK satura a [q_min, q_max]."""
    q_seed = np.zeros(N_JOINTS)
    res = solve_ik([1.0, 1.0, 1.0], q_seed, use_joint4=True)  # objetivo lejano
    assert np.all(res.q >= ROBOT.q_min - 1e-9)
    assert np.all(res.q <= ROBOT.q_max + 1e-9)


def test_active_mask_default_is_j4_fixed():
    assert list(ACTIVE_J4_FIXED) == [True, True, True, False, True]
