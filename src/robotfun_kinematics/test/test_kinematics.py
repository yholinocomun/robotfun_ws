#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tests del núcleo de cinemática 4 GDL (FK, IK analítica/numérica, workspace)."""

import numpy as np
import pytest

from robotfun_kinematics.core import (
    N_JOINTS, ROBOT, WorkspaceLimits, approach_angle, clamp_target,
    fkine, solve_analytic, solve_ik, validate_target,
)
from robotfun_kinematics.core.ik_solver import _wrap

PHI_DOWN = -np.pi / 2.0


def test_n_joints_is_four():
    assert N_JOINTS == 4


def test_fk_home_matches_urdf():
    """FK(HOME=q0): brazo recto y vertical (medidas reales)."""
    _, fr = ROBOT.fkine(np.zeros(N_JOINTS), return_frames=True)
    np.testing.assert_allclose(fr[0][:3, 3], [0.0, 0.0, 0.1375], atol=1e-4)   # hombro
    np.testing.assert_allclose(fr[1][:3, 3], [0.0, 0.0, 0.2652], atol=1e-4)   # codo
    np.testing.assert_allclose(fr[2][:3, 3], [0.0, 0.0, 0.3902], atol=1e-4)   # muñeca
    np.testing.assert_allclose(fr[3][:3, 3], [0.0, 0.0, 0.5102], atol=1e-4)   # TCP


def _reachable_front_targets(n, seed_rng=0):
    """Genera n objetivos cartesianos alcanzables apuntando abajo."""
    rng = np.random.default_rng(seed_rng)
    out = []
    while len(out) < n:
        x = rng.uniform(0.10, 0.28); y = rng.uniform(-0.15, 0.15); z = rng.uniform(0.04, 0.22)
        if solve_analytic(x, y, z, PHI_DOWN).ok:
            out.append((x, y, z))
    return out


def test_analytic_ik_exact():
    """La IK analítica reproduce posición y ángulo de aproximación exactos."""
    for (x, y, z) in _reachable_front_targets(40):
        res = solve_analytic(x, y, z, PHI_DOWN)
        assert res.ok
        p = ROBOT.fkine(res.q)[0:3, 3]
        assert np.linalg.norm(p - [x, y, z]) < 1e-6
        assert abs(_wrap(approach_angle(res.q) - PHI_DOWN)) < 1e-6


@pytest.mark.parametrize("method", ["dls", "newton"])
def test_numeric_ik_with_warm_seed(method):
    """DLS y Newton convergen a la pose con semilla pick-ready (warm-start)."""
    seed = solve_analytic(0.18, 0.0, 0.10, PHI_DOWN).q
    ok = 0
    targets = _reachable_front_targets(30, seed_rng=1)
    for (x, y, z) in targets:
        res = solve_ik([x, y, z], seed, approach=PHI_DOWN, method=method)
        p = ROBOT.fkine(res.q)[0:3, 3]
        if np.linalg.norm(p - [x, y, z]) < 1e-3 and abs(_wrap(approach_angle(res.q) - PHI_DOWN)) < 1e-2:
            ok += 1
    assert ok >= 28          # tolera algún caso al borde del espacio de trabajo


def test_joint_limits_respected():
    res = solve_ik([0.20, 0.0, 0.10], np.zeros(N_JOINTS), approach=PHI_DOWN, method="dls")
    assert np.all(res.q >= ROBOT.q_min - 1e-9)
    assert np.all(res.q <= ROBOT.q_max + 1e-9)


def test_unreachable_target_flagged():
    """Un objetivo demasiado lejos se marca como no alcanzable."""
    res = solve_analytic(0.6, 0.0, 0.4, PHI_DOWN)
    assert not res.ok


def test_workspace_validate_and_clamp():
    lim = WorkspaceLimits()
    ok, _ = validate_target([0.20, 0.0, 0.10], lim)
    assert ok
    bad, reason = validate_target([0.0, 0.0, 0.0], lim)   # bajo la mesa / sin alcance
    assert not bad and reason
    # clamp deja el punto dentro de la caja y del alcance
    c = clamp_target([0.9, 0.9, 0.9], lim)
    ok2, _ = validate_target(c, lim)
    assert ok2


def test_approach_angle_formula():
    """approach_angle(q) = q2+q3+q4 + π/2 (los offsets de codo y muñeca se cancelan)."""
    q = np.array([0.2, -0.4, 0.5, -0.3])
    assert abs(approach_angle(q) - (q[1] + q[2] + q[3] + np.pi / 2.0)) < 1e-9
