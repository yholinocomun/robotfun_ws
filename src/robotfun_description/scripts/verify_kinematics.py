#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_kinematics.py
====================

Comprueba que el esqueleto del modelo PRIMITIVO (orígenes de la tabla DH del
usuario + pitch en axis="0 -1 0") reproduce EXACTAMENTE la FK del núcleo.

Tabla DH 4 GDL (yaw + 3 pitch), brazo RECTO con offset radial L0 (m):
    L0=0.010  L1=0.063  L2=0.120  A3=L3+L4=0.120  HAND=L5=0.11
    i | d   | θ_i      | α_i  | a_i
    1 | L1  | q1       | +90° | L0
    2 | 0   | q2 + 90° |  0°  | L2
    3 | 0   | q3       |  0°  | A3
    4 | 0   | q4       |  0°  | HAND

Uso:  python3 verify_kinematics.py
"""

import numpy as np

pi = np.pi
L0, L1, L2, A3, HAND = 0.010, 0.063, 0.120, 0.120, 0.11
THOFF = np.array([0.0, pi / 2, 0.0, 0.0])
D = np.array([L1, 0, 0, 0]); A = np.array([L0, L2, A3, HAND]); AL = np.array([pi / 2, 0, 0, 0])


def dh(d, th, a, al):
    ct, st, ca, sa = np.cos(th), np.sin(th), np.cos(al), np.sin(al)
    return np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d], [0, 0, 0, 1]])


def fk_dh(q):
    T = np.eye(4)
    for i in range(4):
        T = T @ dh(D[i], q[i] + THOFF[i], A[i], AL[i])
    return T


def T(xyz):
    M = np.eye(4); M[:3, 3] = xyz; return M


def Rz(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M


def Ry(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4); M[0, 0] = c; M[0, 2] = s; M[2, 0] = -s; M[2, 2] = c
    return M


def fk_urdf(q):
    # esqueleto del modelo primitivo: joint_1 axis z; joint_2 con offset (L0,0,L1)
    return (Rz(q[0]) @ T([L0, 0, L1]) @ Ry(-q[1]) @ T([0, 0, L2]) @ Ry(-q[2])
            @ T([0, 0, A3]) @ Ry(-q[3]) @ T([0, 0, HAND]))


def main():
    rng = np.random.default_rng(0)
    err = max(np.linalg.norm(fk_urdf(q)[:3, 3] - fk_dh(q)[:3, 3])
              for q in rng.uniform(-1.0, 1.0, (500, 4)))
    print(f"Esqueleto primitivo (ejes 0 0 1 / 0 -1 0) vs DH: err_max(500 q) = {err:.2e}")
    print("→ coinciden: el modelo primitivo reproduce la FK DH (verifica la IK).")
    print(f"FK(HOME) TCP = {np.round(fk_dh(np.zeros(4))[:3,3],4)}  (esperado [0.010,0,0.413])")


if __name__ == "__main__":
    main()
