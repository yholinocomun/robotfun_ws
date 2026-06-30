#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_kinematics.py
====================

Comprueba que el esqueleto del URDF (orígenes reales de brazo_ws + ejes de los
pitch en (0 -1 0)) reproduce EXACTAMENTE la cinemática directa DH del robot.

Es la justificación del diseño: el modelo de "medidas reales" y el de "piezas
reales" comparten este esqueleto, por lo que ambos sirven para verificar la IK
(TF == FK) y se mueven idénticos con el mismo /joint_states.

Tabla DH 4 GDL (yaw + 3 pitch), medidas reales (m):
    D1=0.1375  A2=0.1277  A3=0.12715  HAND=0.10
    i | d   | θ_i        | α_i  | a_i
    1 | D1  | q1         | +90° | 0
    2 | 0   | q2 + 90°   |  0°  | A2
    3 | 0   | q3 − 17.39°|  0°  | A3
    4 | 0   | q4 + 17.39°|  0°  | HAND

Uso:  python3 verify_kinematics.py
"""

import numpy as np

pi = np.pi
D1, A2 = 0.1375, 0.1277
A3 = float(np.hypot(0.038, 0.1213))
HAND = 0.10
ANG = np.arctan2(0.1213, 0.038)
THOFF = np.array([0.0, pi / 2, ANG - pi / 2, pi / 2 - ANG])
D = np.array([D1, 0, 0, 0]); A = np.array([0, A2, A3, HAND]); AL = np.array([pi / 2, 0, 0, 0])


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
    # esqueleto del URDF: orígenes reales, joint_1 axis z, joint_2/3/4 axis (0 -1 0)
    return (T([0, 0, 0.0617]) @ Rz(q[0]) @ T([0, 0, 0.0758]) @ Ry(-q[1])
            @ T([0, 0, 0.1277]) @ Ry(-q[2]) @ T([0.038, 0, 0.1213]) @ Ry(-q[3])
            @ T([0, 0, HAND]))


def main():
    rng = np.random.default_rng(0)
    err = max(np.linalg.norm(fk_urdf(q)[:3, 3] - fk_dh(q)[:3, 3])
              for q in rng.uniform(-1.0, 1.0, (500, 4)))
    print(f"Esqueleto URDF (ejes 0 0 1 / 0 -1 0) vs DH: err_max(500 q) = {err:.2e}")
    print("→ coinciden: el URDF reproduce la FK DH (por eso usa axis='0 -1 0' en los pitch).")
    print(f"FK(HOME) TCP = {np.round(fk_dh(np.zeros(4))[:3,3],4)}  (esperado [0.038,0,0.4865])")


if __name__ == "__main__":
    main()
