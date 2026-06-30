#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_dh_origins.py
======================

Genera los `<origin xyz rpy>` de los joints del URDF a partir de la tabla DH
**validada** (la misma que usa `robotfun_kinematics.core.dh_model`), de modo que
el árbol TF de RViz coincida EXACTAMENTE con la FK del núcleo (TF == FK).

Conversión DH estándar → URDF (joints con axis="0 0 1"):
    M_0 = I
    M_i = Tz(d_i)·Tx(a_i)·Rx(alpha_i)               (parte no rotacional de A_i)
    origin(joint_i) = M_{i-1} · Rz(theta_offset_i)  (relativo al link padre)
    tool0 (efector real) se alcanza con un joint fijo de origen M_n.

Definiendo G_i = F_i·M_i (F_i = frame URDF de link_i) se demuestra G_i = T_0^i,
por lo que F_n·M_n = T_0^n (la pose real del efector). Verificado abajo a 1e-12.
"""

import numpy as np

pi = np.pi
L0, L1, L2, L3, L4, L5 = 0.010, 0.063, 0.120, 0.090, 0.030, 0.090
D = np.array([L1, 0.0, 0.0, L3 + L4, 0.0])
A = np.array([L0, L2, 0.0, 0.0, L5])
ALPHA = np.array([pi / 2, 0.0, pi / 2, pi / 2, 0.0])
THETA_OFF = np.array([0.0, pi / 2, pi / 2, 0.0, pi / 2])
N = 5


def Tz(d):
    T = np.eye(4); T[2, 3] = d; return T


def Tx(a):
    T = np.eye(4); T[0, 3] = a; return T


def Rx(al):
    c, s = np.cos(al), np.sin(al)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def Rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rot_to_rpy(R):
    """Matriz 3x3 → roll,pitch,yaw (convención URDF Rz·Ry·Rx)."""
    if abs(R[2, 0]) < 1.0 - 1e-9:
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        yaw = 0.0
        if R[2, 0] <= -1.0 + 1e-9:
            pitch = pi / 2; roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -pi / 2; roll = np.arctan2(-R[0, 1], -R[0, 2])
    return roll, pitch, yaw


def fmt(T):
    xyz = T[0:3, 3]
    r, p, y = rot_to_rpy(T[0:3, 0:3])
    return (f'xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" '
            f'rpy="{r:.6f} {p:.6f} {y:.6f}"')


def main():
    M = [np.eye(4)]                       # M[0] = M_0 = I
    origins = []
    for i in range(N):
        origins.append(M[i] @ Rz(THETA_OFF[i]))           # origin(joint_{i+1})
        M.append(Tz(D[i]) @ Tx(A[i]) @ Rx(ALPHA[i]))      # M_{i+1}

    print("# ---- orígenes para el xacro (TF == FK) ----")
    for i, O in enumerate(origins):
        print(f"joint_{i+1} (parent link_{i}): {fmt(O)}")
    print(f"tool_fixed (parent link_5) = M_5: {fmt(M[N])}")

    # Verificación: reconstruir EE por la cadena URDF y comparar con FK directa.
    F = np.eye(4)                          # F_0 = base
    for i in range(N):
        F = F @ origins[i]                 # q = 0 ⇒ Rz(q)=I
    ee_urdf = F @ M[N]
    Tfk = np.eye(4)
    for i in range(N):
        Tfk = Tfk @ Rz(THETA_OFF[i]) @ Tz(D[i]) @ Tx(A[i]) @ Rx(ALPHA[i])
    err = np.linalg.norm(ee_urdf - Tfk)
    print(f"\nVerificación TF==FK en HOME: ||EE_urdf - FK|| = {err:.2e}")
    print(f"EE pos (URDF) = {np.round(ee_urdf[0:3,3],4)} | FK = {np.round(Tfk[0:3,3],4)}")


if __name__ == "__main__":
    main()
