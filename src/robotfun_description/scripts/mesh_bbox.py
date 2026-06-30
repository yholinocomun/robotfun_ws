#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mesh_bbox.py
============

Herramienta de diagnóstico para CENTRAR las meshes reales del brazo en RViz.

Las STL del robot se exportaron en un MARCO CAD GLOBAL COMÚN (todas comparten la
misma esquina máxima del bounding box), no re-origenadas por eslabón. El URDF de
meshes (`robotfun_meshes.urdf.xacro`) solo compensa Z, por eso algunas piezas se
ven descentradas en XY.

Solución recomendada (cualquiera de las dos):
  (A) Re-exportar cada STL desde el CAD con el origen en el eje de su junta.
  (B) Afinar el `<origin xyz>` del <visual> de cada link en RViz.

Este script imprime el bounding box y su centro (a la escala de cada malla) para
guiar el ajuste manual del paso (B). NO modifica nada.

Uso:  python3 mesh_bbox.py
"""

import os
import struct


def bbox(path):
    with open(path, "rb") as f:
        data = f.read()
    verts = []
    if data[:5].lower() == b"solid" and b"facet" in data[:2000].lower():
        for ln in data.decode("latin1").splitlines():
            ln = ln.strip()
            if ln.startswith("vertex"):
                p = ln.split()
                verts.append((float(p[1]), float(p[2]), float(p[3])))
    else:
        n = struct.unpack("<I", data[80:84])[0]
        for i in range(n):
            b = 84 + i * 50
            for j in range(3):
                verts.append(struct.unpack("<3f", data[b + 12 + j * 12:b + 24 + j * 12]))
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


# (mesh, escala usada en el xacro)
ITEMS = [
    ("base_link.stl", 0.001),
    ("eslabon1_link.stl", 0.001),
    ("eslabon2_link.stl", 0.001),
    ("eslabon3_link.stl", 0.0001),
    ("eslabon3_1link.stl", 0.0001),
    ("eslabon4_link.stl", 0.001),
]


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mesh_dir = os.path.join(here, "..", "meshes", "arm")
    print(f"{'mesh':20s} {'escala':7s}  bbox_min(scaled)        centro_xy(scaled)")
    for name, sc in ITEMS:
        mn, mx = bbox(os.path.join(mesh_dir, name))
        cx = (mn[0] + mx[0]) / 2 * sc
        cy = (mn[1] + mx[1]) / 2 * sc
        print(f"{name:20s} {sc:<7} "
              f"({mn[0]*sc:+.3f},{mn[1]*sc:+.3f},{mn[2]*sc:+.3f})  ({cx:+.4f},{cy:+.4f})")
    print("\nNota: estas meshes comparten un marco CAD global; NO las centres por")
    print("bbox de forma independiente (rompe la alineación relativa). Ajusta el")
    print("<origin> de cada <visual> en RViz, o re-exporta los STL desde el CAD.")


if __name__ == "__main__":
    main()
