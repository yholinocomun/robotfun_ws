#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mesh_bbox.py
============

Diagnóstico para CENTRAR las piezas reales del brazo en RViz (modelo de meshes).

Las STL se exportaron en un MARCO CAD GLOBAL común; el <visual> de cada link sólo
compensa la traslación. Si alguna pieza se ve descentrada en XY, ajusta su
`<origin xyz>` en `urdf/common/arm.macro.xacro` (rama use_mesh=true). Este script
imprime el bounding box y su centro (a escala 0.001) para guiar ese ajuste.
No modifica nada.

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


MESHES = ["base_link.stl", "eslabon1_link.stl", "eslabon2_link.stl",
          "eslabon3_link.stl", "eslabon4_link.stl"]
SCALE = 0.001


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    mesh_dir = os.path.join(here, "..", "meshes", "arm")
    print(f"{'mesh':20s} bbox_min(scaled)        centro_xy(scaled)")
    for name in MESHES:
        mn, mx = bbox(os.path.join(mesh_dir, name))
        cx = (mn[0] + mx[0]) / 2 * SCALE
        cy = (mn[1] + mx[1]) / 2 * SCALE
        print(f"{name:20s} ({mn[0]*SCALE:+.3f},{mn[1]*SCALE:+.3f},{mn[2]*SCALE:+.3f})  "
              f"({cx:+.4f},{cy:+.4f})")
    print("\nAjusta el <origin xyz> de cada <visual> (use_mesh=true) en arm.macro.xacro")
    print("para centrar la pieza sobre el eje de su junta. El modelo primitivo")
    print("(model:=primitives) es la referencia cinemática exacta (TF == FK).")


if __name__ == "__main__":
    main()
