# Arquitectura — Brazo 4 GDL + gripper (robotfun_ws)

Sistema modular para un brazo **antropomórfico de 4 GDL (yaw + 3 pitch) + gripper**
con realimentación por potenciómetros y servos accionados por **ESP32 + micro-ROS**,
integrado en **ROS 2**. Objetivo inmediato: **cinemática inversa** robusta.
Objetivo final: **pick & place de pastillas** guiado por cámara (QR/color).

> El roll de muñeca (antiguo joint_4) se **eliminó**; el antiguo joint_5 es ahora
> joint_4. Un brazo yaw+3·pitch tiene **IK analítica cerrada** (la más eficiente).

## Principios de diseño
- **Clean Architecture / Dependency Inversion**: núcleo de cinemática Python
  **puro, sin ROS**; los nodos ROS son adaptadores delgados. Testeable sin ROS.
- **SOLID / DRY**: la tabla DH vive en UN sitio (`core/dh_model.py`); el URDF
  reproduce esa FK (TF == FK a 1e-16); los dos modelos comparten un único
  esqueleto (`urdf/common/arm.macro.xacro`).
- **Interfaz de juntas única** (`joint_1..joint_4`, `gripper`) en descripción,
  cinemática y firmware.

## Paquetes
```
robotfun_description/   URDF/xacro: medidas reales (primitivas) + piezas reales (meshes)
robotfun_kinematics/    núcleo puro (FK, Jacobiano, IK analítica+numérica, workspace) + nodos
robotfun_firmware/      ESP32 micro-ROS (4 juntas + gripper; roll muerto a 90°) + Agent
robotfun_bringup/       composition root (launch integradores) + docs
```

## Modelo cinemático (DH estándar, medidas reales)
`D1=0.1375  A2=0.1277  A3=0.12715  HAND=0.100` (m). HOME = juntas a 0 rad ⇒
servos a 90° ⇒ `FK(HOME)` sitúa hombro/codo/muñeca en las posiciones del URDF
(verificado) y el TCP en `[0.038, 0, 0.4865]`.

| i | d_i | θ_i | α_i | a_i |
|---|-----|-----|-----|-----|
| 1 | D1 | q1 | +90° | 0 |
| 2 | 0 | q2+90° | 0° | A2 |
| 3 | 0 | q3−17.39° | 0° | A3 |
| 4 | 0 | q4+17.39° | 0° | HAND |

El URDF usa `axis="0 0 1"` (yaw) y `axis="0 -1 0"` (los 3 pitch); con ese signo
reproduce exactamente la FK DH (ver `scripts/verify_kinematics.py`). Dos modelos
sin conflicto, misma interfaz de juntas: `display.launch.py model:=primitives`
(medidas reales, referencia de IK) y `model:=meshes` (piezas reales del CAD).

## Cinemática inversa — métodos (de mejor a más general)
Para 4 GDL la tarea es **posición (3) + ángulo de aproximación φ (1)** = 4 ecuaciones.

| método | idea | carácter |
|--------|------|----------|
| **analytic** (recom.) | yaw=atan2 + 2R planar por ley de cosenos | exacta, 0 iteraciones, global |
| **dls** (Levenberg-Marquardt) | `dq=Jᵀ(JJᵀ+λ²I)⁻¹e` | robusta cerca de singularidades |
| **newton** (Gauss-Newton) | `dq=J⁺e` | rápida pero frágil sin amortiguar |
| **gradient** (Jacobiano transpuesto) | `dq=αJᵀe` | simple, converge lento |

Los métodos numéricos son **locales** (dependen de la semilla; se atascan en
límites desde una semilla lejana). En operación se siembran con la postura
actual (warm-start). La **analítica es global y exacta** → es la recomendada.
`φ`: +90°=arriba, 0°=horizontal, **−90°=abajo (pick top-down)**.

## Restricción del área de trabajo (workspace)
`core/workspace.py` define `WorkspaceLimits` (caja x/y/z + alcance radial r) y:
`validate_target()` (¿dentro?), `clamp_target()` (recorta a la zona segura). El
`ik_node` la aplica (parámetro `enforce_workspace`). Ver el README para cómo
ajustar la zona de pick & place. Evita objetivos fuera de alcance, bajo la mesa o
en la singularidad central (r≈0, eje de yaw).

## Flujo de datos
```
   /target_pose ─► ik_node (IK) ─┐
   /joint_goal  ─► trajectory ───┼─► /joint_command [q1..q4,gripper] ─► ESP32 ─► servos
                                  │                                       │
   RViz ◄─ robot_state_publisher ◄┴──────── /joint_states ◄───────────────┘ (pots, 25 Hz)
                fk_check_node ─► /fk_pose (valida TF==FK)
```

## Inconsistencias corregidas
- En `twin_ws`, el URDF no coincidía con la DH actualizada → aquí el URDF
  reproduce la FK (TF==FK).
- En `brazo_ws`, `eslabon3` quedó a escala 0.001 (flota a ~2.4 m): corregido a
  0.0001. Pieza sobrante `eslabon3_1` y engranajes eliminados.
