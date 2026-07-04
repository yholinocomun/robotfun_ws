# Arquitectura — Brazo 4 GDL + gripper (robotfun_ws)

Sistema modular para un brazo **antropomórfico de 4 GDL (yaw + 3 pitch) + gripper**
con realimentación por potenciómetros y servos accionados por **ESP32 + micro-ROS**,
integrado en **ROS 2**. Objetivo inmediato: **cinemática inversa** robusta.
Objetivo final: **pick & place de pastillas** guiado por cámara (QR/color).

> El roll de muñeca (antiguo joint_4) se **eliminó**; el proyecto se reorganiza
> como **5 actuadores** `joint_1..joint_5` (joint_4 = pitch de muñeca, joint_5 =
> gripper). Un brazo yaw+3·pitch tiene **IK analítica cerrada** (la más eficiente).

## Principios de diseño
- **Clean Architecture / Dependency Inversion**: núcleo de cinemática Python
  **puro, sin ROS**; los nodos ROS son adaptadores delgados. Testeable sin ROS.
- **SOLID / DRY**: la tabla DH vive en UN sitio (`core/dh_model.py`); el URDF
  reproduce esa FK (TF == FK a 1e-16); los dos modelos comparten un único
  esqueleto (`urdf/common/arm.macro.xacro`).
- **Interfaz de juntas única** (`joint_1..joint_5`, donde `joint_5` = el gripper)
  en descripción, cinemática y firmware.

## Paquetes
```
robotfun_description/   URDF/xacro: medidas reales (primitivas) + piezas reales (meshes)
robotfun_kinematics/    núcleo puro (FK, Jacobiano, IK analítica+numérica, workspace) + nodos
robotfun_firmware/      ESP32 micro-ROS (5 actuadores joint_1..joint_5; roll eliminado) + Agent
robotfun_bringup/       composition root (launch integradores) + docs
```

## Modelo cinemático (tabla DH del usuario, brazo RECTO con offset base L0)
`L0=0.010  L1=0.063  L2=0.120  A3=L3+L4=0.120  HAND=L5=0.11` (m). HOME = juntas a
0 rad ⇒ servos a 90° ⇒ brazo recto vertical ⇒ `FK(HOME)` → hombro (0.010,0,0.063),
codo (0.010,0,0.183), muñeca (0.010,0,0.303), **TCP (0.010,0,0.413)**.

| i | d_i | θ_i | α_i | a_i |
|---|-----|-----|-----|-----|
| 1 | L1 | q1 | +90° | L0 |
| 2 | 0 | q2+90° | 0° | L2 |
| 3 | 0 | q3 | 0° | A3 |
| 4 | 0 | q4 | 0° | HAND |

El modelo PRIMITIVO usa `axis="0 0 1"` (yaw) y `axis="0 -1 0"` (los 3 pitch) y
reproduce exactamente la FK DH (TF==FK, ver `scripts/verify_kinematics.py`): es la
referencia para verificar la IK. El modelo de MESHES usa los orígenes del CAD
(brazo.urdf, donde las piezas están alineadas) con las mismas juntas y sentidos
de giro; sirve para visualizar el hardware (sus proporciones difieren ~12 cm del
DH; ambos se mueven con el mismo /joint_states). Sin cubo ni piezas flotantes.
`display.launch.py model:=primitives | meshes`.

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
   /joint_goal  ─► trajectory ───┼─► /joint_command [q1..q4,joint_5] ─► ESP32 ─► servos
                                  │                                       │
   RViz ◄─ robot_state_publisher ◄┴──────── /joint_states ◄───────────────┘ (pots, 25 Hz)
                fk_check_node ─► /fk_pose (valida TF==FK)
```
El ESP32 no salta al objetivo: cada `/joint_command` fija un objetivo y un **perfil
trapezoidal por junta** (vel + acel limitadas, re-planificado @50 Hz) mueve los servos
suavemente. Es un suavizado local que complementa al `trajectory_node` y protege el
hardware aunque el comando llegue crudo. Detalle en `robotfun_firmware/README.md`.

## Regla de UNA sola fuente de /joint_states (evita el temblor/bucle en RViz)
RViz "salta" si dos nodos publican `/joint_states` a la vez. `bringup.launch.py`
lo impide con el argumento `mode`, dejando una única fuente:
- `mode:=sim` → **`joint_state_relay`** (ESP32 virtual): `/joint_command → /joint_states`.
  Así el robot sigue a la IK sin hardware.
- `mode:=gui` → `joint_state_publisher_gui` (sliders), sin cinemática.
- `mode:=hardware` → el ESP32 publica `/joint_states` (relay y sliders apagados).
No lanzar `display.launch.py` junto con `bringup` (duplicaría la fuente).

## Inconsistencias corregidas
- En `twin_ws`, el URDF no coincidía con la DH actualizada → aquí el URDF
  reproduce la FK (TF==FK).
- En `brazo_ws`, `eslabon3` quedó a escala 0.001 (flota a ~2.4 m): corregido a
  0.0001. Pieza sobrante `eslabon3_1` y engranajes eliminados.
