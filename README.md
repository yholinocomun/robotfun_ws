# RobotFun — Brazo robótico 4 GDL + gripper (ROS 2)

Workspace del brazo **Pick & Place** antropomórfico de **4 grados de libertad
(yaw + 3 pitch) + gripper**, con realimentación por potenciómetros y servos
accionados por **ESP32 + micro-ROS**. Objetivo inmediato: **cinemática inversa**
robusta. Objetivo final: **pick & place de pastillas** guiado por cámara.

> El roll de muñeca (antiguo joint_4) se **eliminó**; el antiguo joint_5 es ahora
> **joint_4**. Estructura yaw–pitch–pitch–pitch → **IK analítica cerrada**.

## Paquetes
| Paquete | Rol |
|---------|-----|
| `robotfun_description` | URDF/xacro: modelo de **medidas reales** (primitivas, TF==FK) y de **piezas reales** (meshes del CAD); ambos con la misma interfaz de juntas. |
| `robotfun_kinematics` | Núcleo **puro** (FK, Jacobiano, IK analítica + DLS/Newton/gradiente, workspace) + nodos ROS. |
| `robotfun_firmware` | Firmware ESP32 micro-ROS (4 juntas + gripper; servo de roll muerto a 90°) + Agent. |
| `robotfun_bringup` | Composition root + `docs/ARCHITECTURE.md`, `docs/ROADMAP.md`. |

## Compilar
```bash
cd ~/robotfun_ws
colcon build --symlink-install
source install/setup.bash
```

## Ejecutar — usa SIEMPRE `bringup` con un `mode` (una sola fuente de /joint_states)

> ⚠️ El temblor/bucle en RViz ocurre si dos nodos publican `/joint_states` a la vez.
> Por eso **no** lances `display.launch.py` junto con `bringup`. Usa solo `bringup`
> y cambia el `mode`. Cada modo tiene UNA sola fuente de `/joint_states`.

```bash
# (A) SIMULACIÓN: el robot sigue a la IK SIN hardware (relay = ESP32 virtual).
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=primitives
#   en otra terminal, manda un objetivo ALCANZABLE (ver tabla abajo):
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.10, y: 0.0, z: 0.30}}" --once
ros2 topic echo /fk_pose          # comprobar que coincide con el objetivo

# Ver tus piezas reales moviéndose con la misma IK:
ros2 launch robotfun_bringup bringup.launch.py mode:=sim model:=meshes

# (B) SLIDERS manuales (sin cinemática), p. ej. para inspeccionar el modelo:
ros2 launch robotfun_bringup bringup.launch.py mode:=gui model:=meshes
```

### Objetivos alcanzables (brazo pequeño: alcance ≈ 0.35 m)
La pinza recta **hacia abajo (−90°)** es muy exigente para este brazo; usa
`approach_deg` mayor para llegar a más puntos:
```bash
# pinza horizontal (φ=0°): muchos puntos —
ros2 launch robotfun_bringup bringup.launch.py mode:=sim approach_deg:=0.0
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.10, y: 0.0, z: 0.30}}" --once
# pinza a −45°:
ros2 launch robotfun_bringup bringup.launch.py mode:=sim approach_deg:=-45.0
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.20, y: 0.0, z: 0.15}}" --once
# pinza abajo (−90°), solo puntos bajos y cercanos:
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.16, y: 0.0, z: 0.05}}" --once
```
Si ves `IK no resolvió ... fuera de límites/espacio de trabajo`, el objetivo no es
alcanzable con ese ángulo: acércalo, súbelo, o usa un `approach_deg` mayor.

## Con hardware (ESP32)
```bash
# 1) agente micro-ROS (el ESP32 será la fuente de /joint_states):
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0
# 2) bringup en modo hardware (sin relay ni sliders):
ros2 launch robotfun_bringup bringup.launch.py mode:=hardware model:=meshes
```

## Verificar el núcleo (sin ROS)
```bash
cd ~/robotfun_ws/src/robotfun_kinematics
python3 -m robotfun_kinematics.core.dh_model     # FK(HOME) vs URDF
python3 -m robotfun_kinematics.core.ik_solver    # comparación de métodos de IK
pytest test/                                      # tests del núcleo
```

---

## Cinemática inversa: métodos disponibles
Para 4 GDL la tarea es **posición (x,y,z) + ángulo de aproximación φ**. Elige con
`method:=...` (en `bringup` o `kinematics.launch.py`):

| método | cuándo usarlo |
|--------|---------------|
| **analytic** (por defecto) | **siempre que puedas**: exacta, instantánea, global. |
| **dls** (Levenberg-Marquardt) | numérica robusta; buena cerca de singularidades. |
| **newton** (Gauss-Newton) | numérica rápida; necesita buena semilla. |
| **gradient** (Jacobiano transpuesto) | didáctica; converge lento. |

`φ` se da en grados con `approach_deg` (**−90 = pinza hacia abajo**, lo típico de
pick & place). Los métodos numéricos son *locales*: en operación se siembran con
la postura actual del robot (warm-start). La analítica no necesita semilla.

## Cómo restringir el ÁREA DE TRABAJO (IK y pick & place)
La zona segura se define en `core/workspace.py` (`WorkspaceLimits`) y la aplica el
`ik_node`. Tres límites combinables:
- **Caja cartesiana** `x/y/z` — la mesa/bandeja de pastillas.
- **Alcance radial** `r_min..r_max` desde el hombro (máx ≈ 0.355 m; `r_min` evita
  la singularidad central del eje de yaw).
- **Altura mínima** `z_min` — no bajar de la superficie (no chocar la mesa).

Ajústalos por parámetros al lanzar el `ik_node`, p. ej. una mesa de 25×30 cm:
```bash
ros2 run robotfun_kinematics ik_node --ros-args \
  -p method:=analytic -p approach_deg:=-90.0 -p enforce_workspace:=true \
  -p ws_x:="[0.10, 0.28]" -p ws_y:="[-0.15, 0.15]" -p ws_z:="[0.03, 0.20]"
```
Con `enforce_workspace:=true`, un objetivo fuera de la zona se **recorta** a ella
(en vez de forzar una solución peligrosa); con `false` solo avisa. Para pick &
place: pon `z_min` en la altura de la mesa y la caja `x/y` sobre la zona de
pastillas; el `r_max` impide pedir puntos inalcanzables.

---

## Decisiones de diseño (resumen)
- **J4 roll eliminado** → 4 GDL → IK **analítica cerrada** (lo más eficiente y exacto).
- **Radianes en el bus ROS**, grados solo en `servo.write()` (calibración en grados).
- **Dos modelos sin conflicto** (medidas reales / piezas reales), misma interfaz de
  juntas, ambos TF==FK → uno verifica la IK y el otro visualiza el hardware.

Detalle técnico completo en `src/robotfun_bringup/docs/ARCHITECTURE.md`.
