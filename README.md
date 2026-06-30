# RobotFun — Brazo robótico 5 GDL + gripper (ROS 2)

Workspace de desarrollo del brazo **Pick & Place** de 5 grados de libertad (RRRRR)
+ gripper, con realimentación por potenciómetros y servos accionados por
**ESP32 + micro-ROS**. Objetivo inmediato: **cinemática inversa** robusta.
Objetivo final: **pick & place de pastillas** guiado por cámara (QR/color).

## Paquetes
| Paquete | Rol |
|---------|-----|
| `robotfun_description` | URDF/xacro: modelo **primitivo DH-exacto** (TF == FK) y modelo de **meshes reales** del CAD; RViz, límites, posición inicial. |
| `robotfun_kinematics` | Núcleo **puro** (FK, Jacobiano, IK por DLS, trayectorias) + nodos ROS (IK, comprobación FK, control trapezoidal). |
| `robotfun_firmware` | Firmware ESP32 micro-ROS (5 GDL + gripper), **dos versiones por `#define USE_JOINT4`** + launch del Agent. |
| `robotfun_bringup` | Composition root (launch integradores) + `docs/ARCHITECTURE.md` y `docs/ROADMAP.md`. |

Arquitectura y decisiones: ver **`robotfun_bringup/docs/ARCHITECTURE.md`**.

## Compilar
```bash
cd ~/robotfun_ws
colcon build --symlink-install
source install/setup.bash
```

## Ejecutar (sin hardware)
```bash
# Visualizar el modelo canónico (primitivas) con sliders:
ros2 launch robotfun_description display.launch.py
# Comparar con las piezas reales del CAD:
ros2 launch robotfun_description display.launch.py model:=meshes

# Sistema completo (descripción + cinemática):
ros2 launch robotfun_bringup bringup.launch.py
# Mandar un objetivo cartesiano (la IK calcula q y publica /joint_command):
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{position: {x: 0.12, y: -0.05, z: 0.18}}" --once
# Ver la FK de comprobación:
ros2 topic echo /fk_pose
```

## Con hardware (ESP32)
```bash
# 1) Sube robotfun_firmware/firmware/robotfun_esp32_microros.ino (Arduino IDE).
# 2) Arranca el Agent micro-ROS:
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0
# 3) Cinemática + RViz (sin sliders: el ESP32 ya publica /joint_states):
ros2 launch robotfun_bringup bringup.launch.py gui:=false
```

## Verificar el núcleo de cinemática (sin ROS)
```bash
cd ~/robotfun_ws/src/robotfun_kinematics
python3 -m robotfun_kinematics.core.dh_model     # FK(HOME) y redundancia de J4
python3 -m robotfun_kinematics.core.ik_solver    # roundtrip FK<->IK (J4 fijo/activo)
pytest test/                                      # 7 tests
```

---

## Respuestas a las dos decisiones que pediste analizar

### ¿J4 anulado o J4 activo? → **Empieza con J4 ANULADO (fijo) para el pick & place**
J4 es un **roll del antebrazo**: en HOME su efecto sobre la posición de la punta
es **nulo** y cerca de HOME **mal-condiciona** la IK; además J1+J2+J3 ya cubren la
posición 3D y J5 el cabeceo de aproximación. Para coger pastillas (objetos
axisimétricos) el giro de la pinza sobre su eje es irrelevante. Por eso la IK por
defecto **congela J4** (4 GDL efectivos): mejor condicionada y sin deriva.
La versión **J4 activo** (5 GDL) queda lista para cuando la **cámara** dé la
orientación del objeto. Cambias de versión con:
- alto nivel: `use_joint4:=true|false`
- firmware: `#define USE_JOINT4 1|0`

El hardware **siempre** mantiene los 5 GDL + gripper; solo cambia el modo de IK.

### ¿Grados o radianes en el firmware? → **Radianes en el bus ROS; grados solo en el servo**
ROS/RViz/MoveIt2 exigen **radianes** (REP-103, `sensor_msgs/JointState`). El servo
se manda en grados (0..180), así que la conversión rad→grados ocurre
**únicamente** en `servo.write()`. La **calibración** del firmware se expresa en
grados/ADC (lo intuitivo del hardware). Así el sistema es interoperable con
MoveIt2 y a la vez fácil de calibrar.
