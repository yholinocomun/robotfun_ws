# Arquitectura — Brazo 5 GDL + gripper (robotfun_ws)

Sistema modular para un brazo de **5 grados de libertad (RRRRR) + gripper** con
realimentación por potenciómetros y servos accionados por **ESP32 + micro-ROS**,
integrado en **ROS 2**. Objetivo inmediato: **cinemática inversa** robusta.
Objetivo final: **pick & place de pastillas** guiado por cámara (QR/color).

## Principios de diseño
- **Clean Architecture / Dependency Inversion**: el núcleo de cinemática es
  Python **puro, sin ROS**; los nodos ROS son adaptadores delgados que dependen
  del núcleo, nunca al revés. Se puede testear la matemática sin levantar ROS.
- **SOLID / DRY**: la tabla DH vive en UN sitio (`core/dh_model.py`); el URDF se
  **genera** desde ella (TF == FK); el firmware tiene UNA base con dos versiones
  por `#define`. Sin duplicación de la fuente de verdad.
- **Modularidad por paquetes** con interfaz de juntas única (`joint_1..joint_5`,
  `gripper`) compartida por descripción, cinemática y firmware.

## Paquetes
```
robotfun_description/   URDF/xacro (primitivas DH-exactas + meshes reales), RViz, config
robotfun_kinematics/    núcleo puro (FK, Jacobiano, IK DLS, trayectorias) + nodos ROS
robotfun_firmware/      ESP32 micro-ROS (#define USE_JOINT4) + launch del Agent
robotfun_bringup/       composition root (launch integradores) + docs
```

## Flujo de datos
```
        ALTO NIVEL (PC, ROS 2)                         BAJO NIVEL (ESP32, micro-ROS)
 ┌───────────────────────────────┐                   ┌──────────────────────────────┐
 │ robotfun_kinematics           │  /joint_command   │ robotfun_esp32_microros       │
 │  /target_pose ─► IK (DLS) ────┼───────────────────┼─► 6 servos (5 brazo+gripper)  │
 │  /joint_goal  ─► trapezoidal  │  Float32MultiArray │                              │
 │  fk_check ◄─ FK ◄─────────────┼───────────────────┼─◄ 6 potenciómetros            │
 │  (/fk_pose)                   │   /joint_states    │   publica a 25 Hz             │
 └───────────────────────────────┘   JointState       └──────────────────────────────┘
                 │
                 ▼
   robot_state_publisher + RViz  (robotfun_description: primitives | meshes)
```

## Modelo cinemático (DH estándar validado)
`L0=0.010 L1=0.063 L2=0.120 L3=0.090 L4=0.030 L5=0.090` (m). HOME = todas las
juntas a 0 rad ⇒ servos a 90° ⇒ `FK(HOME) = [0.010, 0, 0.393] m` (medido).

| i | d_i | θ_i | α_i | a_i |
|---|-----|-----|-----|-----|
| 1 | L1 | q1 | +90° | L0 |
| 2 | 0 | q2+90° | 0° | L2 |
| 3 | 0 | q3+90° | +90° | 0 |
| 4 | L3+L4 | q4 | +90° | 0 |
| 5 | 0 | q5+90° | 0° | L5 |

El URDF primitivo se genera con `robotfun_description/scripts/generate_dh_origins.py`
(verifica **TF == FK a 1e-12**). Dos modelos sin conflicto, misma interfaz de
juntas: `display.launch.py model:=primitives` (canónico) y `model:=meshes`
(piezas reales del CAD, para comparar/verificar).

## Decisiones técnicas clave

### 1) J4 fijo por defecto (4 GDL efectivos)
J4 es un **roll del antebrazo** (eje Z colineal). Su columna del Jacobiano de
posición es **0 en HOME** y pequeña cerca de él (mal-condiciona el DLS), y es la
DOF **menos útil para posicionar**: J1 (yaw) + J2,J3 (brazo planar) ya cubren la
posición 3D y J5 da el cabeceo de aproximación — suficiente para un pick & place
top-down. Para pastillas axisimétricas el roll de la pinza es irrelevante.

→ La IK **congela J4** por defecto (`use_joint4=false`, máscara `[T,T,T,F,T]`),
obteniendo una IK de 4 GDL bien condicionada y **sin deriva de q4**. La versión
**J4 activo** (5 GDL) queda disponible (`use_joint4=true`) para cuando la cámara
fije la orientación del objeto. El hardware siempre es 5 GDL + gripper; cambia
solo el modo de IK (y su gemelo `#define USE_JOINT4` en el firmware).

### 2) Radianes en el bus ROS, grados solo en el servo
ROS/RViz/MoveIt2 exigen radianes (REP-103, `sensor_msgs/JointState`). La
conversión rad→grados ocurre **únicamente** en `servo.write()` del firmware. La
**calibración** del firmware se expresa en grados/ADC porque es lo intuitivo del
hardware. No se exponen grados en los tópicos (evita ambigüedad con MoveIt2).

### 3) Control de posición con perfil trapezoidal
Los servos son de **posición** (no aceptan velocidad). En vez de un lazo de
velocidad, `trajectory_node` genera en lazo abierto un stream fino de setpoints
con perfil **trapezoidal** (acelera–crucero–desacelera) a 50 Hz; el servo
"persigue" la referencia y el movimiento sale suave. Modo articular y modo
cartesiano (control diferencial: `q += J⁺·dx`).

## Inconsistencia corregida respecto a `twin_ws`
En `twin_ws` el commit "cambio de DH" actualizó `dh_kinematics.py` pero **no**
regeneró `twin5dof.urdf.xacro` (quedó con la tabla DH anterior). Aquí el URDF se
**genera desde la tabla validada**, eliminando esa divergencia (TF == FK).
