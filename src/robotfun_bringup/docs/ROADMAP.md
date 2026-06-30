# Roadmap — hacia el pick & place guiado por cámara

El sistema actual (cinemática inversa + descripción + firmware) está diseñado
para crecer sin romper lo existente. Orden sugerido de ampliaciones.

## Fase 1 — Cinemática inversa (HECHO)
- Núcleo DH validado (FK, Jacobiano, IK DLS) con J4 fijo/activo.
- URDF DH-exacto (TF == FK) + modelo de meshes reales para verificación.
- Firmware ESP32 micro-ROS con las dos versiones de J4.
- Control suave (trapezoidal) y comprobación FK.

**Validar:** `colcon build` → `ros2 launch robotfun_bringup bringup.launch.py` →
`ros2 topic pub /target_pose ...` → ver que `/fk_pose` y RViz coinciden.

## Fase 2 — Gazebo (simulación física)
- Añadir `urdf/robotfun.ros2_control.xacro` (interfaces `position`) y
  `urdf/gazebo.xacro` (plugin `gz_ros2_control`) **como includes opcionales** del
  modelo primitivo (que ya trae `<collision>` e `<inertial>`).
- `robotfun_controller/` con `controllers.yaml`
  (`joint_trajectory_controller` para el brazo, `gripper` aparte) y
  `joint_state_broadcaster`.
- Beneficio: probar la IK y las trayectorias sin hardware, con física y colisiones.

## Fase 3 — MoveIt2 (planificación con colisiones)
- Generar el paquete `robotfun_moveit/` con el **MoveIt Setup Assistant** a partir
  del modelo **primitivo** (es el que tiene colisiones limpias y TF == FK).
- Grupos: `arm` (joint_1..joint_5) y `gripper`. Usar `config/joint_limits.yaml`.
- En modo pick&place, opcionalmente fijar `joint_4` (planificación de 4 GDL),
  coherente con la decisión de J4.
- Beneficio: planificación de trayectorias con evitación de colisiones y
  `MoveGroup` para pick & place.

## Fase 4 — Percepción (cámara: QR / color)
- Nuevo paquete `robotfun_perception/`:
  - Entrada: cámara (`image_raw`, `camera_info`); calibración mano-ojo
    (eye-to-hand) entre `camera_link` y `world`.
  - Detección de pastillas por **QR** (`zbar`/`opencv`) o por **color** (HSV);
    estimación de pose del objeto → `geometry_msgs/PoseStamped` en `world`.
  - Salida: publica la pose del objeto que consume la cinemática / MoveIt2.
- Integración: `perception → (pose objeto) → IK/MoveIt2 → trayectoria → firmware`.
- Aquí **J4 activo** cobra sentido: alinear la pinza con la orientación detectada.

## Fase 5 — Orquestación pick & place
- Máquina de estados (p. ej. `BehaviorTree.CPP` o un nodo de estados):
  `detectar → aproximar → bajar → cerrar gripper → subir → trasladar → soltar`.
- Servicios/acciones propias en `robotfun_msgs/` si se requieren tipos a medida.

## Principios para ampliar sin romper
- No tocar la **fuente de verdad** (tabla DH en `core/dh_model.py`): regenerar el
  URDF si cambia (`scripts/generate_dh_origins.py`).
- Mantener la **interfaz de juntas** (`joint_1..joint_5`, `gripper`) en todo nodo.
- Añadir capacidades como **paquetes nuevos** (Open/Closed), no modificando el
  núcleo. Adaptadores ROS delgados sobre lógica pura testeable.
