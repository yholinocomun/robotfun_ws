# robotfun_firmware — ESP32 (micro-ROS)

Bajo nivel del brazo de **5 GDL + gripper**. El ESP32 mueve 6 servos y publica la
realimentación de 6 potenciómetros.

## Interfaz ROS
| Tópico | Tipo | Sentido | Unidades |
|--------|------|---------|----------|
| `/joint_command` | `std_msgs/Float32MultiArray` | PC → ESP32 | **radianes** `[q1..q5, gripper]` |
| `/joint_states`  | `sensor_msgs/JointState`     | ESP32 → PC | **radianes**, 25 Hz |

> **Radianes en el bus, grados solo al final.** ROS/RViz/MoveIt2 exigen radianes
> (REP-103). La conversión rad→grados de servo ocurre **únicamente** en
> `servo.write()`. La **calibración** (RAW_ZERO, rangos) se expresa en grados/ADC
> porque es lo intuitivo del hardware.

## Dos versiones en un solo archivo (cambia UNA línea)
```c
#define USE_JOINT4 1   // J4 ACTIVO  : 5 GDL (tareas de orientación, futuro/cámara)
#define USE_JOINT4 0   // J4 ANULADO : J4 fijo en HOME (90°, q4=0) → 4 GDL pick&place
```
Su gemelo en alto nivel es el parámetro `use_joint4` de `robotfun_kinematics`.
**Recomendado para empezar: `USE_JOINT4 0`** (pick & place de pastillas).

## Pines fijos
| Canal | Junta | Servo (PWM) | Pot (ADC) |
|------:|-------|-------------|-----------|
| 0 | joint_1 | GPIO 2  | GPIO 32 |
| 1 | joint_2 | GPIO 4  | GPIO 33 |
| 2 | joint_3 | GPIO 5  | GPIO 34 (solo IN) |
| 3 | joint_4 | GPIO 18 | GPIO 35 (solo IN) |
| 4 | joint_5 | GPIO 19 | GPIO 27 (ADC2) |
| 5 | gripper | GPIO 21 | GPIO 26 (ADC2) |

LED de estado en GPIO 13 (GPIO 2 es el servo de j1). ADC2 (26/27) funciona porque
micro-ROS usa serial, no WiFi.

## Calibración
1. Coloca el robot en **HOME** (todas las juntas a 0°, "L invertida").
2. Lee el ADC y copia los valores en `RAW_ZERO[6]`.
3. Si una junta aparece **invertida en RViz** → cambia su signo en `FEEDBACK_DIRECTION[6]`.
4. Si un servo gira **al revés respecto a la DH** → cambia su signo en `SERVO_DIRECTION[6]`.
5. Ajusta `JOINT_MIN_DEG/JOINT_MAX_DEG` por canal (el gripper usa su propio rango).

## Compilar / subir (Arduino IDE)
- Placa: ESP32. Librerías: `micro_ros_arduino`, `ESP32Servo`.
- Sube el `.ino` y luego arranca el Agent:
```bash
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0 baud:=115200
```
