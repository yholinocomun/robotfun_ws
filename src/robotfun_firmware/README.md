# robotfun_firmware — ESP32 (micro-ROS)

Bajo nivel del brazo de **4 GDL + gripper** (yaw + 3 pitch). El ESP32 mueve 5
servos y publica la realimentación de 5 potenciómetros.

> **Cambio importante:** el joint_4 de **roll se eliminó**. El antiguo joint_5
> (pitch de muñeca) es ahora **joint_4**. El servo del roll, si sigue montado, se
> deja **fijo a 90°** (muerto) — pin `DEAD_ROLL_PIN` (18); ponlo en `-1` si ya no está.

## Interfaz ROS
| Tópico | Tipo | Sentido | Unidades |
|--------|------|---------|----------|
| `/joint_command` | `std_msgs/Float32MultiArray` | PC → ESP32 | **radianes** `[q1,q2,q3,q4, gripper]` |
| `/joint_states`  | `sensor_msgs/JointState`     | ESP32 → PC | **radianes**, 25 Hz |

> **Radianes en el bus, grados solo en el servo.** ROS/RViz/MoveIt2 exigen
> radianes (REP-103). La conversión rad→grados ocurre **únicamente** en
> `servo.write()`. La **calibración** se expresa en grados/ADC (lo intuitivo del HW).

## Pines (5 canales)
| Canal | Junta | Servo (PWM) | Pot (ADC) |
|------:|-------|-------------|-----------|
| 0 | joint_1 | GPIO 2  | GPIO 32 |
| 1 | joint_2 | GPIO 4  | GPIO 33 |
| 2 | joint_3 | GPIO 5  | GPIO 34 (solo IN) |
| 3 | joint_4 | GPIO 19 | GPIO 27 (ADC2) |
| 4 | gripper | GPIO 21 | GPIO 26 (ADC2) |

Roll eliminado: servo en GPIO 18 fijo a 90°. LED de estado en GPIO 13.

## Calibración
1. Coloca el robot en **HOME** (todas las juntas a 0°, brazo vertical).
2. Lee el ADC y copia los valores en `RAW_ZERO[5]`.
3. Si una junta aparece **invertida en RViz** → cambia su signo en `FEEDBACK_DIRECTION[5]`.
4. Si un servo gira **al revés respecto a la cinemática** → signo en `SERVO_DIRECTION[5]`.
5. Ajusta `JOINT_MIN_DEG/JOINT_MAX_DEG` por canal (el gripper usa su propio rango).

## Compilar / subir (Arduino IDE)
- Placa: ESP32. Librerías: `micro_ros_arduino`, `ESP32Servo`.
- Sube el `.ino` y arranca el Agent:
```bash
ros2 launch robotfun_firmware microros_agent.launch.py dev:=/dev/ttyUSB0 baud:=115200
```
