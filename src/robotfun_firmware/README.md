# robotfun_firmware — ESP32 (micro-ROS)

Bajo nivel del brazo de **4 GDL + gripper** (yaw + 3 pitch). El ESP32 mueve 5
servos y publica la realimentación de 5 potenciómetros.

> **Reestructuración:** el proyecto se reorganiza como si el robot SIEMPRE hubiese
> tenido **5 actuadores** `joint_1..joint_5`. El antiguo roll de muñeca desaparece
> por completo (sin servo muerto ni canal reservado); `joint_4` es el pitch de
> muñeca (antiguo joint_5) y `joint_5` es el **gripper** (antiguo gripper). Al
> quitar el roll, sus pines se reasignan: se **revive** el pot 35 (ahora joint_4),
> **continúa** el 27 (ahora gripper) y se **anula** el 26. En paralelo el servo
> revive el 18 (joint_4), continúa el 19 (gripper) y anula el 21.

## Interfaz ROS
| Tópico | Tipo | Sentido | Unidades |
|--------|------|---------|----------|
| `/joint_command` | `std_msgs/Float32MultiArray` | PC → ESP32 | **radianes** `[q1,q2,q3,q4, gripper]` |
| `/joint_states`  | `sensor_msgs/JointState`     | ESP32 → PC | **radianes**, 25 Hz, nombres `joint_1..joint_5` |

> **Radianes en el bus, grados solo en el servo.** ROS/RViz/MoveIt2 exigen
> radianes (REP-103). La conversión rad→grados ocurre **únicamente** en
> `servo.write()`. La **calibración** se expresa en grados/ADC (lo intuitivo del HW).

## Movimiento suave (perfil trapezoidal por junta)
El firmware **no** salta al objetivo cuando llega `/joint_command`: lo guarda como
objetivo y un **perfil trapezoidal por junta** (velocidad + aceleración limitadas)
lo alcanza en pasos de 20 ms (50 Hz, igual que el PWM del servo). Se **re-planifica
en cada tick**, así que admite objetivos nuevos a mitad de trayecto sin discontinuidades.
Ley por junta y tick:
```
v_stop = sqrt(2·a_max·|objetivo−pos|)      # vel con la que aún se frena a 0 en el objetivo
v_des  = signo(err)·min(v_max, v_stop)     # crucero o rampa de frenado
v     += clamp(v_des − v, ±a_max·dt)       # rampa de aceleración => arranque/frenado suave
pos   += v·dt                              # se escribe pos al servo
```
Ajusta por canal en el `.ino` (`rad/s`, `rad/s²`); menor `MAX_ACC` = más suave:
```cpp
const float MAX_VEL[NUM_CH] = { 1.5f, 1.5f, 1.5f, 1.5f, 3.0f };   // crucero
const float MAX_ACC[NUM_CH] = { 5.0f, 5.0f, 5.0f, 5.0f, 10.0f };  // aceleración
```
> Mantén estos límites **por encima** de los del `trajectory_node` (v_max=0.6,
> a_max=1.2) para que el firmware solo suavice saltos crudos y no frene la
> trayectoria ya planificada; y **por debajo** del máximo físico del servo.
> Verificado por simulación: vel/acel respetan el límite y no hay overshoot.

## Pines (5 canales) — reasignados tras quitar el roll
| Canal | Junta | Servo (PWM) | Pot (ADC) |
|------:|-------|-------------|-----------|
| 0 | joint_1 | GPIO 2  | GPIO 32 |
| 1 | joint_2 | GPIO 4  | GPIO 33 |
| 2 | joint_3 | GPIO 5  | GPIO 34 (solo IN) |
| 3 | joint_4 | GPIO 18 | GPIO 35 (solo IN) |
| 4 | joint_5 (gripper) | GPIO 19 | GPIO 27 (ADC2) |

Diseño original de 6 canales (referencia del cambio): `SERVO={2,4,5,18,19,21}`,
`POT={32,33,34,35,27,26}`. Se elimina el roll (idx 3) y todo se desplaza: el pot
**revive 35** (joint_4), **continúa 27** (gripper) y **anula 26**; el servo revive
18, continúa 19 y anula 21. LED de estado en GPIO 13.

> El usuario fijó explícitamente los **pines de pot** `{32,33,34,35,27}`. Los de
> servo `{2,4,5,18,19}` siguen el mismo criterio; si tu cableado físico de servos
> no cambió (siguen en `{2,4,5,19,21}`), ajústalo en `SERVO_PINS[]` del `.ino`.

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
