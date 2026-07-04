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

## Movimiento FLUIDO (tarea de tiempo real dedicada)
El firmware **no** salta al objetivo cuando llega `/joint_command`: lo guarda y un
**perfil trapezoidal por junta** (arranca/frena gradual) lo alcanza suave. Ese
perfil corre en una **tarea FreeRTOS con temporización exacta** (50 Hz,
`vTaskDelayUntil`) y **prioridad mayor que `loop()`**, así **preempta** al bucle
cada 20 ms y queda **inmune al jitter de micro-ROS** → movimiento **fluido** (no
"a pasitos"). Resolución fina en µs y **detección de cruce** del objetivo (0
overshoot, sin jitter). Ajusta velocidad/suavidad por junta (**en grados**):
```cpp
const float MAX_VEL[NUM_CH] = {  70,  70,  70,  70, 120 };   // grados/s (crucero)
const float MAX_ACC[NUM_CH] = { 150, 150, 150, 150, 300 };   // grados/s² (suavidad)
```
- Más **fluido/lento** → baja ambos (p. ej. `MAX_VEL=45`, `MAX_ACC=90`).
- Más **rápido** → sube `MAX_VEL`; un `MAX_ACC` bajo = arranque/frenado más sedoso.

### ¿Se resetea el ESP32?
La tarea de servo necesita **pila suficiente**: se crea con **8192 bytes**. Con
4096 se desbordaba (float `sqrtf` + librería de servos) → *"Stack canary watchpoint
triggered (servo_task)"* → **reset**. Va en el **núcleo 1** (deja el 0 libre para el
sistema) y cede CPU con `vTaskDelayUntil` (no dispara el watchdog). Para ver el
motivo de un reset: abre el **Monitor Serie a 115200 SIN el agente** y lee el
mensaje al arrancar (`Stack canary…` = pila; `Brownout detector…` = fuente de
servos débil → usa **fuente externa 5–6 V**, **GND común**, **cap 1000 µF+**).

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

## Nota anti-bucle (si alguna versión "rebota" a HOME)
El firmware es **lazo abierto**: llega al objetivo y se queda; no puede rebotar
solo. Si lo ves ir al ángulo y **volver a HOME en bucle**, el **ESP32 se está
reiniciando** (en cada reset, `setup()` recoloca los servos). Causas y fix:
- **Pila de la tarea de servo insuficiente** → *Stack canary* → reset. Ya se crea
  con **8192 bytes** (ver arriba). No la bajes.
- **Watchdog por no ceder CPU**: `loop()` DEBE terminar con `delay(1)` (ya está).
- **Brownout** (caída de tensión al mover los servos): **fuente externa 5–6 V**
  (no desde el ESP32/USB), **GND común**, **condensador 1000 µF+** cerca de los servos.
Y al mandar un ángulo, usa `--once` y comprueba `ros2 topic info /joint_command
--verbose` → **1 solo publicador**.

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
