/* ============================================================================
 *  robotfun_esp32_microros.ino
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 4 GDL (yaw + 3 pitch) + GRIPPER.
 *
 *  SUAVIZADO = tu sketch "robotfun_ultra_ligero_veloz" (rampa lineal con
 *  MATEMATICA DE ENTEROS y writeMicroseconds), portado a micro-ROS. Es el metodo
 *  que ya te da movimiento suave y NO hace bucle. Solo se cambia la ENTRADA:
 *  antes venia por Serial (grados); ahora viene por /joint_command en RADIANES.
 *  Se conserva el feedback de potenciometros hacia /joint_states (para RViz) y
 *  toda tu calibracion.
 *
 *  Por que este NO entra en bucle:
 *   - Es lazo abierto: mueve el servo hacia el objetivo por pasos y se queda ahi.
 *   - loop() termina en delay(1): cede CPU y evita reset por watchdog (NO quitar).
 *   - No hay memoria RTC ni logica extra que pueda re-homear.
 *   Si AUN asi rebota a HOME, el ESP32 se esta RESETEANDO por HARDWARE
 *   (brownout: fuente de servos debil). Ver README (fuente externa 5-6V, GND
 *   comun, condensador 1000uF). Y al mandar el angulo usa --once y comprueba
 *   'ros2 topic info /joint_command --verbose' -> 1 solo publicador.
 *
 *  Estructura de juntas: 5 actuadores joint_1..joint_5 (sin el antiguo roll).
 *  joint_4 = pitch de muñeca (antiguo joint_5); joint_5 = GRIPPER.
 *
 *  Pipeline:
 *    /joint_command (Float32MultiArray, RADIANES [q1,q2,q3,q4, gripper])
 *        --> objetivo por junta (en "centigrados" enteros)
 *    rampa lineal @50 Hz (writeMicroseconds) --> mueve los 5 servos SUAVE
 *    5 potenciometros --> /joint_states (JointState, RADIANES) @25 Hz
 *    joint_states.name = { joint_1, joint_2, joint_3, joint_4, joint_5 }
 *
 *  PINES (5 canales) — reasignados tras quitar el roll:
 *      idx :   0     1     2       3          4
 *      junta:  j1    j2    j3    j4(pitch)  j5(gripper)
 *      SERVO:  2     4     5     18         19
 *      POT  :  32    33    34    35         27
 *  Si tu cableado de servos no cambio (siguen en {2,4,5,19,21}), ajusta SERVO_PINS[].
 *      LED de estado: GPIO 13.
 * ==========================================================================*/

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#define STATUS_LED_PIN 13
#define NUM_CH         5          // joint_1..joint_4 (brazo) + joint_5 (gripper)
#define GRIPPER_INDEX  4          // joint_5 = gripper

const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 18, 19 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 35, 27 };

const char *JOINT_LABEL[NUM_CH] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" };

// ===========================================================================
//  MAPEO DE SERVO (grados)  ->  igual que tu sketch de referencia
// ---------------------------------------------------------------------------
// servo_deg = SERVO_CENTER_DEG + SERVO_DIRECTION*q_deg ; saturado a [0,180].
// SERVO_DIRECTION: sentido de giro. Tu firmware micro-ROS confirmado usa
//   {1,-1,1,-1,1}. (Tu sketch de calibracion tenia el gripper en -1: si el
//   gripper gira al reves, cambia SOLO SERVO_DIRECTION[4] a -1.)
const int SERVO_DIRECTION[NUM_CH]  = { 1, -1, 1, -1, 1 };
const int SERVO_CENTER_DEG[NUM_CH] = { 90, 90, 90, 90, 90 };
const int JOINT_MIN_DEG[NUM_CH]    = { -90, -90, -90, -90,  0 };
const int JOINT_MAX_DEG[NUM_CH]    = {  90,  90,  90,  90, 70 };

// ===========================================================================
//  FEEDBACK de potenciometros (para /joint_states). TU CALIBRACION, sin cambios.
// ===========================================================================
int RAW_ZERO[NUM_CH] = {
  1267,   // joint_1
  1232,   // joint_2
  1265,   // joint_3
  1405,   // joint_4  (pitch de muñeca; antiguo joint_5)
  1302    // joint_5  (gripper)
};
const float FEEDBACK_DIRECTION[NUM_CH] = { 1, 1, -1, -1, 1 };

const float ADC_TO_DEG = 180.0f / 4095.0f;
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float ALPHA = 0.15f;                      // filtro exponencial del ADC

// ===========================================================================
//  MOVIMIENTO SUAVE  ->  rampa lineal con ENTEROS (tu metodo de referencia)
// ---------------------------------------------------------------------------
// Se trabaja en "centigrados" (grados*100) para usar enteros rapidos.
// Cada 20 ms cada junta avanza como MUCHO PASO_VELOCIDAD centigrados hacia su
// objetivo. Es la SUAVIDAD/velocidad:
//    PASO_VELOCIDAD = 100  -> 1.0 grados/tick  = 50 grados/s   (suave)
//    PASO_VELOCIDAD =  60  -> 0.6 grados/tick  = 30 grados/s   (MAS suave/lento)
//    PASO_VELOCIDAD = 250  -> 2.5 grados/tick  = 125 grados/s  (rapido, como tu ref.)
// Mas bajo = mas suave y mas lento. Subelo si lo quieres mas rapido.
#define SERVO_UPDATE_MS 20
const long PASO_VELOCIDAD = 100;                // centigrados por tick de 20 ms

// Estado del movimiento (centigrados de la junta q; 0 = HOME).
long pos_actual_scaled[NUM_CH] = {0, 0, 0, 0, 0};
long pos_target_scaled[NUM_CH] = {0, 0, 0, 0, 0};
unsigned long last_motion_ms = 0;

// ===========================================================================
Servo servos[NUM_CH];
float feedback_filtered[NUM_CH];
bool  filter_initialized = false;

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t joint_state_pub;
rcl_subscription_t joint_command_sub;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray command_msg;

static char joint_name_buf[NUM_CH][12] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" };
static rosidl_runtime_c__String joint_names[NUM_CH];
static double position_data[NUM_CH];
static double velocity_data[NUM_CH];
static double effort_data[NUM_CH];
static float  command_data[NUM_CH];

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) { digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(100); }
}

long  clampi(long x, long lo, long hi)   { return x < lo ? lo : (x > hi ? hi : x); }
float clampf(float x, float lo, float hi){ return x < lo ? lo : (x > hi ? hi : x); }

// Centigrados de la junta q -> microsegundos PWM (0..180 deg -> 500..2400 us),
// con precision de centigrado (mas fino => mas suave que redondear a grados).
int scaled_deg_to_pwm_us(long q_scaled, int i) {
  q_scaled = clampi(q_scaled, (long)JOINT_MIN_DEG[i] * 100, (long)JOINT_MAX_DEG[i] * 100);
  long servo_scaled = (long)SERVO_CENTER_DEG[i] * 100 + SERVO_DIRECTION[i] * q_scaled; // centigrados
  servo_scaled = clampi(servo_scaled, 0, 18000);                                       // 0..180.00 deg
  return (int)(500 + (servo_scaled * 1055) / 10000);                                   // 500..2400 us
}

// ADC -> radianes (feedback). 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, (float)JOINT_MIN_DEG[i], (float)JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// Callback de /joint_command (RADIANES [q1..q4, gripper]; acepta >=4).
// Solo fija el OBJETIVO (en centigrados); la rampa lo alcanza suave.
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      long q_scaled = (long)(msg->data.data[i] * RAD2DEG * 100.0f);   // rad -> centigrados
      pos_target_scaled[i] = clampi(q_scaled, (long)JOINT_MIN_DEG[i] * 100,
                                              (long)JOINT_MAX_DEG[i] * 100);
    }   // si no llega el gripper (size==4) conserva su objetivo anterior
  }
}

// Rampa lineal por junta (tu metodo). Solo escribe el servo si se mueve.
void update_servos() {
  for (int i = 0; i < NUM_CH; i++) {
    long diff = pos_target_scaled[i] - pos_actual_scaled[i];
    if (diff == 0) continue;                    // ya llego: no re-escribe (ahorra CPU, sin zumbido)
    long ad = diff < 0 ? -diff : diff;
    if (ad <= PASO_VELOCIDAD) pos_actual_scaled[i] = pos_target_scaled[i];  // ultimo tramo
    else if (diff > 0)        pos_actual_scaled[i] += PASO_VELOCIDAD;
    else                      pos_actual_scaled[i] -= PASO_VELOCIDAD;
    servos[i].writeMicroseconds(scaled_deg_to_pwm_us(pos_actual_scaled[i], i));
  }
}

void read_feedback() {
  float measured[NUM_CH];
  for (int i = 0; i < NUM_CH; i++) measured[i] = adc_to_rad(analogRead(POT_PINS[i]), i);
  if (!filter_initialized) {
    for (int i = 0; i < NUM_CH; i++) feedback_filtered[i] = measured[i];
    filter_initialized = true;
  } else {
    for (int i = 0; i < NUM_CH; i++)
      feedback_filtered[i] = ALPHA * measured[i] + (1.0f - ALPHA) * feedback_filtered[i];
  }
  for (int i = 0; i < NUM_CH; i++) position_data[i] = feedback_filtered[i];
}

void timer_callback(rcl_timer_t *t, int64_t last) {
  (void)last;
  if (t == NULL) return;
  read_feedback();
  int64_t ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
  joint_state_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

void setup_joint_state_msg() {
  for (int i = 0; i < NUM_CH; i++) {
    joint_names[i].data = joint_name_buf[i];
    joint_names[i].size = strlen(joint_name_buf[i]);
    joint_names[i].capacity = strlen(joint_name_buf[i]) + 1;
  }
  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = NUM_CH; joint_state_msg.name.capacity = NUM_CH;
  joint_state_msg.position.data = position_data;
  joint_state_msg.position.size = NUM_CH; joint_state_msg.position.capacity = NUM_CH;
  joint_state_msg.velocity.data = velocity_data;
  joint_state_msg.velocity.size = NUM_CH; joint_state_msg.velocity.capacity = NUM_CH;
  joint_state_msg.effort.data = effort_data;
  joint_state_msg.effort.size = NUM_CH; joint_state_msg.effort.capacity = NUM_CH;
  joint_state_msg.header.frame_id.data = (char *)"";
  joint_state_msg.header.frame_id.size = 0; joint_state_msg.header.frame_id.capacity = 1;
}

void setup_command_msg() {
  command_msg.data.data = command_data;
  command_msg.data.size = NUM_CH; command_msg.data.capacity = NUM_CH;
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  for (int i = 0; i < NUM_CH; i++) {
    position_data[i] = velocity_data[i] = effort_data[i] = 0.0;
    command_data[i] = 0.0f; feedback_filtered[i] = 0.0f;
    pos_actual_scaled[i] = pos_target_scaled[i] = 0;   // arranca en HOME, quieto
  }

  Serial.begin(115200);
  set_microros_transports();
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(500);
  }
  digitalWrite(STATUS_LED_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_CH; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
    servos[i].writeMicroseconds(scaled_deg_to_pwm_us(0, i));   // HOME
  }
  delay(1000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robotfun_esp32_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
      &joint_state_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states"));
  RCCHECK(rclc_subscription_init_default(
      &joint_command_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/joint_command"));

  setup_joint_state_msg();
  setup_command_msg();
  rmw_uros_sync_session(1000);

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(40), timer_callback)); // 25 Hz
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &joint_command_sub, &command_msg, &command_callback, ON_NEW_DATA));

  last_motion_ms = millis();
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));   // atiende comandos
  unsigned long now = millis();
  if (now - last_motion_ms >= SERVO_UPDATE_MS) {                      // rampa suave @50 Hz
    last_motion_ms = now;
    update_servos();
  }
  delay(1);   // CEDE CPU (evita reset por watchdog). NO quitar.
}
