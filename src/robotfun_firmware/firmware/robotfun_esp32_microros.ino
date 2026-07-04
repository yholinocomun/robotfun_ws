/* ============================================================================
 *  robotfun_esp32_microros.ino
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 4 GDL (yaw + 3 pitch) + GRIPPER.
 *
 *  Base = tu firmware micro-ROS que FUNCIONA (va al angulo y NO vuelve a HOME).
 *  Unico anadido: MOVIMIENTO SUAVE. El comando ya NO salta de golpe al servo; se
 *  guarda como OBJETIVO y un perfil TRAPEZOIDAL por junta (arranca lento -> crucero
 *  -> frena lento) lo alcanza en pasos de 20 ms. Sin memoria RTC ni nada extra:
 *  se conserva TODO lo de la version que te funcionaba (setup, executor, feedback,
 *  y sobre todo el delay(1) del loop, que evita reset por watchdog).
 *
 *  Reestructuracion previa (se mantiene): 5 actuadores joint_1..joint_5, sin el
 *  antiguo roll. joint_4 = pitch de muñeca (antiguo joint_5); joint_5 = GRIPPER.
 *
 *  Pipeline:
 *    /joint_command (std_msgs/Float32MultiArray, RADIANES [q1,q2,q3,q4, gripper])
 *        --> fija el OBJETIVO de cada junta
 *    perfil trapezoidal por junta @50 Hz --> mueve los 5 servos SUAVEMENTE
 *    5 potenciometros --> /joint_states (sensor_msgs/JointState, RADIANES) @25 Hz
 *    joint_states.name = { joint_1, joint_2, joint_3, joint_4, joint_5 }
 *
 *  UNIDADES: el bus ROS va en RADIANES (REP-103, JointState). La conversion a
 *  GRADOS de servo ocurre solo en servo.write(). La calibracion se expresa en
 *  grados/ADC (intuitivo del hardware).
 *
 *  HOME: todas las juntas a 0 rad => servos a 90 grados (brazo vertical).
 *
 *  PINES (5 canales: 4 brazo + gripper) — REASIGNADOS tras quitar el roll:
 *      idx :   0     1     2       3          4
 *      junta:  j1    j2    j3    j4(pitch)  j5(gripper)
 *      SERVO:  2     4     5     18         19
 *      POT  :  32    33    34    35         27       (34 solo IN; 27 ADC2 OK con serial)
 *  NOTA: si tu cableado de servos no cambio (siguen en {2,4,5,19,21}), ajusta SERVO_PINS[].
 *      LED de estado: GPIO 13 (GPIO 2 es el servo j1).
 * ==========================================================================*/

#include <math.h>

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

// Pines re-asignados (ver cabecera). El roll ya no existe: su pin de servo (18) y
// de pot (35) pasan a ser los de joint_4.
const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 18, 19 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 35, 27 };

const char *JOINT_LABEL[NUM_CH] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" };

// ===========================================================================
//  CALIBRACION (grados / cuentas ADC). Se CONSERVA tal cual la version que funciona.
// ===========================================================================
// RAW_ZERO[i]: ADC (0..4095) de cada pot en HOME (todas las juntas a 0 rad).
int RAW_ZERO[NUM_CH] = {
  1267,   // joint_1
  1232,   // joint_2
  1265,   // joint_3
  1405,   // joint_4  (pitch de muñeca; antiguo joint_5)
  1302    // joint_5  (gripper)
};

// FEEDBACK_DIRECTION[i]: sentido del pot hacia RViz (-1 si sale invertido).
const float FEEDBACK_DIRECTION[NUM_CH] = { 1, 1, -1, -1, 1 };

// SERVO_DIRECTION[i]: sentido del servo respecto al comando. CALIBRAR.
const float SERVO_DIRECTION[NUM_CH] = { 1, -1, 1, -1, 1 };

// ===========================================================================
//  RANGO Y CENTRO POR CANAL  ->  define el ESPACIO DE TRABAJO
// idx:                                   j1    j2    j3    j4   j5(grip)
const float SERVO_CENTER_DEG[NUM_CH] = {  90,   90,   90,   90,   90 };
const float JOINT_MIN_DEG[NUM_CH]    = { -90,  -90,  -90,  -90,    0 };
const float JOINT_MAX_DEG[NUM_CH]    = {  90,   90,   90,   90,   70 };

const float ADC_TO_DEG = 180.0f / 4095.0f;     // pot de 180° sobre 0..4095
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float ALPHA = 0.15f;                      // filtro exponencial del ADC

// ===========================================================================
//  MOVIMIENTO SUAVE  ->  perfil trapezoidal por junta (vel + acel limitadas)
// ---------------------------------------------------------------------------
// Corre a 50 Hz (paso de 20 ms, igual que el PWM del servo). Limites POR JUNTA:
//   MAX_VEL[i] rad/s   -> velocidad de crucero (mas bajo = MAS LENTO / suave)
//   MAX_ACC[i] rad/s^2 -> aceleracion (mas bajo = arranque/frenado MAS suave)
// Valores por defecto LENTOS y suaves. Referencia: 1.0 rad/s ~= 57 grados/s.
// >>> Si lo quieres AUN mas lento, baja MAX_VEL (p. ej. 0.6). Si demasiado lento,
//     subelo (p. ej. 1.5). El gripper (joint_5) va un poco mas agil.
#define SERVO_UPDATE_MS 20
// idx:                          j1     j2     j3     j4    j5(grip)
const float MAX_VEL[NUM_CH] = { 0.9f,  0.9f,  0.9f,  0.9f,  1.8f };   // rad/s
const float MAX_ACC[NUM_CH] = { 2.5f,  2.5f,  2.5f,  2.5f,  5.0f };   // rad/s^2
const float POS_EPS = 0.0035f;   // ~0.2 grados (umbral de "llegada")
const float VEL_EPS = 0.02f;     // rad/s

// Estado del perfil (rad, rad/s). target_pos lo fija /joint_command; cur_pos es lo
// que se escribe al servo; cur_vel es la velocidad del perfil.
float target_pos[NUM_CH];
float cur_pos[NUM_CH];
float cur_vel[NUM_CH];
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

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ADC -> radianes. 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// radianes -> grados de servo (0..180). 0 rad => SERVO_CENTER_DEG[i] (= HOME).
int rad_to_servo_deg(float q_rad, int i) {
  float q_deg = clampf(q_rad * RAD2DEG, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  float servo_deg = SERVO_CENTER_DEG[i] + SERVO_DIRECTION[i] * q_deg;
  return (int)clampf(servo_deg, 0.0f, 180.0f);
}

void apply_servo_commands(const float *q_cmd) {
  for (int i = 0; i < NUM_CH; i++) servos[i].write(rad_to_servo_deg(q_cmd[i], i));
}

// Un paso del perfil trapezoidal por junta. Solo escribe el servo mientras se
// mueve (como tu sketch de referencia); al llegar hace un ultimo write y para.
void update_motion(float dt) {
  for (int i = 0; i < NUM_CH; i++) {
    float err = target_pos[i] - cur_pos[i];
    // Ya llego: fija exacto, para y NO re-escribe (evita zumbido y ahorra CPU).
    if (fabsf(err) < POS_EPS && fabsf(cur_vel[i]) < VEL_EPS) {
      if (cur_pos[i] != target_pos[i]) {
        cur_pos[i] = target_pos[i];
        servos[i].write(rad_to_servo_deg(cur_pos[i], i));
      }
      cur_vel[i] = 0.0f;
      continue;
    }
    // Velocidad maxima con la que AUN puedo frenar hasta 0 justo en el objetivo:
    float v_stop = sqrtf(2.0f * MAX_ACC[i] * fabsf(err));
    float v_des  = (err >= 0.0f ? 1.0f : -1.0f) * fminf(MAX_VEL[i], v_stop);
    // Rampa de aceleracion: limita el cambio de velocidad por tick (=> suave).
    float dv = v_des - cur_vel[i];
    float dv_max = MAX_ACC[i] * dt;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    cur_vel[i] += dv;
    cur_pos[i] += cur_vel[i] * dt;
    servos[i].write(rad_to_servo_deg(cur_pos[i], i));
  }
}

// Callback de /joint_command (RADIANES, [q1..q4, gripper]; acepta >=4).
// Solo fija el OBJETIVO; el perfil suave (update_motion) lo alcanza sin saltos.
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;     // se necesitan al menos las 4 juntas del brazo
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      float lo = JOINT_MIN_DEG[i] * DEG2RAD, hi = JOINT_MAX_DEG[i] * DEG2RAD;
      target_pos[i] = clampf(msg->data.data[i], lo, hi);
    }   // si no llega el gripper (size==4) conserva su objetivo anterior
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
    target_pos[i] = cur_pos[i] = cur_vel[i] = 0.0f;   // arranca en HOME, quieto
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
  }

  float q_home[NUM_CH] = {0, 0, 0, 0, 0};
  apply_servo_commands(q_home);
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

  last_motion_ms = millis();          // arranca el reloj del perfil de movimiento
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  // Atiende comandos entrantes.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  // Avanza el perfil suave cada 20 ms (dt real => robusto a jitter).
  unsigned long now = millis();
  if (now - last_motion_ms >= SERVO_UPDATE_MS) {
    float dt = (now - last_motion_ms) * 0.001f;
    last_motion_ms = now;
    update_motion(dt);
  }
  delay(1);   // CEDE CPU (evita inanicion del IDLE / reset por watchdog). NO quitar.
}
