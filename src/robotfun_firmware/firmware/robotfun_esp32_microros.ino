/* ============================================================================
 *  robotfun_esp32_microros.ino
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 5 GDL (RRRRR) + GRIPPER.
 *
 *  Pipeline:
 *    /joint_command (std_msgs/Float32MultiArray, RADIANES [q1..q5, gripper])
 *        --> mueve 6 servos
 *    6 potenciometros --> /joint_states (sensor_msgs/JointState, RADIANES) @25 Hz
 *
 *  DOS VERSIONES EN UN SOLO ARCHIVO (DRY) — cambia una linea:
 *      #define USE_JOINT4 1   -> J4 ACTIVO  : 5 GDL completos (orientacion).
 *      #define USE_JOINT4 0   -> J4 ANULADO : el servo de J4 se mantiene en HOME
 *                                             (90 deg, q4=0). Coincide con la IK
 *                                             de 4 GDL para pick & place.
 *  El alto nivel (robotfun_kinematics) tiene su gemelo: el parametro `use_joint4`.
 *
 *  UNIDADES — por que RADIANES en el bus y GRADOS solo al final:
 *      ROS/RViz/MoveIt2 trabajan en radianes (REP-103, sensor_msgs/JointState).
 *      El servo fisico se manda en GRADOS (0..180). Por eso el bus ROS es en
 *      radianes y la conversion rad->grados ocurre UNICAMENTE en servo.write().
 *      La CALIBRACION se expresa en grados (intuitiva para el hardware).
 *
 *  HOME del robot: TODAS las juntas a 0 rad  =>  todos los servos a 90 grados.
 *  El 0 rad de cada junta coincide con el HOME de la convencion DH del alto nivel.
 *
 *  PINES FIJOS (no cambiar):
 *      POT_PINS[6]   = {32, 33, 34, 35, 27, 26}   // j1 j2 j3 j4 j5 gripper
 *      SERVO_PINS[6] = { 2,  4,  5, 18, 19, 21}   // j1 j2 j3 j4 j5 gripper
 *      GPIO 34/35 son solo-entrada (ADC ideales). GPIO 26/27 (ADC2) funcionan
 *      porque micro-ROS usa SERIAL (no WiFi). El LED de estado va a GPIO 13
 *      porque GPIO 2 es el servo de j1.
 * ==========================================================================*/

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

// ===========================================================================
//  CONFIGURACION DE VERSION
// ===========================================================================
#define USE_JOINT4 1     // 1 = J4 activo (5 GDL) ; 0 = J4 anulado (home, 4 GDL)

// ===========================================================================
//  CONSTANTES DE HARDWARE
// ===========================================================================
#define STATUS_LED_PIN 13
#define NUM_JOINTS     6          // 5 del brazo + 1 gripper
#define J4_INDEX       3          // indice de joint_4 (eje saliente / roll)
#define GRIPPER_INDEX  5

const int SERVO_PINS[NUM_JOINTS] = {  2,  4,  5, 18, 19, 21 };
const int POT_PINS[NUM_JOINTS]   = { 32, 33, 34, 35, 27, 26 };

const char *JOINT_LABEL[NUM_JOINTS] = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"
};

// ===========================================================================
//  CALIBRACION (en GRADOS / cuentas ADC: lo intuitivo del hardware)
// ===========================================================================
// RAW_ZERO[i]: lectura ADC (0..4095) de cada pot cuando el robot esta en HOME
//   (todas las juntas a 0 rad). Coloca el robot en HOME y copia aqui las lecturas.
int RAW_ZERO[NUM_JOINTS] = {
  1194,   // joint_1
  1165,   // joint_2
  1191,   // joint_3
  1415,   // joint_4
  1423,   // joint_5
  1300    // gripper
};

// FEEDBACK_DIRECTION[i]: sentido del pot hacia RViz (-1 si sale invertido).
const float FEEDBACK_DIRECTION[NUM_JOINTS] = { 1, 1, 1, 1, 1, 1 };

// SERVO_DIRECTION[i]: sentido del servo respecto al comando (+q en el sentido
//   +Z de la DH). Cambia 1 por -1 si un servo gira al reves respecto a la DH.
const float SERVO_DIRECTION[NUM_JOINTS] = { -1, -1, 1, 1, 1, -1 };

// Rango articular por canal (GRADOS). El brazo va ±90; el gripper 0..70 (cierra
//   /abre). Centro del servo = 90 deg = HOME.
const float JOINT_MIN_DEG[NUM_JOINTS] = { -90, -90, -90, -90, -90,   0 };
const float JOINT_MAX_DEG[NUM_JOINTS] = {  90,  90,  90,  90,  90,  70 };
const float SERVO_CENTER_DEG = 90.0f;

// Conversion ADC(12 bits) -> grados:  pot de 180 deg sobre 0..4095.
const float ADC_TO_DEG = 180.0f / 4095.0f;
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;

// Filtro exponencial de la realimentacion ADC (0<alpha<=1; menor = mas suave).
const float ALPHA = 0.15f;

// ===========================================================================
//  ESTADO
// ===========================================================================
Servo servos[NUM_JOINTS];
float feedback_filtered[NUM_JOINTS];   // rad, filtrado
bool  filter_initialized = false;

// micro-ROS
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_publisher_t joint_state_pub;
rcl_subscription_t joint_command_sub;
rcl_timer_t timer;

sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray command_msg;

static char joint_name_buf[NUM_JOINTS][12] = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"
};
static rosidl_runtime_c__String joint_names[NUM_JOINTS];
static double position_data[NUM_JOINTS];
static double velocity_data[NUM_JOINTS];
static double effort_data[NUM_JOINTS];
static float  command_data[NUM_JOINTS];   // ultimo comando recibido (rad)

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) { digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(100); }
}

// ---------------------------------------------------------------------------
//  Utilidades
// ---------------------------------------------------------------------------
float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// ADC -> radianes (realimentacion). 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// radianes -> grados de servo (0..180). 0 rad => 90 deg (centro = HOME).
int rad_to_servo_deg(float q_rad, int i) {
  float q_deg = clampf(q_rad * RAD2DEG, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  float servo_deg = SERVO_CENTER_DEG + SERVO_DIRECTION[i] * q_deg;
  return (int)clampf(servo_deg, 0.0f, 180.0f);
}

// Aplica los comandos a los 6 servos (respeta la version de J4).
void apply_servo_commands(const float *q_cmd) {
  for (int i = 0; i < NUM_JOINTS; i++) {
#if (USE_JOINT4 == 0)
    float q = (i == J4_INDEX) ? 0.0f : q_cmd[i];   // J4 anulado: forzado a HOME
#else
    float q = q_cmd[i];
#endif
    servos[i].write(rad_to_servo_deg(q, i));
  }
}

// ---------------------------------------------------------------------------
//  Callback de /joint_command  (RADIANES, [q1..q5, gripper]; acepta >=5)
// ---------------------------------------------------------------------------
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  if (msg->data.size < 5) return;   // se necesitan al menos las 5 juntas del brazo

  for (int i = 0; i < NUM_JOINTS; i++) {
    if ((size_t)i < msg->data.size) {
      float lo = JOINT_MIN_DEG[i] * DEG2RAD;
      float hi = JOINT_MAX_DEG[i] * DEG2RAD;
      command_data[i] = clampf(msg->data.data[i], lo, hi);
    }
    // si no llega el gripper (size==5) se conserva su ultimo valor.
  }
  apply_servo_commands(command_data);
}

// ---------------------------------------------------------------------------
//  Realimentacion: leer pots, filtrar, volcar a position_data
// ---------------------------------------------------------------------------
void read_feedback() {
  float measured[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; i++) measured[i] = adc_to_rad(analogRead(POT_PINS[i]), i);

#if (USE_JOINT4 == 0)
  measured[J4_INDEX] = 0.0f;   // J4 anulado: se reporta exactamente HOME
#endif

  if (!filter_initialized) {
    for (int i = 0; i < NUM_JOINTS; i++) feedback_filtered[i] = measured[i];
    filter_initialized = true;
  } else {
    for (int i = 0; i < NUM_JOINTS; i++)
      feedback_filtered[i] = ALPHA * measured[i] + (1.0f - ALPHA) * feedback_filtered[i];
  }
  for (int i = 0; i < NUM_JOINTS; i++) position_data[i] = feedback_filtered[i];
}

// ---------------------------------------------------------------------------
//  Timer: publica /joint_states a 25 Hz
// ---------------------------------------------------------------------------
void timer_callback(rcl_timer_t *t, int64_t last) {
  (void)last;
  if (t == NULL) return;
  read_feedback();
  int64_t ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec     = (int32_t)(ns / 1000000000LL);
  joint_state_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

// ---------------------------------------------------------------------------
//  Configuracion del mensaje JointState
// ---------------------------------------------------------------------------
void setup_joint_state_msg() {
  for (int i = 0; i < NUM_JOINTS; i++) {
    joint_names[i].data     = joint_name_buf[i];
    joint_names[i].size     = strlen(joint_name_buf[i]);
    joint_names[i].capacity = strlen(joint_name_buf[i]) + 1;
  }
  joint_state_msg.name.data = joint_names;
  joint_state_msg.name.size = NUM_JOINTS; joint_state_msg.name.capacity = NUM_JOINTS;
  joint_state_msg.position.data = position_data;
  joint_state_msg.position.size = NUM_JOINTS; joint_state_msg.position.capacity = NUM_JOINTS;
  joint_state_msg.velocity.data = velocity_data;
  joint_state_msg.velocity.size = NUM_JOINTS; joint_state_msg.velocity.capacity = NUM_JOINTS;
  joint_state_msg.effort.data = effort_data;
  joint_state_msg.effort.size = NUM_JOINTS; joint_state_msg.effort.capacity = NUM_JOINTS;
  joint_state_msg.header.frame_id.data = (char *)"";
  joint_state_msg.header.frame_id.size = 0; joint_state_msg.header.frame_id.capacity = 1;
}

void setup_command_msg() {
  command_msg.data.data = command_data;
  command_msg.data.size = NUM_JOINTS; command_msg.data.capacity = NUM_JOINTS;
}

// ===========================================================================
//  setup / loop
// ===========================================================================
void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  for (int i = 0; i < NUM_JOINTS; i++) {
    position_data[i] = velocity_data[i] = effort_data[i] = 0.0;
    command_data[i] = 0.0f; feedback_filtered[i] = 0.0f;
  }

  Serial.begin(115200);
  set_microros_transports();

  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {   // esperar al Agent
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(500);
  }
  digitalWrite(STATUS_LED_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_JOINTS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], 500, 2400);
  }

  float q_home[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};   // HOME (servos a 90 deg)
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

  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  delay(1);
}
