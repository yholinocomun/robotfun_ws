/* ============================================================================
 *  robotfun_esp32_microros.ino
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 4 GDL (yaw + 3 pitch) + GRIPPER.
 *
 *  CAMBIO: el JOINT 4 de roll (muñeca giratoria) se ELIMINÓ por completo. El
 *  antiguo joint_5 (pitch de muñeca) es ahora el nuevo joint_4. Quedan 4 juntas
 *  de brazo + gripper = 5 canales. El servo del roll, si sigue montado, se deja
 *  FIJO a 90° (muerto) para que no se mueva.
 *
 *  Pipeline:
 *    /joint_command (std_msgs/Float32MultiArray, RADIANES [q1,q2,q3,q4, gripper])
 *        --> mueve 5 servos
 *    5 potenciometros --> /joint_states (sensor_msgs/JointState, RADIANES) @25 Hz
 *
 *  UNIDADES: el bus ROS va en RADIANES (REP-103, JointState). La conversion a
 *  GRADOS de servo ocurre solo en servo.write(). La calibracion se expresa en
 *  grados/ADC (intuitivo del hardware).
 *
 *  HOME: todas las juntas a 0 rad => servos a 90 grados (brazo vertical).
 *
 *  PINES (4 brazo + gripper):
 *      idx :   0     1     2     3       4
 *      junta:  j1    j2    j3    j4    gripper
 *      SERVO:  2     4     5     19      21
 *      POT  :  32    33    34    27      26     (34 solo IN; 27/26 ADC2 OK con serial)
 *      Servo de ROLL eliminado: GPIO 18, mantenido fijo a 90°.
 *      LED de estado: GPIO 13 (GPIO 2 es el servo j1).
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
#define NUM_CH         5          // 4 juntas de brazo + 1 gripper
#define GRIPPER_INDEX  4

// Servo del roll eliminado: se deja fijo a 90° (muerto). Si ya no está montado,
// pon DEAD_ROLL_PIN en -1 para ignorarlo.
#define DEAD_ROLL_PIN  18

const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 19, 21 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 27, 26 };

const char *JOINT_LABEL[NUM_CH] = { "joint_1", "joint_2", "joint_3", "joint_4", "gripper" };

// ===========================================================================
//  CALIBRACION (grados / cuentas ADC). Ajustar en pruebas.
// ===========================================================================
// RAW_ZERO[i]: ADC (0..4095) de cada pot en HOME (todas las juntas a 0 rad).
int RAW_ZERO[NUM_CH] = {
  1194,   // joint_1
  1165,   // joint_2
  1191,   // joint_3
  1423,   // joint_4  (antiguo joint_5, pitch de muñeca)
  1300    // gripper
};

// FEEDBACK_DIRECTION[i]: sentido del pot hacia RViz (-1 si sale invertido).
const float FEEDBACK_DIRECTION[NUM_CH] = { 1, 1, 1, 1, 1 };

// SERVO_DIRECTION[i]: sentido del servo respecto al comando (+q en el sentido
//   positivo de la convencion DH). Cambia 1 por -1 si gira al reves. CALIBRAR.
const float SERVO_DIRECTION[NUM_CH] = { -1, -1, 1, 1, -1 };

// Rango articular por canal (grados). Brazo ±90; gripper 0..70 (cierra/abre).
const float JOINT_MIN_DEG[NUM_CH] = { -90, -90, -90, -90,  0 };
const float JOINT_MAX_DEG[NUM_CH] = {  90,  90,  90,  90, 70 };
const float SERVO_CENTER_DEG = 90.0f;

const float ADC_TO_DEG = 180.0f / 4095.0f;     // pot de 180° sobre 0..4095
const float DEG2RAD = 0.017453292519943295f;
const float RAD2DEG = 57.29577951308232f;
const float ALPHA = 0.15f;                      // filtro exponencial del ADC

// ===========================================================================
Servo servos[NUM_CH];
Servo dead_roll_servo;
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

static char joint_name_buf[NUM_CH][12] = { "joint_1", "joint_2", "joint_3", "joint_4", "gripper" };
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

// radianes -> grados de servo (0..180). 0 rad => 90° (centro = HOME).
int rad_to_servo_deg(float q_rad, int i) {
  float q_deg = clampf(q_rad * RAD2DEG, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  float servo_deg = SERVO_CENTER_DEG + SERVO_DIRECTION[i] * q_deg;
  return (int)clampf(servo_deg, 0.0f, 180.0f);
}

void apply_servo_commands(const float *q_cmd) {
  for (int i = 0; i < NUM_CH; i++) servos[i].write(rad_to_servo_deg(q_cmd[i], i));
}

// Callback de /joint_command (RADIANES, [q1..q4, gripper]; acepta >=4).
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;     // se necesitan al menos las 4 juntas del brazo
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      float lo = JOINT_MIN_DEG[i] * DEG2RAD, hi = JOINT_MAX_DEG[i] * DEG2RAD;
      command_data[i] = clampf(msg->data.data[i], lo, hi);
    }   // si no llega el gripper (size==4) conserva su ultimo valor
  }
  apply_servo_commands(command_data);
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
#if (DEAD_ROLL_PIN >= 0)
  dead_roll_servo.setPeriodHertz(50);
  dead_roll_servo.attach(DEAD_ROLL_PIN, 500, 2400);
  dead_roll_servo.write(90);          // roll eliminado: fijo a 90° (muerto)
#endif

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

  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  delay(1);
}
