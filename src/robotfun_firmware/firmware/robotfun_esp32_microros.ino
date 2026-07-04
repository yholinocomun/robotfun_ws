/* ============================================================================
 *  robotfun_esp32_microros.ino   —   VERSION ROBUSTA (un solo hilo, sin FreeRTOS)
 *  ---------------------------------------------------------------------------
 *  BAJO NIVEL (micro-ROS) del brazo de 4 GDL (yaw + 3 pitch) + GRIPPER.
 *
 *  POR QUE ESTA VERSION NO SE RESETEA:
 *    La version anterior usaba una TAREA FreeRTOS dedicada; eso introducia el
 *    reset (desbordamiento de pila / interaccion con el watchdog del sistema).
 *    Aqui NO hay tarea: TODO corre en loop(), igual que tus versiones que NUNCA
 *    se reseteaban. La FLUIDEZ no venia de la tarea, sino de mover el servo con
 *    PASOS FINOS en microsegundos (float), no en saltos de 1 grado -> eso se hace
 *    igual dentro de loop(). Un temporizador por micros() da el paso de 20 ms.
 *      - Sin tarea, sin memoria RTC, loop() cede CPU con delay(1) (no watchdog).
 *      - Lazo abierto: llega al objetivo y se queda (no se re-homea solo).
 *
 *  SI AUN SE RESETEA -> es BROWNOUT (la fuente no aguanta la corriente de los
 *  servos). Confirmalo: Monitor Serie 115200 SIN el agente; al arrancar imprime
 *  "Brownout detector was triggered". Arreglo de raiz: fuente EXTERNA 5-6V para
 *  los servos (NO desde el ESP32/USB), GND comun, condensador 1000uF+ cerca de
 *  los servos. Parche de software (ultimo recurso): DISABLE_BROWNOUT abajo.
 *
 *  Juntas: 5 actuadores joint_1..joint_5 (sin el antiguo roll). joint_4 = pitch
 *  de muñeca (antiguo joint_5); joint_5 = GRIPPER.
 *
 *  PINES: SERVO {2,4,5,18,19}  POT {32,33,34,35,27}.  LED de estado: GPIO 13.
 *  (Si tu cableado de servos sigue en {2,4,5,19,21}, ajusta SERVO_PINS[].)
 * ==========================================================================*/

// >>> Parche de ULTIMO RECURSO si el reset es por BROWNOUT y no puedes arreglar
//     la alimentacion ya mismo. Pon 1 para desactivar el detector de brownout.
//     OJO: es un band-aid; lo correcto es una fuente de servos externa 5-6V.
#define DISABLE_BROWNOUT 0

#include <math.h>
#if DISABLE_BROWNOUT
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#define STATUS_LED_PIN 13
#define NUM_CH         5
#define GRIPPER_INDEX  4

const int SERVO_PINS[NUM_CH] = {  2,  4,  5, 18, 19 };
const int POT_PINS[NUM_CH]   = { 32, 33, 34, 35, 27 };
const char *JOINT_LABEL[NUM_CH] = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" };

// ===========================================================================
//  MAPEO DE SERVO (grados). servo_deg = CENTER + DIR*q_deg, saturado a [0,180].
//  Si el gripper (joint_5) gira al reves, pon SERVO_DIRECTION[4] = -1.
// ===========================================================================
const int   SERVO_DIRECTION[NUM_CH]  = {  1, -1,  1, -1,  1 };
const float SERVO_CENTER_DEG[NUM_CH] = { 90, 90, 90, 90, 90 };
const float JOINT_MIN_DEG[NUM_CH]    = { -90, -90, -90, -90,  0 };
const float JOINT_MAX_DEG[NUM_CH]    = {  90,  90,  90,  90, 70 };
const int   SERVO_US_MIN = 500;      // us a 0 grados   (coincide con attach)
const int   SERVO_US_MAX = 2400;     // us a 180 grados

// ===========================================================================
//  FLUIDEZ: perfil trapezoidal por junta (grados/s, grados/s^2).
//  MAX_VEL = crucero (mas bajo = mas lento). MAX_ACC = suavidad del arranque/frenado.
// ===========================================================================
// idx:                            j1     j2     j3     j4    j5(grip)
const float MAX_VEL[NUM_CH] = {  70.0f, 70.0f, 70.0f, 70.0f, 120.0f };  // grados/s
const float MAX_ACC[NUM_CH] = { 150.0f,150.0f,150.0f,150.0f, 300.0f };  // grados/s^2
#define SERVO_TICK_US 20000UL                   // 20 ms = 50 Hz (trama del servo)

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
Servo servos[NUM_CH];

// Perfil de movimiento (grados de junta). target_deg lo fija /joint_command.
float target_deg[NUM_CH];
float cur_deg[NUM_CH];
float cur_vel[NUM_CH];
int   last_us[NUM_CH];                 // ultimo PWM escrito (para escribir solo si cambia)
unsigned long last_tick_us = 0;

// Feedback
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

// grados de junta q -> microsegundos PWM (float => resolucion fina => fluido).
int deg_to_us(float q_deg, int i) {
  q_deg = clampf(q_deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  float servo_deg = SERVO_CENTER_DEG[i] + SERVO_DIRECTION[i] * q_deg;
  servo_deg = clampf(servo_deg, 0.0f, 180.0f);
  return (int)(SERVO_US_MIN + servo_deg * (float)(SERVO_US_MAX - SERVO_US_MIN) / 180.0f);
}

// ADC -> radianes (feedback). 0 rad corresponde a RAW_ZERO[i] (HOME).
float adc_to_rad(int raw, int i) {
  float deg = FEEDBACK_DIRECTION[i] * (float)(raw - RAW_ZERO[i]) * ADC_TO_DEG;
  deg = clampf(deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
  return deg * DEG2RAD;
}

// Un paso del perfil trapezoidal por junta (FLUIDO). Escribe el servo solo si el
// PWM cambia. Deteccion de cruce del objetivo => 0 overshoot, sin jitter.
void update_servos(float dt) {
  for (int i = 0; i < NUM_CH; i++) {
    float tgt = clampf(target_deg[i], JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
    float err = tgt - cur_deg[i];
    if (fabsf(err) > 1e-4f) {
      float v_stop = sqrtf(2.0f * MAX_ACC[i] * fabsf(err));
      float v_des  = (err >= 0.0f ? 1.0f : -1.0f) * fminf(MAX_VEL[i], v_stop);
      float dv = v_des - cur_vel[i];
      float dvm = MAX_ACC[i] * dt;
      if (dv >  dvm) dv =  dvm;
      if (dv < -dvm) dv = -dvm;
      cur_vel[i] += dv;
      float next = cur_deg[i] + cur_vel[i] * dt;
      if ((tgt - cur_deg[i]) * (tgt - next) <= 0.0f) { cur_deg[i] = tgt; cur_vel[i] = 0.0f; }
      else                                             cur_deg[i] = next;
    } else {
      cur_vel[i] = 0.0f;
    }
    int us = deg_to_us(cur_deg[i], i);
    if (us != last_us[i]) { servos[i].writeMicroseconds(us); last_us[i] = us; }
  }
}

// Callback de /joint_command (RADIANES [q1..q4, gripper]; acepta >=4).
void command_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 4) return;
  for (int i = 0; i < NUM_CH; i++) {
    if ((size_t)i < msg->data.size) {
      float q_deg = msg->data.data[i] * RAD2DEG;
      target_deg[i] = clampf(q_deg, JOINT_MIN_DEG[i], JOINT_MAX_DEG[i]);
    }
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
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);    // desactiva el detector de brownout (band-aid)
#endif

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);

  for (int i = 0; i < NUM_CH; i++) {
    position_data[i] = velocity_data[i] = effort_data[i] = 0.0;
    command_data[i] = 0.0f; feedback_filtered[i] = 0.0f;
    target_deg[i] = cur_deg[i] = cur_vel[i] = 0.0f;   // arranca en HOME, quieto
    last_us[i] = -1;
  }

  // Servos: attach + HOME, ESCALONADO para no pedir toda la corriente de golpe.
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < NUM_CH; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PINS[i], SERVO_US_MIN, SERVO_US_MAX);
    last_us[i] = deg_to_us(0.0f, i);
    servos[i].writeMicroseconds(last_us[i]);
    delay(120);                                 // separa el arranque de cada servo
  }
  delay(600);

  Serial.begin(115200);
  set_microros_transports();
  while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); delay(500);
  }
  digitalWrite(STATUS_LED_PIN, HIGH);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

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

  last_tick_us = micros();
  digitalWrite(STATUS_LED_PIN, LOW);
}

void loop() {
  // Paso del perfil suave cada 20 ms EXACTOS (por micros(), robusto a overflow).
  unsigned long now = micros();
  if (now - last_tick_us >= SERVO_TICK_US) {
    float dt = (now - last_tick_us) * 1e-6f;
    last_tick_us = now;
    update_servos(dt);
  }
  // micro-ROS: recibe /joint_command y publica el feedback (spin corto).
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  delay(1);   // cede CPU (evita reset por watchdog). NO quitar.
}
