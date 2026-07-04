/* ============================================================================
 *  robotfun_calibracion_raw.ino
 *  ---------------------------------------------------------------------------
 *  Sketch AUXILIAR de CALIBRACIÓN del HOME (no usa micro-ROS).
 *
 *  Lee los 5 potenciómetros y muestra el ADC CRUDO por el Monitor Serie.
 *  Sirve para fijar RAW_ZERO[] del firmware principal:
 *
 *    1) Sube ESTE sketch al ESP32.
 *    2) Abre el Monitor Serie a 115200 baudios.
 *    3) Coloca el brazo EXACTAMENTE en HOME (vertical, servos a su centro).
 *    4) Copia los 5 valores "raw" que se imprimen en RAW_ZERO[5] del firmware
 *       principal (robotfun_esp32_microros.ino), en el MISMO orden.
 *    5) Vuelve a subir el firmware principal.
 *
 *  Pines de los pots (igual que el firmware principal, tras reasignar pines):
 *      idx :   0     1     2       3          4
 *      junta:  j1    j2    j3    j4(pitch)  j5(gripper)
 *      POT  :  32    33    34    35         27
 *  (se REVIVIÓ el pin 35 para j4, CONTINÚA con 27 para el gripper, se ANULÓ 26)
 * ==========================================================================*/

const int NUM_CH = 5;
const int POT_PINS[NUM_CH] = { 32, 33, 34, 35, 27 };
const char *LABEL[NUM_CH]  = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5" };

// Filtro de media móvil para una lectura estable
const int FILT = 20;
int buf[NUM_CH][FILT];
long acc[NUM_CH];
int idx = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);          // 0..4095
  analogSetAttenuation(ADC_11db);    // rango completo ~3.3 V
  for (int i = 0; i < NUM_CH; i++) {
    pinMode(POT_PINS[i], INPUT);
    for (int k = 0; k < FILT; k++) buf[i][k] = 0;
    acc[i] = 0;
  }
  delay(300);
  Serial.println("== Calibracion RAW (coloca el brazo en HOME y lee estos valores) ==");
}

void loop() {
  for (int i = 0; i < NUM_CH; i++) {
    acc[i] -= buf[i][idx];
    buf[i][idx] = analogRead(POT_PINS[i]);
    acc[i] += buf[i][idx];
  }
  idx = (idx + 1) % FILT;

  // Imprime ~4 veces por segundo
  static unsigned long t = 0;
  if (millis() - t > 250) {
    t = millis();
    Serial.print("RAW_ZERO[5] = { ");
    for (int i = 0; i < NUM_CH; i++) {
      Serial.print((int)(acc[i] / FILT));
      Serial.print(i < NUM_CH - 1 ? ", " : " ");
    }
    Serial.print("};   //");
    for (int i = 0; i < NUM_CH; i++) { Serial.print(" "); Serial.print(LABEL[i]); }
    Serial.println();
  }
  delay(5);
}
