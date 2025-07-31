// === Konfigurasi Pin Motor ===
const int RPWM_1 = 8;   // Motor 1
const int LPWM_1 = 9;
const int RPWM_2 = 6;   // Motor 2
const int LPWM_2 = 7;
const int RPWM_3 = 5;   // Motor 3
const int LPWM_3 = 4;
const int RPWM_4 = 2;   // Motor 4
const int LPWM_4 = 3;

// === Parameter Robot ===
const float WHEEL_BASE = 0.88;     // jarak antar roda (meter)
const float WHEEL_DIAMETER = 0.35; // diameter roda (meter)
const int MAX_PWM = 100;
const float MAX_SPEED_MS = 1.5;

float v = 0.0;  // linear.x dari ROS2
float w = 0.0;  // angular.z dari ROS2

void setup() {
  Serial.begin(115200);

  // Set semua pin sebagai output
  int pins[] = {RPWM_1, LPWM_1, RPWM_2, LPWM_2, RPWM_3, LPWM_3, RPWM_4, LPWM_4};
  for (int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    parseCmdVel(input);
    applyKinematics(v, w);
  }
}

void parseCmdVel(String data) {
  int vIndex = data.indexOf("v:");
  int wIndex = data.indexOf("w:");
  if (vIndex >= 0 && wIndex >= 0) {
    v = data.substring(vIndex + 2, data.indexOf(',', vIndex)).toFloat();
    w = data.substring(wIndex + 2).toFloat();
  }
}

void applyKinematics(float v, float w) {
  float v_l = v - (w * WHEEL_BASE / 2.0);
  float v_r = v + (w * WHEEL_BASE / 2.0);

  // Kontrol motor kiri (Motor 1 dan 3)
  controlMotor(LPWM_1, RPWM_1, v_l);
  controlMotor(LPWM_3, RPWM_3, v_l);

  // Kontrol motor kanan (Motor 2 dan 4)
  controlMotor(LPWM_2, RPWM_2, v_r);
  controlMotor(LPWM_4, RPWM_4, v_r);
}

void controlMotor(int lpwm, int rpwm, float velocity) {
  int pwm_value = (int)((abs(velocity) / MAX_SPEED_MS) * MAX_PWM);
  pwm_value = constrain(pwm_value, 0, MAX_PWM); // Pastikan tidak melebihi batas

  if (velocity > 0.01) {
    analogWrite(rpwm, pwm);
    analogWrite(lpwm, 0);
  } else if (velocity < 0.01) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, pwm);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}
