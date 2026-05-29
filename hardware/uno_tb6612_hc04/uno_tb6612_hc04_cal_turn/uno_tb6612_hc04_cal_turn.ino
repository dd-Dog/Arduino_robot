/*
 * 原地转向标定 — 测 SPEED_LOW 下「度 / 毫秒」
 *
 * 公用参数：libraries/uno_tb6612_hc04/common_1_0/motion.h
 * 标定结果写回 motion.h，各避障 sketch 自动同步。
 *
 * Arduino IDE：打开文件夹 uno_tb6612_hc04_cal_turn 上传。
 * 库路径：见 docs/CONFIG.md
 */

#include <uno_tb6612_hc04.h>

// 单次原地转多久（毫秒）；1200ms @ 0.105 °/ms ≈ 126°
const unsigned long CAL_SPIN_MS = 1200;

const int TURN_DIR = 1;   // 1=右转，-1=左转

const unsigned long PAUSE_BETWEEN_MS = 5000;
const unsigned long COUNTDOWN_MS     = 2000;

void turnLeft(int speed);
void turnRight(int speed);
void stopAll();
void setMotorLeft(int speed);
void setMotorRight(int speed);
void setMotorChannel(int pinIn1, int pinIn2, int pinPwm, int speed);

void setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  stopAll();
  digitalWrite(PIN_STBY, HIGH);

  Serial.begin(9600);
  delay(300);

  Serial.println(F("=== turn cal @ SPEED_LOW ==="));
  Serial.print(F("SPEED_LOW="));
  Serial.println(SPEED_LOW);
  Serial.print(F("CAL_SPIN_MS="));
  Serial.println(CAL_SPIN_MS);
  Serial.print(F("DEG_PER_MS (lib)="));
  Serial.println(DEG_PER_MS_AT_SPEED_LOW, 4);
  Serial.print(F("guess spin deg="));
  Serial.println(CAL_SPIN_MS * DEG_PER_MS_AT_SPEED_LOW, 1);
  Serial.print(F("turn "));
  Serial.println(TURN_DIR > 0 ? F("RIGHT") : F("LEFT"));
  Serial.println(F("Mark 0 deg, then each spin measure actual D."));
  Serial.println(F("DEG_PER_MS = D / CAL_SPIN_MS  ->  motion.h"));
  Serial.println();
}

void loop() {
  Serial.println(F("--- next spin in 2s (mark front) ---"));
  delay(COUNTDOWN_MS);

  unsigned long t0 = millis();

  if (TURN_DIR > 0) {
    turnRight(SPEED_LOW);
  } else {
    turnLeft(SPEED_LOW);
  }

  delay(CAL_SPIN_MS);
  stopAll();

  unsigned long elapsed = millis() - t0;

  Serial.print(F("done ms="));
  Serial.print(elapsed);
  Serial.print(F("  if D deg actual: DEG_PER_MS="));
  Serial.print(F("D/"));
  Serial.println(elapsed);
  Serial.print(F("  if exactly 360 deg: DEG_PER_MS="));
  Serial.println(360.0f / (float)elapsed, 4);
  Serial.println();

  delay(PAUSE_BETWEEN_MS);
}

void turnLeft(int speed) {
  setMotorLeft(-speed);
  setMotorRight(speed);
}

void turnRight(int speed) {
  setMotorLeft(speed);
  setMotorRight(-speed);
}

void stopAll() {
  setMotorLeft(0);
  setMotorRight(0);
}

void setMotorLeft(int speed) {
  if (MOTOR_LEFT_DIR_REVERSE) {
    speed = -speed;
  }
  setMotorChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA, speed);
}

void setMotorRight(int speed) {
  if (MOTOR_RIGHT_DIR_REVERSE) {
    speed = -speed;
  }
  setMotorChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB, speed);
}

void setMotorChannel(int pinIn1, int pinIn2, int pinPwm, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(pinIn1, HIGH);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPwm, speed);
  } else if (speed < 0) {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, HIGH);
    analogWrite(pinPwm, -speed);
  } else {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPwm, 0);
  }
}
