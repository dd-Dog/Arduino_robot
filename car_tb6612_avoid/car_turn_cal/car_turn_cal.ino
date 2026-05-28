/*
 * 原地转向标定 — 测 SPEED_LOW 下「度 / 毫秒」
 *
 * 用途：校准 car_tb6612_avoid_v2 里的
 *       const float DEG_PER_MS_AT_SPEED_LOW = 0.30f;
 *
 * Arduino IDE：打开文件夹 car_turn_cal 上传。
 * 接线与 v2 相同（TB6612 D3~D8，STBY D10），无需 HC-SR04。
 *
 * 用法：
 *   1. 车轮架空，或地面贴一条胶带当 0° 参考线。
 *   2. 串口 9600，看提示；每次转 CAL_SPIN_MS 毫秒后停 5s 再转。
 *   3. 用 protractor / 手机量角 / 数整圈，记下实际转角 D（度）。
 *   4. DEG_PER_MS = D / CAL_SPIN_MS  → 写回 v2。
 *
 * 例：转 1200ms 实测刚好 360° → DEG_PER_MS = 360/1200 = 0.30
 */

// ---------- 与 v2 保持一致 ----------
const int PIN_PWMA = 5;
const int PIN_AIN1 = 4;
const int PIN_AIN2 = 3;
const int PIN_PWMB = 6;
const int PIN_BIN1 = 7;
const int PIN_BIN2 = 8;
const int PIN_STBY = 10;

const bool MOTOR_LEFT_DIR_REVERSE  = false;
const bool MOTOR_RIGHT_DIR_REVERSE = false;

const int SPEED_LOW = 120;   // 与 v2 相同

// 单次原地转多久（毫秒）；v2 标定约 0.10~0.11 °/ms → 1200ms 约 120°~135°
const unsigned long CAL_SPIN_MS = 1200;

// 1=右转（与 v2 turnRight 一致），-1=左转
const int TURN_DIR = 1;

const unsigned long PAUSE_BETWEEN_MS = 5000;
const unsigned long COUNTDOWN_MS     = 2000;

// 仅用于串口打印「按当前猜测值应转多少度」
const float GUESS_DEG_PER_MS = 0.105f;

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
  Serial.print(F("guess DEG_PER_MS="));
  Serial.println(GUESS_DEG_PER_MS, 4);
  Serial.print(F("guess spin deg="));
  Serial.println(CAL_SPIN_MS * GUESS_DEG_PER_MS, 1);
  Serial.print(F("turn "));
  Serial.println(TURN_DIR > 0 ? F("RIGHT") : F("LEFT"));
  Serial.println(F("Mark 0 deg, then each spin measure actual D."));
  Serial.println(F("DEG_PER_MS = D / CAL_SPIN_MS"));
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
