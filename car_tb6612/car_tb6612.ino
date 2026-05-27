/*
 * 四驱小车 — 只需 1 块 TB6612（只有 A、B 两路，不是 4 路）
 *
 *   本程序按「一块驱动板、两路输出」编写，不需要第二块 TB6612。
 *
 *        ┌── TB6612（1 块）──┐
 *        │  A 路：PWMA/AIN   │── AO1 AO2 ── 左前 + 左后（并联，同向）
 *        │  B 路：PWMB/BIN   │── BO1 BO2 ── 右前 + 右后（并联，同向）
 *        └───────────────────┘
 *        程序里：setMotorLeft = A 路，setMotorRight = B 路
 *
 * 模块参数（见引脚图）：
 *   每路工作电流 1.2A / 峰值 3.2A（一侧两电机并联，电流会叠加，勿堵转）
 *   VM 4.5~10V   VCC 2.7~5.5V（接 5V）   STBY=HIGH 使能
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  TB6612 引脚          接法                              │
 * ├─────────────────────────────────────────────────────────┤
 * │  VM                   电池正极 4.5~10V                  │
 * │  VCC                  Arduino 5V                      │
 * │  GND (任一脚)         与 Uno、电池负共地                │
 * │  STBY                 D10，程序拉高使能                 │
 * │  PWMA  AIN1  AIN2     D5   D4   D3  → 左轮速度/方向    │
 * │  AO1  AO2             左电机 + -                        │
 * │  PWMB  BIN1  BIN2     D6   D7   D8  → 右轮速度/方向    │
 * │  BO1  BO2             右电机 + -                        │
 * └─────────────────────────────────────────────────────────┘
 *
 * 方向逻辑（与引脚图一致）：
 *   正转：IN1=HIGH, IN2=LOW  + PWM
 *   反转：IN1=LOW,  IN2=HIGH + PWM
 *   停止：IN1=LOW,  IN2=LOW  PWM=0
 *   短刹：IN1=HIGH, IN2=HIGH（本程序未用）
 *
 * 调试：车轮架空；某侧反了改 MOTOR_LEFT/RIGHT_DIR_REVERSE
 *
 * 重要：请用 Arduino IDE 打开文件夹 car_tb6612（不要打开 sketch_may21a）
 * IDE 须显示：car_tb6612 | Arduino Uno，上传成功后再测引脚。
 */

#define DEBUG_HOLD_FORWARD 0  // 1=一直前进便于万用表测 D3/D4/D5（测完改回 0）

// ---------- 逻辑控制脚 → Arduino ----------
const int PIN_PWMA = 5;    // A 路 PWM → 左侧两电机
const int PIN_AIN1 = 4;
const int PIN_AIN2 = 3;

const int PIN_PWMB = 6;    // B 路 PWM → 右侧两电机
const int PIN_BIN1 = 7;
const int PIN_BIN2 = 8;

const int PIN_STBY = 10;   // 待机控制，必须 HIGH 才工作

// 架空试车若前进变后退，把对应侧改为 true
const bool MOTOR_LEFT_DIR_REVERSE  = false;
const bool MOTOR_RIGHT_DIR_REVERSE = false;

const int SPEED_LOW  = 120;   // 转向稍慢
const int SPEED_HIGH = 200;   // 直行（≤255，过大可能电流偏大）

const int MS_FORWARD  = 2000;
const int MS_BACKWARD = 2000;
const int MS_TURN     = 800;
const int MS_STOP     = 500;

void setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  stopAll();

  // 引脚图：STBY 拉高，芯片退出待机，A/B 路才响应
  digitalWrite(PIN_STBY, HIGH);

  delay(1000);  // 上电延时，便于架空检查
}

void loop() {
#if DEBUG_HOLD_FORWARD
  // 测引脚：前进时常应为 D4≈5V, D3≈0V, D5有PWM, D10≈5V, D6~D8同理
  driveForward(255);
  return;
#endif

  driveForward(SPEED_HIGH);
  delay(MS_FORWARD);
  stopAll();
  delay(MS_STOP);

  driveBackward(SPEED_HIGH);
  delay(MS_BACKWARD);
  stopAll();
  delay(MS_STOP);

  turnLeft(SPEED_LOW);
  delay(MS_TURN);
  stopAll();
  delay(MS_STOP);

  turnRight(SPEED_LOW);
  delay(MS_TURN);
  stopAll();
  delay(MS_STOP);

  delay(2000);
}

// ==================== 整车动作 ====================

void driveForward(int speed) {
  setMotorLeft(speed);
  setMotorRight(speed);
}

void driveBackward(int speed) {
  setMotorLeft(-speed);
  setMotorRight(-speed);
}

// 原地左转：左轮反转、右轮正转
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

// ==================== 两路电机：左=A 路，右=B 路（同一块 TB6612）====================

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

/*
 * 单路 TB6612 通道（A 或 B 相同）
 * speed：-255~255，正=引脚图“正转”，负=“反转”，0=停止
 */
void setMotorChannel(int pinIn1, int pinIn2, int pinPwm, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    // 正转：IN1=HIGH, IN2=LOW
    digitalWrite(pinIn1, HIGH);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPwm, speed);
  } else if (speed < 0) {
    // 反转：IN1=LOW, IN2=HIGH
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, HIGH);
    analogWrite(pinPwm, -speed);
  } else {
    // 停止：IN1=IN2=LOW，PWM=0
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    analogWrite(pinPwm, 0);
  }
}

// 可选：急停短刹（需要时调用）
void brakeChannel(int pinIn1, int pinIn2, int pinPwm) {
  digitalWrite(pinIn1, HIGH);
  digitalWrite(pinIn2, HIGH);
  analogWrite(pinPwm, 255);
}

void brakeAll() {
  brakeChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA);
  brakeChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB);
}
