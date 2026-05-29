/*
 * 四驱小车 — 避障版 v1（固定朝前 HC-SR04）
 *
 * ALG_ID = "ALG-1.0"  slug: reactive_alt
 * 策略说明：本平台 VERSION.md
 * 后续 v2 规划：docs/DEVELOPMENT.md
 *
 * 平台：uno_tb6612_hc04
 * Arduino IDE：打开文件夹 uno_tb6612_hc04_avoid_v1_alg_1_0 上传。
 * 刷演示版：打开 uno_tb6612_hc04_demo_motor，无需 git 回退。
 *
 * HC-SR04：Vcc 5V  Gnd 共地  Trig D9  Echo D12
 * 调试：DEBUG_HOLD_FORWARD / DEBUG_SENSOR_ONLY / SERIAL_DEBUG
 */

#define ALG_ID           "ALG-1.0"
#define ALG_SLUG         "reactive_alt"
#define FIRMWARE_VERSION "v1"

#define DEBUG_HOLD_FORWARD 0
#define DEBUG_SENSOR_ONLY  0   // 1=只串口输出距离，电机不动
#define SERIAL_DEBUG        1   // 1=打印测距结果
#define SERIAL_DEBUG_RAW     1   // 1=同时打印 us= 与失败原因（排查 cm=-1 时保持为 1）

// ---------- TB6612（与演示版相同）----------
const int PIN_PWMA = 5;
const int PIN_AIN1 = 4;
const int PIN_AIN2 = 3;

const int PIN_PWMB = 6;
const int PIN_BIN1 = 7;
const int PIN_BIN2 = 8;

const int PIN_STBY = 10;

// ---------- HC-SR04 ----------
const int PIN_TRIG = 9;
const int PIN_ECHO = 12;

// 若避障时「该前进却倒车」，两侧都改为 true（与演示版架空标定方式相同）
const bool MOTOR_LEFT_DIR_REVERSE  = true;
const bool MOTOR_RIGHT_DIR_REVERSE = true;

const int SPEED_LOW  = 120;
const int SPEED_HIGH = 200;

const int MAX_VALID_CM   = 400;
const int MS_LOOP_PAUSE  = 50;

// ---------- 自适应避障（遇障仍堵则档位升高，参数从小到大）----------
const int OBSTACLE_CM_MIN   = 18;    // 初始触发距离
const int OBSTACLE_CM_STEP  = 4;     // 每升一档略提前刹车（最大见 OBSTACLE_CM_MAX）
const int OBSTACLE_CM_MAX   = 32;

const int MS_BACKUP_MIN     = 280;
const int MS_BACKUP_STEP    = 140;
const int MS_BACKUP_MAX     = 900;

const int MS_TURN_MIN       = 320;
const int MS_TURN_STEP      = 220;   // 左右切换时转角递增
const int MS_TURN_MAX       = 1400;

const byte STUCK_MAX_LEVEL  = 4;     // 0~4 共 5 档
const unsigned long STUCK_REPEAT_MS = 1800;  // 此时间内再次遇障 → 升档
const unsigned long STUCK_RESET_MS  = 2500;  // 前方畅通这么久 → 回 0 档

// 脱困状态（正常避障 loop 使用）
static byte stuckLevel = 0;
static int lastTurnDir = -1;         // 1=右转，-1=左转；每次与上次反向（初值-1→首次右转）
static unsigned long lastAvoidMs = 0;
static unsigned long pathClearSinceMs = 0;

bool isPathBlocked(long cm);
int obstacleThresholdCm();
int backupMsForLevel();
int turnMsForLevel();
void runAdaptiveAvoidance();

void setup() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  stopAll();
  digitalWrite(PIN_STBY, HIGH);

#if SERIAL_DEBUG || DEBUG_SENSOR_ONLY
  Serial.begin(9600);
  delay(200);
  Serial.print(F("uno_tb6612_hc04_avoid_v1_alg_1_0 "));
  Serial.print(ALG_ID);
  Serial.print(F(" "));
  Serial.println(ALG_SLUG);
  Serial.println(F("=== HC-SR04 9600 ==="));
  Serial.print(F("Echo idle (应多为0): "));
  Serial.println(digitalRead(PIN_ECHO));
  Serial.println(F("Trig=D9 Echo=D12; 一直 us=0 查接线/Trig与Echo是否接反"));
#endif

  delay(1000);
}

void loop() {
#if DEBUG_HOLD_FORWARD
  // 只测电机
  driveForward(255);

#elif DEBUG_SENSOR_ONLY
  // 只测距；单次读数 + 原始 us 便于排查
  printDistanceDebug();
  delay(250);

#else
  // 正常避障（自适应脱困）
  long cm = readDistanceCmFiltered();

#if SERIAL_DEBUG
  Serial.print(F("cm="));
  Serial.print(cm);
  Serial.print(F(" lvl="));
  Serial.println(stuckLevel);
#endif

  if (isPathBlocked(cm)) {
    pathClearSinceMs = 0;
    runAdaptiveAvoidance();
    return;
  }

  // 前方够开阔：一段时间后降低脱困档位
  if (pathClearSinceMs == 0) {
    pathClearSinceMs = millis();
  } else if (millis() - pathClearSinceMs >= STUCK_RESET_MS) {
    stuckLevel = 0;
  }

  driveForward(SPEED_LOW);
  delay(MS_LOOP_PAUSE);
#endif
}

// ==================== 自适应脱困 ====================

bool isPathBlocked(long cm) {
  return cm > 0 && cm < obstacleThresholdCm();
}

int obstacleThresholdCm() {
  int t = OBSTACLE_CM_MIN + (int)stuckLevel * OBSTACLE_CM_STEP;
  return (t > OBSTACLE_CM_MAX) ? OBSTACLE_CM_MAX : t;
}

int backupMsForLevel() {
  int ms = MS_BACKUP_MIN + (int)stuckLevel * MS_BACKUP_STEP;
  return constrain(ms, MS_BACKUP_MIN, MS_BACKUP_MAX);
}

int turnMsForLevel() {
  int ms = MS_TURN_MIN + (int)stuckLevel * MS_TURN_STEP;
  return constrain(ms, MS_TURN_MIN, MS_TURN_MAX);
}

void runAdaptiveAvoidance() {
  unsigned long now = millis();

  // 很快又撞墙：说明上次转弯不够 → 升档（后退更远、转更久、阈值略增）
  if (lastAvoidMs != 0 && (now - lastAvoidMs) < STUCK_REPEAT_MS) {
    if (stuckLevel < STUCK_MAX_LEVEL) {
      stuckLevel++;
    }
  }
  lastAvoidMs = now;

  // 与上次反向，且 stuckLevel 越大 turnMs 越长（更大转角）
  lastTurnDir = -lastTurnDir;

  stopAll();
  delay(150);

  driveBackward(SPEED_LOW);
  delay(backupMsForLevel());
  stopAll();
  delay(150);

  if (lastTurnDir > 0) {
    turnRight(SPEED_LOW);
  } else {
    turnLeft(SPEED_LOW);
  }
  delay(turnMsForLevel());
  stopAll();
  delay(200);

#if SERIAL_DEBUG
  Serial.print(F("avoid dir="));
  Serial.print(lastTurnDir > 0 ? F("R") : F("L"));
  Serial.print(F(" back="));
  Serial.print(backupMsForLevel());
  Serial.print(F(" turn="));
  Serial.println(turnMsForLevel());
#endif
}

// ==================== HC-SR04 ====================

// 返回 Echo 高电平宽度（微秒）；0 = pulseIn 超时（常见：未接、接反、模块无响应）
unsigned long measureEchoUs() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, 30000UL);
}

// 把 Echo 脉宽 us 换成 cm；无效返回 -1
long usToCm(unsigned long us) {
  if (us == 0) {
    return -1;
  }
  long cm = (long)(us / 58);
  if (cm < 2 || cm > MAX_VALID_CM) {
    return -1;
  }
  return cm;
}

long readDistanceCm() {
  return usToCm(measureEchoUs());
}

void printDistanceDebug() {
  unsigned long us = measureEchoUs();
  long cm = usToCm(us);
#if SERIAL_DEBUG_RAW
  Serial.print(F("us="));
  Serial.print(us);
  Serial.print(F(" cm="));
  Serial.print(cm);
  if (us == 0) {
    Serial.println(F("  [超时:查Vcc/Gnd/Trig与Echo是否接反]"));
  } else if (cm < 0) {
    Serial.println(F("  [超量程:<2cm或>400cm]"));
  } else {
    Serial.println();
  }
#else
  Serial.print(F("cm="));
  Serial.println(cm);
#endif
}

long readDistanceCmFiltered() {
  long a = readDistanceCm();
  delay(20);
  long b = readDistanceCm();
  delay(20);
  long c = readDistanceCm();

  if (a < 0) a = 999;
  if (b < 0) b = 999;
  if (c < 0) c = 999;

  long m = a;
  if (b < m) m = b;
  if (c < m) m = c;

  return (m >= 999) ? -1L : m;
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

void brakeChannel(int pinIn1, int pinIn2, int pinPwm) {
  digitalWrite(pinIn1, HIGH);
  digitalWrite(pinIn2, HIGH);
  analogWrite(pinPwm, 255);
}

void brakeAll() {
  brakeChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA);
  brakeChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB);
}
