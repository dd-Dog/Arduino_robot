/*
 * 四驱小车 — 避障版 v2（固定朝前 HC-SR04 + 状态机 + 右手法则）
 *
 * 版本：FIRMWARE_VERSION = "v2"
 * 与 v1 的差异见本目录 VERSION.md / docs/DEVELOPMENT.md
 *
 * 主要变化：
 *   1. 小车机械参数显式建模（长/宽/高、转向标定 DEG_PER_MS）
 *   2. 距离分段决策：远→快，中→慢，近→转向探测，极近→后退
 *      （不再「一遇障就停 + 后退 + 转向」一气呵成）
 *   3. 状态机：IDLE / FORWARD / DECIDE / TURN_PROBE / BACKUP / ESCAPE
 *   4. 右手法则：DECIDE 优先右探，不行直行，再不行左探，最后后退升档
 *
 * 接线（与 v1 相同）：
 *   电机：D3~D8、STBY D10
 *   HC-SR04：Trig D9，Echo D12，Vcc 5V，Gnd 共地
 *
 * 调试宏：
 *   DEBUG_HOLD_FORWARD / DEBUG_SENSOR_ONLY / SERIAL_DEBUG
 */

#define FIRMWARE_VERSION "v2"

#define DEBUG_HOLD_FORWARD 0
#define DEBUG_SENSOR_ONLY  1   // 1=状态机+SERIAL_DEBUG 照常，电机不转（测完改回 0）
#define SERIAL_DEBUG       1   // 9600；须为 1 才有 ST/FWD/turn/ESC 等（SENSOR_ONLY 不再只打 cm）
#define SERIAL_DEBUG_RAW   0

// ==================== 引脚 ====================

const int PIN_PWMA = 5;
const int PIN_AIN1 = 4;
const int PIN_AIN2 = 3;

const int PIN_PWMB = 6;
const int PIN_BIN1 = 7;
const int PIN_BIN2 = 8;

const int PIN_STBY = 10;

const int PIN_TRIG = 9;
const int PIN_ECHO = 12;

// ==================== 小车机械参数 ====================
// 调参时按实际底盘测一下；只是用于推导避障阈值/转向时间，写错不会损坏硬件

const int CAR_LENGTH_CM = 26;   // 车头到车尾
const int CAR_WIDTH_CM  = 16;   // 车体宽
const int CAR_HEIGHT_CM = 12;    // 车体高+雷达等配件高度（仅供记录）

const int SENSOR_OFFSET_CM = -1; // 探头相对车头的位置（正=突出于车头，负=退后到车体里）
// 防撞按「车上最靠前的部位」为准：
//   offset > 0（探头突出）：探头先撞 → 关心探头距 = 测距，阈值不补偿
//   offset < 0（探头退后）：车头先撞 → 车头距 = 测距 + offset = 测距 - |offset|
//                            阈值要补 |offset| 才能让车头留出同样余量
//   offset = 0：探头与车头齐平，按测距直接判断
// 统一为：阈值补偿量 = max(0, -SENSOR_OFFSET_CM)
// HC-SR04 检测锥角约 ±15°，物理探测下限 2cm

// ---- 转向标定（架空空载，慢速）----
// 在 SPEED_LOW 转速下，原地转向「每毫秒大约多少度」
// 标定方法：上传 DEBUG_TURN_CAL=1（如未启用，可临时改 setup 里跑 turnRight 1000ms 看角度）
// 当前值是中等小车的经验估计，请按实车实测调整
const int   SPEED_LOW    = 120;
const int   SPEED_MID    = 160;
const int   SPEED_HIGH   = 200;
const float DEG_PER_MS_AT_SPEED_LOW = 0.30f;   // ≈ 300°/s

// 由 DEG_PER_MS 推导
int turnMsForDegrees(int deg) {
  if (deg <= 0) return 0;
  long ms = (long)(deg / DEG_PER_MS_AT_SPEED_LOW);
  return (int)constrain(ms, 80L, 3000L);
}

const int MAX_TURN_DEG = 180;   // 单次决策内最大转向角

// ==================== 电机方向修正 ====================

const bool MOTOR_LEFT_DIR_REVERSE  = false;  // 雷达改向后相对 v1 反过一次；某侧反了单独改
const bool MOTOR_RIGHT_DIR_REVERSE = false;

// ==================== 距离分段（按车头距推导，自动补偿探头偏移）====================
// PAD_* 是「车头到障碍」希望保留的余量；探头偏移由 SENSOR_OFFSET_CM 自动加进去。
// 实际比较使用「测距值」，所以阈值 = (车头余量) - SENSOR_OFFSET_CM。

const int BRAKE_PAD_CM   = 12;  // 减速余量
const int STOP_PAD_CM    = 6;   // 必须停车余量
const int BACKUP_PAD_CM  = 4;   // 极近，必须先后退

// 车上最靠前部位的「期望余量」（仅供推导，不直接和 cm 比较）
inline int frontStop()  { return STOP_PAD_CM + BACKUP_PAD_CM; }
inline int frontBrake() { return CAR_WIDTH_CM + STOP_PAD_CM; }
inline int frontSafe()  { return CAR_LENGTH_CM + BRAKE_PAD_CM; }

// 探头退后时给阈值加 |offset|；突出 / 齐平时为 0
inline int sensorBackMargin() {
  return (SENSOR_OFFSET_CM < 0) ? -SENSOR_OFFSET_CM : 0;
}

// 与「测距 cm」比较的阈值（探头退后越多越保守，突出/齐平不补偿）
inline int dStop()    { return frontStop()  + sensorBackMargin(); }
inline int dBrake()   { return frontBrake() + sensorBackMargin(); }
inline int dSafe()    { return frontSafe()  + sensorBackMargin(); }

// 软件滞回（非物理余量）：见 VERSION.md「dClear / dProbeOk 为什么要在 dSafe 上再加余量」
inline int dClear()   { return dSafe() + 10; }  // 直行升全速，防 dSafe 边界反复加减速
inline int dProbeOk() { return dSafe() + 5; }   // 转向探测判通，防转完立刻再 DECIDE

const int MAX_VALID_CM = 400;

// ==================== 避障/脱困参数 ====================

const int PROBE_TURN_DEG_RIGHT = 35;   // 右手法则：先右探一点
const int PROBE_TURN_DEG_LEFT  = 35;   // 不行再左探
const int ESCAPE_TURN_DEG      = 90;   // 后退升档时的大转角

const int MS_BACKUP_BASE       = 350;
const int MS_BACKUP_STEP       = 150;
const int MS_BACKUP_MAX        = 900;

const byte STUCK_MAX_LEVEL     = 3;
const unsigned long STUCK_REPEAT_MS = 2000;
const unsigned long STUCK_RESET_MS  = 3000;

// 抖动抑制
const byte BLOCK_CONFIRM = 2;
const byte CLEAR_CONFIRM = 3;

// ==================== 状态机 ====================

// 兼容老 Arduino 编译器：不使用 C++11 强类型枚举
typedef byte RobotState;
const RobotState ST_IDLE       = 0;
const RobotState ST_FORWARD    = 1;
const RobotState ST_DECIDE     = 2;
const RobotState ST_TURN_PROBE = 3;
const RobotState ST_BACKUP     = 4;
const RobotState ST_ESCAPE     = 5;

static RobotState state = ST_IDLE;
static unsigned long stateEnterMs = 0;

// 运行时状态（含义见 VERSION.md「v2 运行时状态变量」）
static byte stuckLevel = 0;
static int lastTurnDirSign = 1;
static unsigned long lastEscapeMs = 0;
static unsigned long pathClearSinceMs = 0;
static byte blockedStreak = 0;          //连续探测距离小于dBrake的次数，满两次进入DECIDE
static byte clearStreak = 0;           // 预留，当前未参与逻辑
static long lastValidCm = 100;

// 右手法则决策中的临时变量
static byte probePhase = 0;            // 0=未开始 1=右探 2=回正 3=左探 4=完成
static int  probeCommittedTurnDeg = 0; // 已转出的角度，便于回正
static int  probeCommittedTurnDir = 0; // 1=已向右累计，-1=已向左累计

const int MS_LOOP_PAUSE = 30;   // 每圈 loop 末尾停顿；见 VERSION.md「MS_LOOP_PAUSE」

// ==================== 前置声明 ====================

void enterState(RobotState s);
const __FlashStringHelper* stateName(RobotState s);
long readDistanceCmFiltered();
long readDistanceCm();
unsigned long measureEchoUs();
long usToCm(unsigned long us);
void printDistanceDebug();

void driveForward(int speed);
void driveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopAll();
void setMotorLeft(int speed);
void setMotorRight(int speed);
void setMotorChannel(int pinIn1, int pinIn2, int pinPwm, int speed);

void runForward();
void runDecide();
void runTurnProbe();
void runBackup();
void runEscape();
void doTurnInPlace(int dirSign, int deg);

// ==================== setup/loop ====================

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
  Serial.print(F("car_tb6612_avoid "));
  Serial.println(FIRMWARE_VERSION);
  Serial.print(F("car LxWxH="));
  Serial.print(CAR_LENGTH_CM); Serial.print('x');
  Serial.print(CAR_WIDTH_CM); Serial.print('x');
  Serial.println(CAR_HEIGHT_CM);
  Serial.print(F("sensorOffset="));
  Serial.println(SENSOR_OFFSET_CM);
  Serial.print(F("d(cm) Stop/Brake/Safe/Clear/ProbeOk="));
  Serial.print(dStop());    Serial.print('/');
  Serial.print(dBrake());   Serial.print('/');
  Serial.print(dSafe());    Serial.print('/');
  Serial.print(dClear());   Serial.print('/');
  Serial.println(dProbeOk());
#endif

  delay(1000);
  enterState(ST_FORWARD);
}

void loop() {
#if DEBUG_HOLD_FORWARD
  driveForward(255);
  return;
#endif

  switch (state) {
    case ST_FORWARD:    runForward();   break;
    case ST_DECIDE:     runDecide();    break;
    case ST_TURN_PROBE: runTurnProbe(); break;
    case ST_BACKUP:     runBackup();    break;
    case ST_ESCAPE:     runEscape();    break;
    default:            enterState(ST_FORWARD); break;
  }
  delay(MS_LOOP_PAUSE);
}

// ==================== 状态切换辅助 ====================

void enterState(RobotState s) {
  state = s;
  stateEnterMs = millis();

#if SERIAL_DEBUG
  Serial.print(F("ST -> "));
  Serial.println(stateName(s));
#endif

  if (s == ST_FORWARD) {
    blockedStreak = 0;
  } else if (s == ST_DECIDE) {
    probePhase = 0;
    probeCommittedTurnDeg = 0;
    probeCommittedTurnDir = 0;
    stopAll();
  } else if (s == ST_BACKUP || s == ST_ESCAPE) {
    blockedStreak = 0;
    clearStreak = 0;
  }
}

const __FlashStringHelper* stateName(RobotState s) {
  switch (s) {
    case ST_IDLE:       return F("IDLE");
    case ST_FORWARD:    return F("FORWARD");
    case ST_DECIDE:     return F("DECIDE");
    case ST_TURN_PROBE: return F("TURN_PROBE");
    case ST_BACKUP:     return F("BACKUP");
    case ST_ESCAPE:     return F("ESCAPE");
  }
  return F("?");
}

// ==================== FORWARD ====================
// 按距离分段调速；近障升级到 DECIDE；极近直接 BACKUP

void runForward() {
  long cm = readDistanceCmFiltered();
  if (cm > 0) {
    lastValidCm = cm;
  }
  // cm=-1 时用上次有效距离；软物吸波也可能 -1，见 VERSION.md「lastValidCm」
  long d = (cm > 0) ? cm : lastValidCm;

#if SERIAL_DEBUG
  Serial.print(F("FWD cm="));
  Serial.print(cm);
  Serial.print(F(" d="));
  Serial.println(d);
#endif

  // 极近：先后退一段，再决策
  if (d > 0 && d < dStop()) {
    enterState(ST_BACKUP);
    return;
  }

  // 近障：先看是否要进入 DECIDE
  if (d > 0 && d < dBrake()) {
    blockedStreak++;
    if (blockedStreak >= BLOCK_CONFIRM) {
      enterState(ST_DECIDE);
      return;
    }
    driveForward(SPEED_LOW);
    return;
  }
  blockedStreak = 0;

  // 中距：减速直行
  if (d > 0 && d < dSafe()) {
    driveForward(SPEED_LOW);
    pathClearSinceMs = 0;
    return;
  }

  // 路通：判定足够久后复位脱困档位
  if (d >= dClear()) {
    if (pathClearSinceMs == 0) {
      pathClearSinceMs = millis();
    } else if (millis() - pathClearSinceMs >= STUCK_RESET_MS) {
      stuckLevel = 0;
    }
    driveForward(SPEED_HIGH);
  } else {
    driveForward(SPEED_MID);
  }
}

// ==================== DECIDE（右手法则探测）====================
// 阶段：先右探一点测距 → 路通就转过去 → 否则回正测中 → 否则左探 → 否则 ESCAPE

void runDecide() {
  stopAll();
  delay(120);

  long cm = readDistanceCmFiltered();
  if (cm > 0) lastValidCm = cm;

#if SERIAL_DEBUG
  Serial.print(F("DEC ph="));
  Serial.print(probePhase);
  Serial.print(F(" cm="));
  Serial.println(cm);
#endif

  // 极近直接后退
  if (cm > 0 && cm < dStop()) {
    enterState(ST_BACKUP);
    return;
  }

  switch (probePhase) {
    case 0: {
      // 当前前方就够畅通 → 直接回 FORWARD
      if (cm > 0 && cm >= dProbeOk()) {
        enterState(ST_FORWARD);
        return;
      }
      // 先右探（右手法则优先）
      probePhase = 1;
      doTurnInPlace(+1, PROBE_TURN_DEG_RIGHT);
      probeCommittedTurnDir = +1;
      probeCommittedTurnDeg += PROBE_TURN_DEG_RIGHT;
      return;
    }
    case 1: {
      if (cm > 0 && cm >= dProbeOk()) {
        // 右侧可走，确认（保持已转角度）
#if SERIAL_DEBUG
        Serial.println(F("decide=R OK"));
#endif
        probePhase = 4;
        enterState(ST_FORWARD);
        return;
      }
      // 右不通 → 回正
      doTurnInPlace(-probeCommittedTurnDir, probeCommittedTurnDeg);
      probeCommittedTurnDeg = 0;
      probeCommittedTurnDir = 0;
      probePhase = 2;
      return;
    }
    case 2: {
      // 回到原朝向再测一次
      if (cm > 0 && cm >= dProbeOk()) {
#if SERIAL_DEBUG
        Serial.println(F("decide=F OK"));
#endif
        enterState(ST_FORWARD);
        return;
      }
      // 左探
      probePhase = 3;
      doTurnInPlace(-1, PROBE_TURN_DEG_LEFT);
      probeCommittedTurnDir = -1;
      probeCommittedTurnDeg += PROBE_TURN_DEG_LEFT;
      return;
    }
    case 3: {
      if (cm > 0 && cm >= dProbeOk()) {
#if SERIAL_DEBUG
        Serial.println(F("decide=L OK"));
#endif
        enterState(ST_FORWARD);
        return;
      }
      // 三面皆堵 → 后退脱困
#if SERIAL_DEBUG
      Serial.println(F("decide=stuck"));
#endif
      // 回正再脱困（避免转角累计偏一边）
      doTurnInPlace(-probeCommittedTurnDir, probeCommittedTurnDeg);
      probeCommittedTurnDeg = 0;
      probeCommittedTurnDir = 0;
      enterState(ST_ESCAPE);
      return;
    }
  }
}

// ==================== 原地转向（同步版，时间法）====================

void doTurnInPlace(int dirSign, int deg) {
  int ms = turnMsForDegrees(deg);
#if SERIAL_DEBUG
  Serial.print(F("turn "));
  Serial.print(dirSign > 0 ? F("R") : F("L"));
  Serial.print(' ');
  Serial.print(deg);
  Serial.print(F("deg "));
  Serial.print(ms);
  Serial.println(F("ms"));
#endif
  if (dirSign >= 0) turnRight(SPEED_LOW);
  else              turnLeft(SPEED_LOW);
  delay(ms);
  stopAll();
  delay(120);
}

// 状态化的 TURN_PROBE 暂未使用（保留接口，便于以后非阻塞改写）
void runTurnProbe() {
  enterState(ST_DECIDE);
}

// ==================== BACKUP ====================
// 距离极近 → 后退一段固定时间（按 stuckLevel 略加长），再进入 DECIDE

void runBackup() {
  int ms = MS_BACKUP_BASE + (int)stuckLevel * MS_BACKUP_STEP;
  ms = constrain(ms, MS_BACKUP_BASE, MS_BACKUP_MAX);

#if SERIAL_DEBUG
  Serial.print(F("BACK "));
  Serial.print(ms);
  Serial.println(F("ms"));
#endif

  driveBackward(SPEED_LOW);
  delay(ms);
  stopAll();
  delay(150);

  enterState(ST_DECIDE);
}

// ==================== ESCAPE ====================
// 三面皆堵：升档 → 大角度反向转 → 回 DECIDE 再测

void runEscape() {
  unsigned long now = millis();

  if (lastEscapeMs != 0 && (now - lastEscapeMs) < STUCK_REPEAT_MS) {
    if (stuckLevel < STUCK_MAX_LEVEL) stuckLevel++;
  }
  lastEscapeMs = now;

  // 后退多一点
  int backMs = MS_BACKUP_BASE + (int)(stuckLevel + 1) * MS_BACKUP_STEP;
  backMs = constrain(backMs, MS_BACKUP_BASE, MS_BACKUP_MAX);
  driveBackward(SPEED_LOW);
  delay(backMs);
  stopAll();
  delay(150);

  // 反向大角度
  lastTurnDirSign = -lastTurnDirSign;
  int deg = ESCAPE_TURN_DEG + (int)stuckLevel * 20;
  if (deg > MAX_TURN_DEG) deg = MAX_TURN_DEG;
  doTurnInPlace(lastTurnDirSign, deg);

#if SERIAL_DEBUG
  Serial.print(F("ESC lvl="));
  Serial.print(stuckLevel);
  Serial.print(F(" dir="));
  Serial.print(lastTurnDirSign > 0 ? F("R") : F("L"));
  Serial.print(F(" deg="));
  Serial.println(deg);
#endif

  enterState(ST_DECIDE);
}

// ==================== HC-SR04 ====================

unsigned long measureEchoUs() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, 30000UL);
}

long usToCm(unsigned long us) {
  if (us == 0) return -1;
  long cm = (long)(us / 58);
  if (cm < 2 || cm > MAX_VALID_CM) return -1;
  return cm;
}

long readDistanceCm() {
  return usToCm(measureEchoUs());
}

long readDistanceCmFiltered() {
  long a = readDistanceCm(); delay(15);
  long b = readDistanceCm(); delay(15);
  long c = readDistanceCm();

  if (a < 0) a = 999;
  if (b < 0) b = 999;
  if (c < 0) c = 999;

  long m = a;
  if (b < m) m = b;
  if (c < m) m = c;
  return (m >= 999) ? -1L : m;
}

void printDistanceDebug() {
  unsigned long us = measureEchoUs();
  long cm = usToCm(us);
#if SERIAL_DEBUG_RAW
  Serial.print(F("us="));
  Serial.print(us);
  Serial.print(F(" cm="));
  Serial.println(cm);
#else
  Serial.print(F("cm="));
  Serial.println(cm);
#endif
}

// ==================== 电机 ====================

void driveForward(int speed)  { setMotorLeft(speed);  setMotorRight(speed); }
void driveBackward(int speed) { setMotorLeft(-speed); setMotorRight(-speed); }
void turnLeft(int speed)      { setMotorLeft(-speed); setMotorRight(speed); }
void turnRight(int speed)     { setMotorLeft(speed);  setMotorRight(-speed); }
void stopAll()                { setMotorLeft(0);      setMotorRight(0); }

void setMotorLeft(int speed) {
  if (MOTOR_LEFT_DIR_REVERSE) speed = -speed;
  setMotorChannel(PIN_AIN1, PIN_AIN2, PIN_PWMA, speed);
}

void setMotorRight(int speed) {
  if (MOTOR_RIGHT_DIR_REVERSE) speed = -speed;
  setMotorChannel(PIN_BIN1, PIN_BIN2, PIN_PWMB, speed);
}

void setMotorChannel(int pinIn1, int pinIn2, int pinPwm, int speed) {
#if DEBUG_SENSOR_ONLY
  speed = 0;
#else
  speed = constrain(speed, -255, 255);
#endif
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
