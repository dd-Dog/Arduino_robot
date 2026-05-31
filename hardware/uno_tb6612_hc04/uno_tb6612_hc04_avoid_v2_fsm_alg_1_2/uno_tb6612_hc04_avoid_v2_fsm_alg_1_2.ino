/*
 * 四驱小车 — 避障 v2 + ALG-1.2（阶梯右探 / 左探 + 缩短后退）
 *
 * ALG_ID = "ALG-1.2"  slug: fsm_rhr_step
 * 基线 ALG-1.1 保留在 ..._alg_1_1/，本目录为独立 sketch，不覆盖原文件。
 *
 * 本版相对 1.1：
 *   1. BACKUP / ESCAPE 后退时长缩短（见 params_alg_1_2_cfg_1_0.h）
 *   2. DECIDE：右探 15°→30°→…→90°，用尽回正再左探同样阶梯，再 ESCAPE
 *
 * 规划未实现（见 docs/DEVELOPMENT.md）：dProbePass、斜走回正、过障判据
 *
 * 参数（改一行 #include 切换，勿覆盖 cfg_1_0）：
 *   cfg-1.0 → params_alg_1_2_cfg_1_0.h（阶梯 90°、common 标定 0.105）
 *   cfg-1.1 → params_alg_1_2_cfg_1_1.h（跑道 45°、新电池 DEG 0.118）【当前默认】
 */

#define ALG_ID           "ALG-1.2"
#define ALG_SLUG         "fsm_rhr_step"
#define FIRMWARE_VERSION "v2.2"

#define DEBUG_HOLD_FORWARD 0
#define DEBUG_SENSOR_ONLY  0
#define SERIAL_DEBUG       1
#define SERIAL_DEBUG_RAW   0

#include <uno_tb6612_hc04.h>
#include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_2/params_alg_1_2_cfg_1_1.h"
// #include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_2/params_alg_1_2_cfg_1_0.h"

// ==================== 状态机 ====================

typedef byte RobotState;
const RobotState ST_IDLE       = 0;
const RobotState ST_FORWARD    = 1;
const RobotState ST_DECIDE     = 2;
const RobotState ST_TURN_PROBE = 3;
const RobotState ST_BACKUP     = 4;
const RobotState ST_ESCAPE     = 5;

// DECIDE 内：阶梯探测子相位
const byte PH_DECIDE_ENTRY   = 0;
const byte PH_RIGHT_CHECK    = 1;
const byte PH_CENTER_CHECK   = 2;
const byte PH_LEFT_CHECK     = 3;

static RobotState state = ST_IDLE;
static unsigned long stateEnterMs = 0;

static byte stuckLevel = 0;
static int lastTurnDirSign = 1;
static unsigned long lastEscapeMs = 0;
static unsigned long pathClearSinceMs = 0;
static byte blockedStreak = 0;
static byte clearStreak = 0;
static byte nearStreak = 0;
static unsigned long lastBackupEndMs = 0;
static long lastValidCm = 100;

static byte probePhase = PH_DECIDE_ENTRY;
static byte probeStepIdx = 0;
static int  probeCommittedTurnDeg = 0;
static int  probeCommittedTurnDir = 0;

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
bool backupInCooldown();
bool decideTryForward(long cm, bool allowRightAfterBackup);
bool decideShouldBackupOnNear(long cm);
void resetDecideProbe();
void doProbeStepTurn(int dirSign, byte stepIdx);
void unwindProbeTurn();

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
  Serial.print(F("uno_tb6612_hc04_avoid_v2_fsm_alg_1_2 "));
  Serial.print(ALG_ID);
  Serial.print(F(" "));
  Serial.println(ALG_SLUG);
  Serial.print(F("params "));
  Serial.print(ALG_PARAMS_ID);
  Serial.print(F(" "));
  Serial.println(ALG_PARAMS_CFG);
  Serial.print(F("probe steps="));
  Serial.print(PROBE_STEP_COUNT);
  Serial.print(F(" maxCum="));
  Serial.print(probeCumDeg(PROBE_STEP_COUNT - 1));
  Serial.print(F(" deg/step="));
  Serial.println(PROBE_STEP_DEG[0]);
#if defined(ALG_HAS_DEG_PER_MS_OVERRIDE)
  Serial.print(F("DEG_PER_MS alg="));
  Serial.println(ALG_DEG_PER_MS_AT_SPEED_LOW, 4);
#else
  Serial.print(F("DEG_PER_MS common="));
  Serial.println(DEG_PER_MS_AT_SPEED_LOW, 4);
#endif
  Serial.print(F("BACK ms base/step/max="));
  Serial.print(MS_BACKUP_BASE);
  Serial.print('/');
  Serial.print(MS_BACKUP_STEP);
  Serial.print('/');
  Serial.println(MS_BACKUP_MAX);
  Serial.print(F("d Stop/Brake/Safe/Clear/ProbeOk="));
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

// ==================== 状态切换 ====================

void resetDecideProbe() {
  probePhase = PH_DECIDE_ENTRY;
  probeStepIdx = 0;
  probeCommittedTurnDeg = 0;
  probeCommittedTurnDir = 0;
}

void enterState(RobotState s) {
  state = s;
  stateEnterMs = millis();

#if SERIAL_DEBUG
  Serial.print(F("ST -> "));
  Serial.println(stateName(s));
#endif

  if (s == ST_FORWARD) {
    blockedStreak = 0;
    nearStreak = 0;
  } else if (s == ST_DECIDE) {
    resetDecideProbe();
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

bool backupInCooldown() {
  return lastBackupEndMs != 0
      && (millis() - lastBackupEndMs) < BACKUP_COOLDOWN_MS;
}

bool decideTryForward(long cm, bool allowRightAfterBackup) {
  if (cm <= 0 || cm < dProbeOk()) {
    clearStreak = 0;
    return false;
  }
  if (!allowRightAfterBackup && backupInCooldown()) {
    clearStreak = 0;
    return false;
  }
  clearStreak++;
  if (clearStreak >= CLEAR_CONFIRM) {
    clearStreak = 0;
    enterState(ST_FORWARD);
    return true;
  }
  return false;
}

// 仅 DECIDE 入口极近才 BACKUP；右探回正后应 CENTER→左探，不因 dStop 误倒车
bool decideShouldBackupOnNear(long cm) {
  return cm > 0 && cm < dStop() && probePhase == PH_DECIDE_ENTRY;
}

void doProbeStepTurn(int dirSign, byte stepIdx) {
  int delta = probeStepDeltaDeg(stepIdx);
  if (delta <= 0) return;
  doTurnInPlace(dirSign, delta);
  probeCommittedTurnDir = dirSign;
  probeCommittedTurnDeg += delta;
#if SERIAL_DEBUG
  Serial.print(F("probe "));
  Serial.print(dirSign > 0 ? F("R") : F("L"));
  Serial.print(F(" step="));
  Serial.print(stepIdx);
  Serial.print(F(" +"));
  Serial.print(delta);
  Serial.print(F(" cum="));
  Serial.println(probeCumDeg(stepIdx));
#endif
}

void unwindProbeTurn() {
  if (probeCommittedTurnDeg <= 0) return;
  doTurnInPlace(-probeCommittedTurnDir, probeCommittedTurnDeg);
  probeCommittedTurnDeg = 0;
  probeCommittedTurnDir = 0;
}

// ==================== FORWARD ====================

void runForward() {
  long cm = readDistanceCmFiltered();
  if (cm > 0) {
    lastValidCm = cm;
  }
  long d = (cm > 0) ? cm : lastValidCm;

#if SERIAL_DEBUG
  Serial.print(F("FWD cm="));
  Serial.print(cm);
  Serial.print(F(" d="));
  Serial.println(d);
#endif

  if (d > 0 && d < dStop()) {
    if (backupInCooldown()) {
      nearStreak = 0;
      blockedStreak++;
      if (blockedStreak >= BLOCK_CONFIRM) {
        enterState(ST_DECIDE);
      } else {
        stopAll();
      }
      return;
    }
    nearStreak++;
    if (nearStreak < NEAR_CONFIRM) {
      stopAll();
      return;
    }
    nearStreak = 0;
    enterState(ST_BACKUP);
    return;
  }
  nearStreak = 0;

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

  if (d > 0 && d < dSafe()) {
    driveForward(SPEED_LOW);
    pathClearSinceMs = 0;
    return;
  }

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

// ==================== DECIDE（右探阶梯 → 回正 → 正前测 → 左探阶梯 → ESCAPE）====================
// 极近 BACKUP 仅在 PH_DECIDE_ENTRY；回正后不因 dStop 倒车

void runDecide() {
  stopAll();
  delay(120);

  long cm = readDistanceCmFiltered();
  if (cm > 0) lastValidCm = cm;

#if SERIAL_DEBUG
  Serial.print(F("DEC ph="));
  Serial.print(probePhase);
  Serial.print(F(" st="));
  Serial.print(probeStepIdx);
  Serial.print(F(" cm="));
  Serial.println(cm);
#endif

  if (decideShouldBackupOnNear(cm)) {
    enterState(ST_BACKUP);
    return;
  }

  switch (probePhase) {
    case PH_DECIDE_ENTRY: {
      if (decideTryForward(cm, false)) {
        return;
      }
      if (cm > 0 && cm >= dProbeOk() && !backupInCooldown()) {
        stopAll();
        return;
      }
      probeStepIdx = 0;
      doProbeStepTurn(+1, probeStepIdx);
      probePhase = PH_RIGHT_CHECK;
      return;
    }

    case PH_RIGHT_CHECK: {
      if (decideTryForward(cm, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=R step OK"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbeOk()) {
        stopAll();
        return;
      }
      probeStepIdx++;
      if (probeStepIdx < PROBE_STEP_COUNT) {
        doProbeStepTurn(+1, probeStepIdx);
        return;
      }
      unwindProbeTurn();
      probeStepIdx = 0;
      probePhase = PH_CENTER_CHECK;
      return;
    }

    case PH_CENTER_CHECK: {
      if (decideTryForward(cm, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=F OK"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbeOk()) {
        stopAll();
        return;
      }
      probeStepIdx = 0;
      doProbeStepTurn(-1, probeStepIdx);
      probePhase = PH_LEFT_CHECK;
      return;
    }

    case PH_LEFT_CHECK: {
      if (decideTryForward(cm, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=L step OK"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbeOk()) {
        stopAll();
        return;
      }
      probeStepIdx++;
      if (probeStepIdx < PROBE_STEP_COUNT) {
        doProbeStepTurn(-1, probeStepIdx);
        return;
      }
#if SERIAL_DEBUG
      Serial.println(F("decide=stuck"));
#endif
      unwindProbeTurn();
      enterState(ST_ESCAPE);
      return;
    }
  }
}

// ==================== 转向 / BACKUP / ESCAPE ====================

void doTurnInPlace(int dirSign, int deg) {
  int ms = turnMsForAlg(deg);
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

void runTurnProbe() {
  enterState(ST_DECIDE);
}

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

  lastBackupEndMs = millis();
  enterState(ST_DECIDE);
}

void runEscape() {
  unsigned long now = millis();

  if (lastEscapeMs != 0 && (now - lastEscapeMs) < STUCK_REPEAT_MS) {
    if (stuckLevel < STUCK_MAX_LEVEL) stuckLevel++;
  }
  lastEscapeMs = now;

  int backMs = MS_BACKUP_BASE + (int)(stuckLevel + 1) * MS_BACKUP_STEP;
  backMs = constrain(backMs, MS_BACKUP_BASE, MS_BACKUP_MAX);
  driveBackward(SPEED_LOW);
  delay(backMs);
  stopAll();
  delay(150);

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

  lastBackupEndMs = millis();
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
