/*
 * 四驱小车 — 避障 v2 + ALG-1.3（dProbePass + 斜走回正）
 *
 * ALG_ID = "ALG-1.3"  slug: fsm_pass_weave
 * 基线 ALG-1.2 保留在 ..._alg_1_2/，不覆盖。
 *
 * 相对 1.2：
 *   1. 探测判通：dProbePass(累计探角)，转角越大要求 cm 越大
 *   2. 探通后：WEAVE 低速短前进 → RECENTER 撤销探角 → FORWARD
 *      （正前直通 cum=0 时跳过 WEAVE/RECENTER）
 *
 * 参数（#include 切换，勿覆盖 cfg_1_0）：
 *   cfg-1.0 → 固定 WEAVE 450ms
 *   cfg-1.1 → 按累计探角加长 WEAVE【当前默认】
 */

#define ALG_ID           "ALG-1.3"
#define ALG_SLUG         "fsm_pass_weave"
#define FIRMWARE_VERSION "v2.3"

#define DEBUG_HOLD_FORWARD 0
#define DEBUG_SENSOR_ONLY  0
#define SERIAL_DEBUG       1
#define SERIAL_DEBUG_RAW   0

#include <uno_tb6612_hc04.h>
#include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_3/params_alg_1_3_cfg_1_1.h"
// #include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_3/params_alg_1_3_cfg_1_0.h"

// ==================== 状态机 ====================

typedef byte RobotState;
const RobotState ST_IDLE       = 0;
const RobotState ST_FORWARD    = 1;
const RobotState ST_DECIDE     = 2;
const RobotState ST_TURN_PROBE = 3;
const RobotState ST_BACKUP     = 4;
const RobotState ST_ESCAPE     = 5;
const RobotState ST_WEAVE      = 6;
const RobotState ST_RECENTER   = 7;

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

static int weaveRecenterDir = 0;
static int weaveRecenterDeg = 0;

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
void runWeave();
void runRecenter();
void doTurnInPlace(int dirSign, int deg);
bool backupInCooldown();
bool decideTryPass(long cm, int cumProbeDeg, bool allowRightAfterBackup);
bool decideShouldBackupOnNear(long cm);
void resetDecideProbe();
void doProbeStepTurn(int dirSign, byte stepIdx);
void unwindProbeTurn();
void beginPassWeaveRecenter();
int probeCumForPhase();

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
  Serial.print(F("uno_tb6612_hc04_avoid_v2_fsm_alg_1_3 "));
  Serial.print(ALG_ID);
  Serial.print(F(" "));
  Serial.println(ALG_SLUG);
  Serial.print(F("params "));
  Serial.print(ALG_PARAMS_ID);
  Serial.print(F(" "));
  Serial.println(ALG_PARAMS_CFG);
  Serial.print(F("probe maxCum="));
  Serial.println(probeMaxCumDeg());
  Serial.print(F("dPass 0/max="));
  Serial.print(dProbePass(0));
  Serial.print('/');
  Serial.println(dProbePass(probeMaxCumDeg()));
  Serial.print(F("dProbeOk="));
  Serial.println(dProbeOk());
  Serial.print(F("WEAVE 30deg ms="));
  Serial.println(msWeaveForCumDeg(30));
#if defined(ALG_HAS_DEG_PER_MS_OVERRIDE)
  Serial.print(F("DEG_PER_MS="));
  Serial.println(ALG_DEG_PER_MS_AT_SPEED_LOW, 4);
#endif
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
    case ST_WEAVE:      runWeave();     break;
    case ST_RECENTER:   runRecenter();  break;
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
    weaveRecenterDeg = 0;
    weaveRecenterDir = 0;
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
    case ST_WEAVE:      return F("WEAVE");
    case ST_RECENTER:   return F("RECENTER");
  }
  return F("?");
}

bool backupInCooldown() {
  return lastBackupEndMs != 0
      && (millis() - lastBackupEndMs) < BACKUP_COOLDOWN_MS;
}

int probeCumForPhase() {
  if (probePhase == PH_CENTER_CHECK || probePhase == PH_DECIDE_ENTRY) {
    return 0;
  }
  return probeCommittedTurnDeg;
}

void beginPassWeaveRecenter() {
  if (probeCommittedTurnDeg <= 0) {
    enterState(ST_FORWARD);
    return;
  }
  weaveRecenterDir = probeCommittedTurnDir;
  weaveRecenterDeg = probeCommittedTurnDeg;
  probeCommittedTurnDeg = 0;
  probeCommittedTurnDir = 0;
  enterState(ST_WEAVE);
}

bool decideTryPass(long cm, int cumProbeDeg, bool allowRightAfterBackup) {
  int thr = dProbePass(cumProbeDeg);

  if (cm <= 0 || cm < thr) {
    clearStreak = 0;
    return false;
  }
  if (!allowRightAfterBackup && backupInCooldown()) {
    clearStreak = 0;
    return false;
  }

  clearStreak++;
#if SERIAL_DEBUG
  if (clearStreak == 1) {
    Serial.print(F("pass thr="));
    Serial.print(thr);
    Serial.print(F(" cum="));
    Serial.println(cumProbeDeg);
  }
#endif

  if (clearStreak >= CLEAR_CONFIRM) {
    clearStreak = 0;
    beginPassWeaveRecenter();
    return true;
  }
  return false;
}

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
  Serial.print(probeCommittedTurnDeg);
  Serial.print(F(" passThr="));
  Serial.println(dProbePass(probeCommittedTurnDeg));
#endif
}

void unwindProbeTurn() {
  if (probeCommittedTurnDeg <= 0) return;
  doTurnInPlace(-probeCommittedTurnDir, probeCommittedTurnDeg);
  probeCommittedTurnDeg = 0;
  probeCommittedTurnDir = 0;
}

// ==================== WEAVE / RECENTER ====================

void runWeave() {
  int wms = msWeaveForCumDeg(weaveRecenterDeg);

#if SERIAL_DEBUG
  Serial.print(F("WEAVE "));
  Serial.print(wms);
  Serial.print(F("ms cum="));
  Serial.print(weaveRecenterDeg);
  Serial.print(F(" spd="));
  Serial.println(SPEED_WEAVE);
#endif

#if !DEBUG_SENSOR_ONLY
  driveForward(SPEED_WEAVE);
  delay(wms);
  stopAll();
  delay(80);
#endif

  enterState(ST_RECENTER);
}

void runRecenter() {
  if (weaveRecenterDeg <= 0) {
    enterState(ST_FORWARD);
    return;
  }

#if SERIAL_DEBUG
  Serial.print(F("RECENTER "));
  Serial.print(weaveRecenterDir > 0 ? F("R") : F("L"));
  Serial.print(' ');
  Serial.println(weaveRecenterDeg);
#endif

#if !DEBUG_SENSOR_ONLY
  doTurnInPlace(-weaveRecenterDir, weaveRecenterDeg);
#endif

  weaveRecenterDeg = 0;
  weaveRecenterDir = 0;
  enterState(ST_FORWARD);
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

// ==================== DECIDE ====================

void runDecide() {
  stopAll();
  delay(120);

  long cm = readDistanceCmFiltered();
  if (cm > 0) lastValidCm = cm;

  int cum = probeCumForPhase();

#if SERIAL_DEBUG
  Serial.print(F("DEC ph="));
  Serial.print(probePhase);
  Serial.print(F(" cum="));
  Serial.print(cum);
  Serial.print(F(" cm="));
  Serial.println(cm);
#endif

  if (decideShouldBackupOnNear(cm)) {
    enterState(ST_BACKUP);
    return;
  }

  switch (probePhase) {
    case PH_DECIDE_ENTRY: {
      if (decideTryPass(cm, 0, false)) {
        return;
      }
      if (cm > 0 && cm >= dProbePass(0) && !backupInCooldown()) {
        stopAll();
        return;
      }
      probeStepIdx = 0;
      doProbeStepTurn(+1, probeStepIdx);
      probePhase = PH_RIGHT_CHECK;
      return;
    }

    case PH_RIGHT_CHECK: {
      if (decideTryPass(cm, probeCommittedTurnDeg, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=R pass"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbePass(probeCommittedTurnDeg)) {
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
      if (decideTryPass(cm, 0, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=F pass"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbePass(0)) {
        stopAll();
        return;
      }
      probeStepIdx = 0;
      doProbeStepTurn(-1, probeStepIdx);
      probePhase = PH_LEFT_CHECK;
      return;
    }

    case PH_LEFT_CHECK: {
      if (decideTryPass(cm, probeCommittedTurnDeg, true)) {
#if SERIAL_DEBUG
        Serial.println(F("decide=L pass"));
#endif
        return;
      }
      if (cm > 0 && cm >= dProbePass(probeCommittedTurnDeg)) {
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
