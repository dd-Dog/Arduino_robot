#pragma once
/*
 * 算法参数 ALG-1.3 — slug: fsm_pass_weave
 * 文件：params_alg_1_3_cfg_1_0.h
 *
 * 相对 ALG-1.2 cfg-1.1（保留不覆盖）：
 *   - dProbePass(累计探角)：转角越大，要求 cm 越高
 *   - 探通后 WEAVE：沿探角低速短前进 → RECENTER 回正 → FORWARD
 *
 * sketch：uno_tb6612_hc04_avoid_v2_fsm_alg_1_3
 */

#ifndef UNO_TB6612_HC04_H_INCLUDED
#error "Include <uno_tb6612_hc04.h> before params_alg_1_3_cfg_1_0.h"
#endif

#define ALG_PARAMS_ID     "ALG-1.3"
#define ALG_PARAMS_CFG    "cfg-1.0"
#define ALG_PARAMS_SLUG   "fsm_pass_weave"

#define ALG_HAS_DEG_PER_MS_OVERRIDE 1
const float ALG_DEG_PER_MS_AT_SPEED_LOW = 0.118f;

// ==================== 距离余量 PAD ====================

const int BRAKE_PAD_CM  = 12;
const int STOP_PAD_CM   = 6;
const int BACKUP_PAD_CM = 4;

const int DCLEAR_EXTRA_CM   = 10;
const int DPROBEOK_EXTRA_CM = 5;

inline int frontStop()  { return STOP_PAD_CM + BACKUP_PAD_CM; }
inline int frontBrake() { return CAR_WIDTH_CM + STOP_PAD_CM; }
inline int frontSafe()  { return CAR_LENGTH_CM + BRAKE_PAD_CM; }

inline int sensorBackMargin() {
  return (SENSOR_OFFSET_CM < 0) ? -SENSOR_OFFSET_CM : 0;
}

inline int dStop()    { return frontStop()  + sensorBackMargin(); }
inline int dBrake()   { return frontBrake() + sensorBackMargin(); }
inline int dSafe()    { return frontSafe()  + sensorBackMargin(); }
inline int dClear()   { return dSafe() + DCLEAR_EXTRA_CM; }
inline int dProbeOk() { return dSafe() + DPROBEOK_EXTRA_CM; }

const int MAX_TURN_DEG = 180;

// ==================== 阶梯探测 15°×3 → 45° ====================

const byte PROBE_STEP_COUNT = 3;
const int PROBE_STEP_DEG[PROBE_STEP_COUNT] = {15, 15, 15};

inline int probeStepDeltaDeg(byte stepIdx) {
  if (stepIdx >= PROBE_STEP_COUNT) return 0;
  return PROBE_STEP_DEG[stepIdx];
}

inline int probeCumDeg(byte stepIdx) {
  int sum = 0;
  for (byte i = 0; i <= stepIdx && i < PROBE_STEP_COUNT; i++) {
    sum += PROBE_STEP_DEG[i];
  }
  return sum;
}

// ==================== dProbePass：累计探角越大，要求前方 cm 越大 ====================

const int DPASS_AT_0_CM   = 26;
const int DPASS_AT_MAX_CM = 40;

inline int probeMaxCumDeg() {
  return probeCumDeg(PROBE_STEP_COUNT - 1);
}

inline int dProbePass(int cumProbeDeg) {
  if (cumProbeDeg < 0) cumProbeDeg = 0;
  int maxCum = probeMaxCumDeg();
  if (maxCum <= 0) return DPASS_AT_MAX_CM;
  if (cumProbeDeg > maxCum) cumProbeDeg = maxCum;
  long span = (long)(DPASS_AT_MAX_CM - DPASS_AT_0_CM) * (long)cumProbeDeg / (long)maxCum;
  return DPASS_AT_0_CM + (int)span;
}

// ==================== 探通后斜走 + 回正 ====================

const int MS_WEAVE_FORWARD   = 450;   // 沿当前探角低速前进时长
const int SPEED_WEAVE        = 100;   // 略低于 SPEED_LOW，减小侧向扫过

// ==================== 脱困 ====================

const int ESCAPE_TURN_DEG = 60;

const int MS_BACKUP_BASE = 180;
const int MS_BACKUP_STEP = 90;
const int MS_BACKUP_MAX  = 480;

const byte STUCK_MAX_LEVEL = 3;
const unsigned long STUCK_REPEAT_MS = 2000;
const unsigned long STUCK_RESET_MS  = 3000;

const byte BLOCK_CONFIRM = 2;
const byte CLEAR_CONFIRM = 3;
const byte NEAR_CONFIRM  = 2;

const unsigned long BACKUP_COOLDOWN_MS = 1200;
const int MS_LOOP_PAUSE = 30;

inline int msWeaveForCumDeg(int cumProbeDeg) {
  (void)cumProbeDeg;
  return MS_WEAVE_FORWARD;
}

inline int turnMsForAlg(int deg) {
  if (deg <= 0) return 0;
#if defined(ALG_HAS_DEG_PER_MS_OVERRIDE)
  long ms = (long)(deg / ALG_DEG_PER_MS_AT_SPEED_LOW);
#else
  long ms = (long)(deg / DEG_PER_MS_AT_SPEED_LOW);
#endif
  return (int)constrain(ms, 80L, 3000L);
}
