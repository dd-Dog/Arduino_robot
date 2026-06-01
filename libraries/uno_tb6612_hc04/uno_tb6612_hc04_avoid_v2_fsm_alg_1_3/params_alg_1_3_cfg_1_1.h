#pragma once
/*
 * 算法参数 ALG-1.3 — cfg-1.1（加长 WEAVE）
 * 文件：params_alg_1_3_cfg_1_1.h
 *
 * 相对 cfg-1_0（保留不覆盖）：
 *   左探通过后 WEAVE 太短 → RECENTER 时右侧仍蹭柱；
 *   改为 MS_WEAVE = BASE + 累计探角×PER_DEG（下限/上限钳位）。
 *
 * 实车：单障 ×5 全过但回正蹭右侧 → 本参数集加长斜走。
 */

#ifndef UNO_TB6612_HC04_H_INCLUDED
#error "Include <uno_tb6612_hc04.h> before params_alg_1_3_cfg_1_1.h"
#endif

#define ALG_PARAMS_ID     "ALG-1.3"
#define ALG_PARAMS_CFG    "cfg-1.1"
#define ALG_PARAMS_SLUG   "fsm_pass_weave"

#define ALG_HAS_DEG_PER_MS_OVERRIDE 1
const float ALG_DEG_PER_MS_AT_SPEED_LOW = 0.118f;

#define ALG_WEAVE_BY_CUM_DEG 1

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

// WEAVE：探角越大多走一会，让车头+车身越过柱后再 RECENTER
const int MS_WEAVE_BASE       = 320;
const int MS_WEAVE_PER_DEG    = 14;   // 30°→约 740ms，45°→约 950ms
const int MS_WEAVE_MIN        = 550;
const int MS_WEAVE_MAX        = 950;
const int SPEED_WEAVE         = 100;

inline int msWeaveForCumDeg(int cumProbeDeg) {
  if (cumProbeDeg < 0) cumProbeDeg = 0;
  long ms = (long)MS_WEAVE_BASE + (long)cumProbeDeg * (long)MS_WEAVE_PER_DEG;
  return (int)constrain(ms, (long)MS_WEAVE_MIN, (long)MS_WEAVE_MAX);
}

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

inline int turnMsForAlg(int deg) {
  if (deg <= 0) return 0;
#if defined(ALG_HAS_DEG_PER_MS_OVERRIDE)
  long ms = (long)(deg / ALG_DEG_PER_MS_AT_SPEED_LOW);
#else
  long ms = (long)(deg / DEG_PER_MS_AT_SPEED_LOW);
#endif
  return (int)constrain(ms, 80L, 3000L);
}
