#pragma once
/*
 * 算法参数 ALG-1.2 — slug: fsm_rhr_step
 * 文件：params_alg_1_2_cfg_1_0.h
 *
 * 相对 ALG-1.1 / cfg_1_0：
 *   - 缩短 BACKUP / ESCAPE 后退时长
 *   - 右探 / 左探改为阶梯角（默认 15°×6 档，累计至 90°）
 *
 * sketch：uno_tb6612_hc04_avoid_v2_fsm_alg_1_2
 */

#ifndef UNO_TB6612_HC04_H_INCLUDED
#error "Include <uno_tb6612_hc04.h> before params_alg_1_2_cfg_1_0.h"
#endif

#define ALG_PARAMS_ID     "ALG-1.2"
#define ALG_PARAMS_CFG    "cfg-1.0"
#define ALG_PARAMS_SLUG   "fsm_rhr_step"

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

// ==================== 阶梯探测（每档增量角，累计 15→90）====================

const byte PROBE_STEP_COUNT = 6;
const int PROBE_STEP_DEG[PROBE_STEP_COUNT] = {15, 15, 15, 15, 15, 15};

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

// ==================== 脱困（后退缩短；大角仍留给 ESCAPE）====================

const int ESCAPE_TURN_DEG = 90;

const int MS_BACKUP_BASE = 200;
const int MS_BACKUP_STEP = 100;
const int MS_BACKUP_MAX  = 550;

const byte STUCK_MAX_LEVEL = 3;
const unsigned long STUCK_REPEAT_MS = 2000;
const unsigned long STUCK_RESET_MS  = 3000;

// ==================== 抖动抑制 / 时序 ====================

const byte BLOCK_CONFIRM = 2;
const byte CLEAR_CONFIRM = 3;
const byte NEAR_CONFIRM  = 2;

const unsigned long BACKUP_COOLDOWN_MS = 1200;
const int MS_LOOP_PAUSE = 30;

// cfg_1_0 使用 common_1_0/motion.h 的 turnMsForDegrees
inline int turnMsForAlg(int deg) {
  return turnMsForDegrees(deg);
}
