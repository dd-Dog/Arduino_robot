#pragma once
/*
 * 算法参数 ALG-1.2 — cfg-1.1（跑道 / 新电池）
 * 文件：params_alg_1_2_cfg_1_1.h
 *
 * 相对 cfg_1_0（保留不覆盖）：
 *   - 右/左阶梯探测：3 档 ×15°，单侧最大 45°（避免 90° 横出跑道）
 *   - 新电池转角偏大：覆盖 DEG_PER_MS（0.118，约 +12% → 同样目标角缩短 ms）
 *   - ESCAPE 转角 60°；后退略缩短（高压下同样 ms 退更远）
 *
 * 切换：sketch 内 #include 本文件或 params_alg_1_2_cfg_1_0.h
 * 复测：uno_tb6612_hc04_cal_turn，按实车改 ALG_DEG_PER_MS_AT_SPEED_LOW
 */

#ifndef UNO_TB6612_HC04_H_INCLUDED
#error "Include <uno_tb6612_hc04.h> before params_alg_1_2_cfg_1_1.h"
#endif

#define ALG_PARAMS_ID     "ALG-1.2"
#define ALG_PARAMS_CFG    "cfg-1.1"
#define ALG_PARAMS_SLUG   "fsm_rhr_step"

// 本参数集覆盖 common_1_0/motion.h 的转向标定（仅本 sketch 使用 turnMsForAlg）
#define ALG_HAS_DEG_PER_MS_OVERRIDE 1
const float ALG_DEG_PER_MS_AT_SPEED_LOW = 0.118f;

// ==================== 距离余量 PAD（与 cfg_1_0 相同）====================

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

// ==================== 阶梯探测：15°×3 → 单侧最大 45° ====================

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

// ==================== 脱困 ====================

const int ESCAPE_TURN_DEG = 60;

const int MS_BACKUP_BASE = 180;
const int MS_BACKUP_STEP = 90;
const int MS_BACKUP_MAX  = 480;

const byte STUCK_MAX_LEVEL = 3;
const unsigned long STUCK_REPEAT_MS = 2000;
const unsigned long STUCK_RESET_MS  = 3000;

// ==================== 抖动抑制 / 时序 ====================

const byte BLOCK_CONFIRM = 2;
const byte CLEAR_CONFIRM = 3;
const byte NEAR_CONFIRM  = 2;

const unsigned long BACKUP_COOLDOWN_MS = 1200;
const int MS_LOOP_PAUSE = 30;

// 供 sketch 中 doTurnInPlace 使用（cfg_1_0 仍用 motion.h 的 turnMsForDegrees）
inline int turnMsForAlg(int deg) {
  if (deg <= 0) return 0;
#if defined(ALG_HAS_DEG_PER_MS_OVERRIDE)
  long ms = (long)(deg / ALG_DEG_PER_MS_AT_SPEED_LOW);
#else
  long ms = (long)(deg / DEG_PER_MS_AT_SPEED_LOW);
#endif
  return (int)constrain(ms, 80L, 3000L);
}
