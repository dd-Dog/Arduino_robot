#pragma once
/*
 * 算法参数 ALG-1.1 — slug: fsm_rhr
 * 文件：params_alg_1_1_cfg_1_0.h
 *       ALG-1.1 算法参数，cfg_1_0 = 本参数集第 1 版（调参对比可升 cfg_1_1）
 *
 * 用于 sketch：uno_tb6612_hc04_avoid_v2_fsm_alg_1_1
 * 平台公用：先 #include <uno_tb6612_hc04.h>  （→ common_1_0/）
 *
 * 路径：libraries/uno_tb6612_hc04/uno_tb6612_hc04_avoid_v2_fsm_alg_1_1/
 * 对比：复制本文件为 params_alg_1_1_cfg_1_1.h 或新建 alg_1_2 目录
 */

#ifndef UNO_TB6612_HC04_H_INCLUDED
#error "Include <uno_tb6612_hc04.h> before params_alg_1_1_cfg_1_0.h"
#endif

#define ALG_PARAMS_ID     "ALG-1.1"
#define ALG_PARAMS_CFG    "cfg-1.0"
#define ALG_PARAMS_SLUG   "fsm_rhr"

// ==================== 距离余量 PAD（推导 dStop/dBrake/dSafe）====================

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

// ==================== 探测 / 脱困 ====================

const int PROBE_TURN_DEG_RIGHT = 35;
const int PROBE_TURN_DEG_LEFT  = 35;
const int ESCAPE_TURN_DEG      = 90;

const int MS_BACKUP_BASE = 350;
const int MS_BACKUP_STEP = 150;
const int MS_BACKUP_MAX  = 900;

const byte STUCK_MAX_LEVEL = 3;
const unsigned long STUCK_REPEAT_MS = 2000;
const unsigned long STUCK_RESET_MS  = 3000;

// ==================== 抖动抑制 / 时序 ====================

const byte BLOCK_CONFIRM = 2;
const byte CLEAR_CONFIRM = 3;
const byte NEAR_CONFIRM  = 2;

const unsigned long BACKUP_COOLDOWN_MS = 1200;
const int MS_LOOP_PAUSE = 30;
