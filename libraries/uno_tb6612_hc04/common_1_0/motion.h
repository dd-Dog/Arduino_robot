#pragma once
// 速度与转向标定。改 DEG_PER_MS 后请用 uno_tb6612_hc04_cal_turn 复测。

const int SPEED_LOW  = 120;
const int SPEED_MID  = 160;
const int SPEED_HIGH = 200;

// 落地标定：木地板≈0.1125，瓷砖≈0.10，折中 0.105（见 cal_turn）
const float DEG_PER_MS_AT_SPEED_LOW = 0.105f;

const int MAX_VALID_CM = 400;

inline int turnMsForDegrees(int deg) {
  if (deg <= 0) return 0;
  long ms = (long)(deg / DEG_PER_MS_AT_SPEED_LOW);
  return (int)constrain(ms, 80L, 3000L);
}
