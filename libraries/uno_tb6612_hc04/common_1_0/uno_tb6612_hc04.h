#pragma once
/*
 * 平台公用参数 — uno_tb6612_hc04
 *
 * 在 sketch 顶部：
 *   #include <uno_tb6612_hc04.h>
 *
 * 电机方向与默认值不同时，在 #include 之前：
 *   #define UNO_TB6612_MOTOR_LEFT_REVERSE  1
 *
 * 路径：libraries/uno_tb6612_hc04/common_1_0/
 */

#include <Arduino.h>

#include "pins.h"
#include "motor_dir.h"
#include "chassis.h"
#include "motion.h"

#define UNO_TB6612_HC04_H_INCLUDED 1
