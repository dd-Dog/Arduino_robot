#pragma once
// 车身尺寸与探头安装（用于 v2 阈值推导；详见 VERSION.md）

const int CAR_LENGTH_CM = 26;   // 车头到车尾
const int CAR_WIDTH_CM  = 16;
const int CAR_HEIGHT_CM = 12;   // 含雷达高度，仅供记录

const int SENSOR_OFFSET_CM = -1; // 正=探头突出车头，负=退后
