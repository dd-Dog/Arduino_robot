# Arduino_robot

Arduino 四驱小车示例，使用 **一块 TB6612FNG**（A/B 两路）驱动左右两侧电机。

## 工程结构

```
car_tb6612/
  car_tb6612.ino   # 主程序：前进 / 后退 / 左转 / 右转
```

## 硬件

- Arduino Uno
- TB6612FNG × 1（双路）
- 四驱底盘：左侧两电机接 A 路（AO1/AO2），右侧两电机接 B 路（BO1/BO2）
- VM：电池 4.5~10V；VCC：5V；GND 与 Uno 共地

## 引脚（默认）

| TB6612 | Arduino |
|--------|---------|
| PWMA   | D5      |
| AIN1   | D4      |
| AIN2   | D3      |
| PWMB   | D6      |
| BIN1   | D7      |
| BIN2   | D8      |
| STBY   | D10     |

## 使用

1. Arduino IDE 打开 `car_tb6612` 文件夹
2. 开发板选 **Arduino Uno**，上传
3. **架空车轮** 试车；方向反了改程序里 `MOTOR_*_DIR_REVERSE`

## 仓库

https://github.com/dd-Dog/Arduino_robot
