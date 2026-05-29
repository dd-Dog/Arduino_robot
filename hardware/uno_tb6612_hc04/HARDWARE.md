# 硬件平台：`uno_tb6612_hc04`

| 项 | 规格 |
|----|------|
| 平台 ID | `uno_tb6612_hc04` |
| **ALG 主版本** | **1**（本硬件平台下算法均为 ALG-1.x） |
| 主控 | Arduino Uno |
| 驱动 | 1× TB6612FNG（A/B 双路，四驱并联） |
| 测距 | 1× HC-SR04，**固定朝前**（Trig D9，Echo D12） |
| 供电 | 2×18650 串联 → 电源模块 7~10V → 5V / VM |

完整接线、供电、安装说明见仓库根目录 [README.md](../../README.md)。

## 引脚占用（本平台）

| 引脚 | 功能 |
|------|------|
| D3~D8 | 电机方向 / PWM |
| D9 | HC-SR04 Trig |
| D10 | TB6612 STBY |
| D12 | HC-SR04 Echo |

## 本平台 sketch 列表

| 文件夹 | 用途 |
|--------|------|
| `uno_tb6612_hc04_demo_motor` | 电机演示 |
| `uno_tb6612_hc04_avoid_v1_alg_1_0` | 避障 ALG-1.0 |
| `uno_tb6612_hc04_avoid_v2_fsm_alg_1_1` | 避障 ALG-1.1（状态机） |
| `uno_tb6612_hc04_cal_turn` | 转向标定辅助 |

算法说明与对比见 [VERSION.md](./VERSION.md)；命名规则见 [docs/NAMING.md](../../docs/NAMING.md)。
