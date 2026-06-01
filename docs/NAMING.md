# 目录与文件命名约定

## 层级

```
hardware/
└── <平台ID>/                          ← 一种硬件（如 uno_tb6612_hc04）
    └── <平台ID>_<结构slug>_alg_<主>_<次>/
        └── <同名>.ino
```

## 平台 ID：`<MCU>_<驱动>_<传感器>`

**当前：** `uno_tb6612_hc04`（Uno + TB6612 + 固定 HC-SR04）

## 算法版本号 `ALG-{主}.{次}`

| 段位 | 含义 | 何时 +1 |
|------|------|---------|
| **主版本（1、2、3…）** | **不同硬件平台** | 换 MCU、换驱动板、换传感器组合 → 新 `hardware/<新平台ID>/`，从 `x.0` 起 |
| **次版本（.0、.1、.2…）** | **同一硬件上的不同算法** | 新避障策略、新状态机、同平台下调参大改 → 新 sketch 文件夹 `_alg_主_次` |

**示例：**

| 硬件平台 | ALG 主版本 | 本平台算法（次版本） |
|----------|------------|----------------------|
| `uno_tb6612_hc04` | **1** | 1.0 反应式、**1.1** 状态机… |
| `uno_tb6612_servo180`（规划） | **2** | 2.0 扫描 + 状态机… |

**后缀规则：** `ALG-1.1` → 文件夹后缀 `_alg_1_1`（点改下划线）。

## 本平台 sketch 对照

| ALG ID | Sketch 文件夹 | 结构 slug |
|--------|---------------|-----------|
| — | `uno_tb6612_hc04_demo_motor` | 电机演示（无 ALG） |
| **ALG-1.0** | `uno_tb6612_hc04_avoid_v1_alg_1_0` | `reactive_alt` |
| **ALG-1.1** | `uno_tb6612_hc04_avoid_v2_fsm_alg_1_1` | `fsm_rhr` |
| **ALG-1.2** | `uno_tb6612_hc04_avoid_v2_fsm_alg_1_2` | `fsm_rhr_step` |
| **ALG-1.3** | `uno_tb6612_hc04_avoid_v2_fsm_alg_1_3` | `fsm_pass_weave` |
| — | `uno_tb6612_hc04_cal_turn` | 转向标定（无 ALG） |

**新增同硬件算法：** 次版本 +1，如 ALG-1.2 → `..._avoid_v2_fsm_alg_1_2`。

**新增硬件：** 新平台目录 + 新主版本，如 `hardware/uno_tb6612_servo180/..._alg_2_0`。

## 参数文件（`libraries/uno_tb6612_hc04/`）

与 sketch 分离，分两层目录：

| 层级 | 路径 | 示例 |
|------|------|------|
| **平台公用** | `common_1_0/*.h` | `pins.h`、`chassis.h`、`motion.h` |
| **算法专用** | `<sketch名>/params_alg_<主>_<次>_cfg_<参数集>.h` | `params_alg_1_1_cfg_1_0.h`、`params_alg_1_2_cfg_1_0.h` |

- `alg_1_1`：对应 **ALG-1.1**（与 sketch 文件夹后缀一致）
- `cfg_1_0`：同一算法下**第 1 套**调参（对比时复制为 `cfg_1_1`，或换 `#include`）

sketch 引用：

```cpp
#include <uno_tb6612_hc04.h>
#include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_1/params_alg_1_1_cfg_1_0.h"
```

详见 [CONFIG.md](CONFIG.md)。

## 代码（sketch 内宏）

## Arduino IDE

打开 sketch **文件夹**（与 `.ino` 同名），如 `uno_tb6612_hc04_avoid_v2_fsm_alg_1_3`（当前）或 `..._alg_1_2` / `..._alg_1_1`。

## 实验记录

```
平台=uno_tb6612_hc04 | ALG-1.1 fsm_rhr | params=cfg-1.0 | 结论…
```
