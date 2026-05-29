# 参数配置说明

## 1. 平台公用 — `libraries/uno_tb6612_hc04/common_1_0/`

| 文件 | 内容 |
|------|------|
| `common_1_0/pins.h` | 引脚 |
| `common_1_0/motor_dir.h` | 电机正反转 |
| `common_1_0/chassis.h` | 车身尺寸、`SENSOR_OFFSET_CM` |
| `common_1_0/motion.h` | `SPEED_*`、`DEG_PER_MS` |

sketch **不要**写子路径，只用库名 include（库根 `uno_tb6612_hc04.h` 会转发到 `common_1_0/`）：

```cpp
#include <uno_tb6612_hc04.h>   // ✓ 正确
// #include "uno_tb6612_hc04/common_1_0/..."  ✗ 会编译失败
```

## 2. 算法专用 — `libraries/uno_tb6612_hc04/<sketch名>/`

```
libraries/uno_tb6612_hc04/
├── uno_tb6612_hc04.h              ← 转发到 common_1_0/
├── common_1_0/                    ← 平台公用 cfg-1.0
└── uno_tb6612_hc04_avoid_v2_fsm_alg_1_1/
    └── params_alg_1_1_cfg_1_0.h   ← ALG-1.1 参数集 cfg 1.0
```

命名：`params_alg_<主>_<次>_cfg_<参数集版本>.h`

- `alg_1_1`：对应 ALG-1.1
- `cfg_1_0`：同一算法下第 1 套参数（对比时复制为 `cfg_1_1` 或改数值另存）

```cpp
#include <uno_tb6612_hc04.h>
#include "uno_tb6612_hc04_avoid_v2_fsm_alg_1_1/params_alg_1_1_cfg_1_0.h"
```

头文件内：`ALG_PARAMS_ID`、`ALG_PARAMS_CFG`、`ALG_PARAMS_SLUG`。

## 库路径

Sketchbook = 仓库根 `robotic_01`（含 `libraries/`）。

## 改参数后

| 改了什么 | 建议 |
|----------|------|
| `common_1_0/motion.h` | 跑 `cal_turn` |
| `common_1_0/chassis.h` | 看 v2 串口 dStop/dBrake… |
| `params_alg_*_cfg_*.h` | 只重编对应 sketch |
| `common_1_0/pins.h` | 全平台冒烟 |
