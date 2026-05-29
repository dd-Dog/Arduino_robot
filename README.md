# Arduino_robot



UNO 最大的问题是什么？



| 项目 | 限制 |

|------|------|

| RAM | 2KB |

| Flash | 32KB |

| CPU | 16MHz AVR |



所以：**控制类**任务 UNO 够用；**数据处理类**（多传感器、复杂算法）会越来越吃力。



四驱小车学习工程：一块 **TB6612FNG**（双路）驱动左右电机，Arduino Uno 控制。  

演示版已在实车调试通过（前进 / 后退 / 左转 / 右转）；避障版含 HC-SR04 与自适应脱困，已实车可跑。



**仓库：** https://github.com/dd-Dog/Arduino_robot



---



## 工程结构

命名规则见 [docs/NAMING.md](docs/NAMING.md)：**硬件平台**与**算法 sketch** 分开目录；引脚/车身/速度等公用参数见 [docs/CONFIG.md](docs/CONFIG.md)（`libraries/uno_tb6612_hc04`）。

```
Arduino_robot/                 ← 仓库根（GitHub 同名；本地文件夹可仍为 robotic_01，Cursor 打开父目录即可）

├── README.md
├── docs/
│   ├── NAMING.md              ← 目录/文件命名约定
│   ├── CONFIG.md              ← 公用参数库说明
│   └── DEVELOPMENT.md         ← 开发路线
├── libraries/
│   └── uno_tb6612_hc04/       ← common_1_0/ 公用 + 各算法 params_alg_*_cfg_*.h
└── hardware/
    └── uno_tb6612_hc04/       ← 平台：Uno + TB6612 + 固定 HC-SR04
        ├── HARDWARE.md        ← 本平台接线索引
        ├── VERSION.md         ← 本平台算法说明与对比
        ├── uno_tb6612_hc04_demo_motor/      ← 电机演示
        ├── uno_tb6612_hc04_avoid_v1_alg_1_0/         ← 避障 v1
        ├── uno_tb6612_hc04_avoid_v2_fsm_alg_1_1/     ← 避障 v2（状态机，当前主力）
        └── uno_tb6612_hc04_cal_turn/        ← 转向标定辅助
```

| 用途 | Arduino IDE 打开的 sketch 文件夹 |
|------|----------------------------------|
| **电机演示** | `hardware/uno_tb6612_hc04/uno_tb6612_hc04_demo_motor` |
| **避障 v1** | `hardware/uno_tb6612_hc04/uno_tb6612_hc04_avoid_v1_alg_1_0` |
| **避障 v2** | `hardware/uno_tb6612_hc04/uno_tb6612_hc04_avoid_v2_fsm_alg_1_1` |
| **转向标定** | `hardware/uno_tb6612_hc04/uno_tb6612_hc04_cal_turn` |

换算法 = 在 IDE 里换打开的文件夹再上传，**无需 git 回退**。新增硬件在 `hardware/` 下新建平台目录；新增算法在同一平台下新建 `uno_tb6612_hc04_<算法名>/`。



---



## 主要硬件



| 部件 | 数量 | 说明 |

|------|------|------|

| Arduino Uno | 1 | 主控 |

| TB6612FNG 电机驱动模块 | 1 | 双路 A/B，峰值约 3.2A/路 |

| 四驱底盘 + 直流电机 | 1 套 | 左 2 轮并联 → A 路，右 2 轮并联 → B 路 |

| HC-SR04（或 HC-SR04+） | 1 | 仅避障版；Trig/Echo 脉冲模式 |

| 面包板电源模块（MB102 类） | 1 | DC IN 7~10V，输出 5V / 3.3V |

| 电池 | 1 组 | 推荐 **2 节 18650 串联（约 7.4V）** |

| 杜邦线、面包板 | 若干 | |



### HC-SR04 安装建议



- **位置**：车头**正中间**，两圆孔**水平朝前**（勿朝下照地、勿朝上）。

- **高度**：约在前轮上沿附近（常见 5~15 cm 离地），能照到挡在前方的障碍，又不易误报地面。

- **接线**：模块 **Trig → D9**，**Echo → D12**（接反会一直 `cm=-1`）。



---



## 供电说明



### 推荐接法（一套电池）



```

2×18650 串联（约 7.4V，满电 ~8.4V）



  电池 + ──┬──→ 电源模块 DC IN（圆孔，要求 7~10V）

           └──→ TB6612 VM（电机电源）



  电池 - ──┬──→ 电源模块 GND

           ├──→ TB6612 GND（任一脚，模块上 3 个 GND 相通）

           └──→ Arduino GND



  电源模块 5V ──→ Arduino 5V、TB6612 VCC、HC-SR04 Vcc

  电源模块开关 ──→ 断开模块输入（逻辑 5V）；VM 若仍接电池则电机端可能仍有电

```



### 要点



| 项目 | 要求 |

|------|------|

| 电源模块 **DC IN** | **7~10V**（4 节 1.2V 仅 4.8V **不够**，5V 会掉到约 3V） |

| **TB6612 VM** | 4.5~10V，接电池正，**不要**接 5V 轨 |

| **TB6612 VCC** | 接 **5V** |

| **共地** | 电池负、模块 GND、Uno、TB6612、HC-SR04 **必须相连** |

| 上传程序 | 可仅用 USB 给 Uno；**试车**建议电池 + 模块 |



---



## 接线表



### TB6612 ↔ Arduino



| TB6612 | Arduino | 作用 |

|--------|---------|------|

| PWMA | **D5** | 左轮 PWM |

| AIN1 | **D4** | 左轮方向 |

| AIN2 | **D3** | 左轮方向 |

| PWMB | **D6** | 右轮 PWM |

| BIN1 | **D7** | 右轮方向 |

| BIN2 | **D8** | 右轮方向 |

| STBY | **D10** | 高电平使能 |

| VCC | **5V** | 逻辑电 |

| GND | **GND** | 共地 |

| VM | **电池 +** | 电机 7~10V |

| AO1、AO2 | 左侧电机 | 并联 |

| BO1、BO2 | 右侧电机 | 并联 |



### HC-SR04 ↔ Arduino（仅避障版）



| HC-SR04 | Arduino | 说明 |

|---------|---------|------|

| Vcc | **5V** | |

| Gnd | **GND** | 共地 |

| **Trig** | **D9** | 勿与 Echo 接反 |

| **Echo** | **D12** | |



### 引脚占用一览（避障版）



| D3~D8 | 电机 |

| D9 | HC-SR04 Trig |

| D10 | TB6612 STBY |

| D12 | HC-SR04 Echo |



---



## 代码功能完成情况



### 演示版（`uno_tb6612_hc04_demo_motor`）✅



| 功能 | 说明 |

|------|------|

| TB6612 双路驱动 | A=左，B=右 |

| 自动动作循环 | 前进 2s → 后退 → 左转 → 右转 |

| 方向修正 / 调试 | `MOTOR_*_DIR_REVERSE`、`DEBUG_HOLD_FORWARD` |



### 避障版（`uno_tb6612_hc04_avoid_v1_alg_1_0` / `uno_tb6612_hc04_avoid_v2_fsm_alg_1_1`）✅ 实车可跑



| 功能 | 说明 |

|------|------|

| HC-SR04 测距 | Trig/Echo，三次取最小滤波 |

| 滞回 + 确认 | 近障 `OBSTACLE_CM_*` 触发，畅通 `PATH_CLEAR_CM`；连续 2 次近才避障、3 次远才恢复 |

| 分段调速 | 远 `SPEED_CRUISE` / 中 `SPEED_LOW` / 近 `SPEED_SLOW` |

| 无效读数 | 不当作路通；连续无效短暂停再测 |

| 自适应脱困 | 反向加大转角；距离变近才升档；转向后短距离探测 |

| 串口调试 | `SERIAL_DEBUG=1`（9600） |

| 仅测距模式 | `DEBUG_SENSOR_ONLY=1` |

| 电机方向 | 本车 `MOTOR_LEFT/RIGHT_DIR_REVERSE=false`（与演示版一致；某侧反了单独改） |



#### 避障默认行为



上电约 1 秒后：前方无障碍则**低速直行**；距离小于阈值则**停 → 后退 → 转向**；狭窄处连续遇障会自动**升档**加大后退与转角。



#### 自适应参数（v1：`uno_tb6612_hc04_avoid_v1_alg_1_0.ino` 顶部可调）



| 常量 | 含义 |

|------|------|

| `OBSTACLE_CM_MIN` ~ `MAX` | 触发距离（随 stuck 档略增） |

| `PATH_CLEAR_CM` | 判定路通（须大于障碍阈值，建议大 10cm+） |

| `BLOCK_CONFIRM` / `CLEAR_CONFIRM` | 近障/远障连续次数，抑抖 |

| `MS_BACKUP_*` / `MS_TURN_*` | 后退与转向时间 |

| `STUCK_REPEAT_MS` / `STUCK_RESET_MS` | 升档与复位时机 |



单探头无法判断左右哪边更空，**死胡同仍可能出不来**；比固定左右同角摆荡更合理。



### 后续可做



| 功能 | 状态 |

|------|------|

| 按键 / 遥控 | ❌ |

| 串口控制 | ❌ |

| 循迹 | ❌ |

| 第二颗超声波 / 红外 | ❌ 改善窄道决策 |

| 上电待机（不自跑） | ❌ |



---



## 使用步骤



### 演示版



1. IDE 打开 `hardware/uno_tb6612_hc04/uno_tb6612_hc04_demo_motor`，接好 TB6612，**车轮架空**上传。

2. 5V 约 **4.9~5.1V**；某侧倒转则改 `MOTOR_*_DIR_REVERSE`。



### 避障版



1. IDE 打开 **`uno_tb6612_hc04_avoid_v2_fsm_alg_1_1`**（或 v1 文件夹），接好 TB6612 + HC-SR04（Trig=D9，Echo=D12）。

2. 先 `DEBUG_SENSOR_ONLY=1`，串口 **9600** 看 `cm=` 是否正常（手掌 20~40cm 有变化）。

3. 改 `DEBUG_SENSOR_ONLY=0` 上传；架空再地面试避障。

4. 该前进却倒车：改 `MOTOR_*_DIR_REVERSE`（当前避障版为 `false`）。

5. 调参：窄道仍摆 → 加大 `MS_TURN_STEP` 或 `STUCK_REPEAT_MS`；误触发 → 减小 `OBSTACLE_CM_MIN`。



刷回演示版：IDE 打开 **`uno_tb6612_hc04_demo_motor`** 上传。



---



## 常见问题



| 现象 | 可能原因 |

|------|----------|

| 电机不转，5V 约 3V | 电源模块输入 **低于 7V** |

| 一直 `cm=-1` 或 `us=0` | **Trig/Echo 接反**、未共地、线松动 |

| 编译 `DistanceRaw` 等错误 | 勿在同一 sketch 文件夹放多个 `.ino` |

| 避障时倒车前进反了 | 改 `MOTOR_*_DIR_REVERSE` |

| 窄道左右摆不出 | 单探头局限；升档参数或加第二传感器 |

| 编译/上传错乱 | 确认 IDE 标题为当前 sketch 文件夹名（如 `uno_tb6612_hc04_avoid_v2_fsm_alg_1_1`） |



---



## 变更记录（摘要）



- 初版：TB6612 双路四驱、自动动作循环。

- 调试：整理工程路径；`DEBUG_HOLD_FORWARD`；移除旧 sketch。

- 供电：电源模块需 **7~10V** 输入，2×18650 串联。

- 避障：新增避障 sketch（HC-SR04、自适应脱困）；与演示版分文件夹维护。
- 目录重组：`hardware/uno_tb6612_hc04/<平台>_<算法>/`；见 `docs/NAMING.md`。


