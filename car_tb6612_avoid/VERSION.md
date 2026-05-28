# car_tb6612_avoid 版本说明

> **说明**：避障策略、阈值推导、滞回等「为什么这样设计」的解释性内容，以本文档为准；代码里只保留简短注释并指向此处。

## Arduino Uno 执行模型（单线程、顺序执行）

本工程跑在 **Arduino Uno** 上：**没有操作系统、没有多线程**，`setup()` 跑一遍后只有 `loop()` 在死循环里**一行一行顺序执行**。

| 现象 | 原因 |
|------|------|
| `delay(500)` 转向时**不能**同时测距 | CPU 卡在 `delay` 里，别的代码不会跑 |
| `delay(30)` 末尾停顿但电机还在转 | `driveForward()` 已把 PWM 设好，电机由硬件维持；只是程序在等 |
| 脱困时车「发呆」几秒 | `BACKUP` / `doTurnInPlace` 里长 `delay` 占满 CPU，属设计如此 |
| 不能「一边全速一边后台扫描」 | 要并行只能拆状态机 + 缩短单次阻塞，或换更强 MCU / 写中断驱动 |

所以 v2 用 **状态机**（`FORWARD` / `DECIDE` / …）而不是多任务：每一时刻只做一件事，用 `state` 记住「下一步该干啥」。  
`MS_LOOP_PAUSE`、测距三次取最小、滞回阈值，都是在这个前提下让**顺序执行**更稳、别过激。

| 版本 | 文件 | 状态 |
|------|------|------|
| **v1** | `car_tb6612_avoid_v1.ino` | 稳定基线（固定雷达，时间转向 + 自适应脱困） |
| **v2** | `car_tb6612_avoid_v2.ino` | **当前开发**：机械参数化 + 距离分段 + 状态机 + 右手法则 |

切换：Arduino 同一 sketch 文件夹里所有 `.ino` 会一起编译，**不能同时启用两个版本**。  
编辑/上传哪个版本时，把另一个临时改名为 `*.ino.bak`（或剪切到其它目录）即可。  
后续路线图见 [docs/DEVELOPMENT.md](../docs/DEVELOPMENT.md)。

上传后串口会打印当前 `FIRMWARE_VERSION`，便于核对。

---

## v1 概述

- **传感器**：单路 HC-SR04，**固定安装在车头正前方**（无舵机扫描）。
- **执行器**：TB6612 双路驱动四驱；转向、后退用 **定时 `delay`**，无编码器闭环。
- **目标**：开阔地持续低速前进；前方近障时 **停 → 后退 → 转向** 脱困。

---

## v1 主循环流程

```
        ┌─────────────┐
        │  上电初始化  │
        └──────┬──────┘
               ▼
        ┌─────────────┐
   ┌───│ 三次测距取最小 │◄────────────────┐
   │   └──────┬──────┘                     │
   │          ▼                             │
   │   距离 < 障碍阈值？                    │
   │     │是          │否                  │
   │     ▼            ▼                    │
   │  自适应脱困    低速直行                 │
   │  退+反向转向   （SPEED_LOW）           │
   │     │            │                    │
   │     └────────────┴────────────────────┘
   │          （loop 重复）
```

---

## v1 测距

| 项目 | 实现 |
|------|------|
| 引脚 | Trig **D9**，Echo **D12** |
| 单次 | Trig 10µs 触发 → `pulseIn` 测 Echo 高电平 → `cm = us / 58` |
| 有效范围 | 约 2～400 cm；超时或超范围返回 **-1** |
| 滤波 | 连续读 **3 次**，取 **最小值**（偏保守，利于提前发现近障） |
| 无效读数 | v1 中 `cm=-1` 时 **不触发**避障（`isPathBlocked` 要求 `cm>0`），车会继续低速前进 |

---

## v1 避障触发

```cpp
isPathBlocked(cm)  ⇔  cm > 0 且  cm < obstacleThresholdCm()
```

`obstacleThresholdCm()` = `OBSTACLE_CM_MIN` + `stuckLevel` × `OBSTACLE_CM_STEP`（上限 `OBSTACLE_CM_MAX`）。

| 常量（默认） | 含义 |
|--------------|------|
| `OBSTACLE_CM_MIN` = 18 | 0 档触发距离（cm） |
| `OBSTACLE_CM_STEP` = 4 | 每升一档阈值 +4 cm |
| `OBSTACLE_CM_MAX` = 32 | 阈值上限 |

**无滞回**：略大于阈值即恢复直行（v2 可考虑 `PATH_CLEAR_CM` 大于触发阈值以减少抖动）。

---

## v1 自适应脱困（`stuckLevel` 0～4）

每次进入 `runAdaptiveAvoidance()`：

1. **停车**（约 150 ms）
2. **后退** `backupMsForLevel()`（随档位增加，约 280～900 ms）
3. **原地转向**：与 **上一次转向相反**（`lastTurnDir` 取反；首次默认 **右转**）
4. 转向时长 `turnMsForLevel()`（随档位增加，约 320～1400 ms）

### 升档

若距 **上次脱困** 不足 `STUCK_REPEAT_MS`（默认 1800 ms）且 **再次**触发避障 → `stuckLevel++`（最高 `STUCK_MAX_LEVEL` = 4）。

### 降档（复位）

前方 **不触发**避障且持续超过 `STUCK_RESET_MS`（默认 2500 ms）→ `stuckLevel = 0`。

### 设计意图

| 行为 | 目的 |
|------|------|
| 左右 **交替** 且转角随档位 **加大** | 避免固定右转原地转圈；窄道时加大转角尝试脱出 |
| 阈值随档位略增 | 连续卡死时略提前刹车 |

### 已知局限（v1）

- **单探头**：不知左/右哪侧更空，窄通道、U 形角仍可能左右摆。
- **时间转向**：电量、地面摩擦力变化会导致实际转角不一致。
- **无转向后探测**：转完即进入下一轮 loop，不在短距离内复测是否仍堵。
- **无效读数**：`-1` 时不避障，开阔处偶发无效时可能盲冲。

---

## v1 直行

- 速度：`SPEED_LOW`（默认 120），未做距离分段调速。
- 每轮 loop 末尾 `MS_LOOP_PAUSE`（50 ms）。

---

## v1 调试宏

| 宏 | 作用 |
|----|------|
| `DEBUG_HOLD_FORWARD=1` | 只测电机，不测距 |
| `DEBUG_SENSOR_ONLY=1` | 状态机 + `SERIAL_DEBUG` 照常，**电机不转**（不再只打 `cm=`） |
| `SERIAL_DEBUG=1` | 串口 9600 打印 `cm`、`stuckLevel` 等 |
| `SERIAL_DEBUG_RAW=1` | 额外打印 `us` 与失败原因 |

---

## v1 电机方向

本车底盘标定（雷达改向后与演示版一致）：`MOTOR_LEFT_DIR_REVERSE = false`，`MOTOR_RIGHT_DIR_REVERSE = false`。  
若「该前进却倒车」，与演示版相同方式改这两项。

---

## v2 概述

相对 v1，主要做四件事（见 [DEVELOPMENT.md](../docs/DEVELOPMENT.md) v2 章节有详细图）：

1. **小车机械参数显式化**：`CAR_LENGTH_CM / CAR_WIDTH_CM / CAR_HEIGHT_CM`，`DEG_PER_MS_AT_SPEED_LOW`，`MAX_TURN_DEG`。
2. **距离分段决策**：阈值 `dStop / dBrake / dSafe / dClear / dProbeOk` 由车身尺寸 + 制动余量推导；只有 **极近** 才后退，**近** 时只转向探测。
3. **状态机**：`IDLE → FORWARD → DECIDE → (TURN_PROBE) → BACKUP/ESCAPE`，每次状态切换串口打印。
4. **右手法则探测**：DECIDE 内部分相位 **右探 → 回正 → 左探 → 后退升档**，符合贴墙避障习惯。

## v2 距离阈值原理详解

> 思路：**先想清楚车要做什么动作，再算出该动作需要多少距离。**  
> 不是凭感觉填 18/25/32，而是从「车身尺寸 + 安全余量」推出来。

### 三个 PAD（安全气垫，由近到远）

| PAD | 默认 (cm) | 含义 |
|-----|-----------|------|
| `STOP_PAD_CM` | 6 | 必须停车时希望留的最小空气；最前部位离障碍 6cm 就算紧张 |
| `BACKUP_PAD_CM` | 4 | 已经太近，要给「后退动作启动」的反应时间留位置 |
| `BRAKE_PAD_CM` | 12 | 从中速刹到低速所需的距离 |

它们是不同档位的"安全距离储备"，下面的 `d*` 都是用它们 + 车身尺寸算出来的。

### 每个 d* 在车要做哪个动作时被触发

#### `dStop` = `STOP_PAD + BACKUP_PAD`（10）

**含义**：「这么近了，只转向也会蹭墙，必须先后退。」

- `STOP_PAD`：基础"不能更近了"红线
- `BACKUP_PAD`：后退动作本身有反应时间，启动那一瞬车头还会前移一点

→ 进入 **BACKUP** 状态

#### `dBrake` = `CAR_WIDTH_CM + STOP_PAD`（22）

**含义**：「这么近，原地转向会让车身扫过一段空间，要先停下选方向。」

**为什么用车宽？** 原地转 90° 时车身上任意一点扫过的最大半径约：

\[
r = \tfrac{1}{2}\sqrt{L^2 + W^2}
\]

本车 26×16：`r ≈ 15.3cm`。  
`CAR_WIDTH_CM`（16）略大于 `r`，是个**保守上界**——只要前方 ≥ 车宽，原地转就不会蹭到。  
再加 `STOP_PAD` 是"边转边留 6cm 空气"。

→ 进入 **DECIDE**（停下做右手法则）

#### `dSafe` = `CAR_LENGTH_CM + BRAKE_PAD`（38）

**含义**：「前方还能容下一整个车身 + 一段刹车距离，可以走，但要减速。」

- `CAR_LENGTH_CM`：留一个完整车身长的空气（直觉：撞墙前还有"自己这么长"的余地）
- `BRAKE_PAD`：从高速 → 低速的减速过程

→ 用 `SPEED_LOW`；`dSafe ~ dClear` 之间用 `SPEED_MID`

#### `dClear` = `dSafe + 10`（49）

**含义**：「前方明显比 `dSafe` 还远，确实开阔，可以全速。」

→ 用 `SPEED_HIGH`；满足 `dClear` 持续 `STUCK_RESET_MS` 后还会复位 `stuckLevel`

#### `dProbeOk` = `dSafe + 5`（44）

**含义**：DECIDE 阶段右/左探后，**这个方向测距 ≥ dProbeOk 就认为路通**，转过去走。

→ 比 `dSafe` 略严一点，保证转过去之后至少能走一段，不会立刻又触发 DECIDE

### `dClear` / `dProbeOk` 为什么要在 `dSafe` 上再加余量？

这两个 `+` **不是给车留物理空间**，而是给软件留**滞回带（hysteresis）**，避免在阈值边界反复横跳。

#### 没有滞回时会怎样（以直行为例）

若升档、降档都用同一个 `dSafe`（39cm）：

```
cm = 38（≈dSafe）→ 减速 SPEED_LOW
惯性还在 → cm 涨到 39 → 升速 SPEED_MID
速度变快 → cm 又跌到 37 → 又减速...
```

测距本身还有 ±2cm 抖动 → 车在 `dSafe` 附近**反复加减速**，一顿一顿。

#### `dClear = dSafe + 10`：直行升档的「死区」

```
距离 (cm)
              SPEED_HIGH
   ↑ 49 ━━━━ dClear ━━━━━━━━━━━  「升档线」：要全速，必须明显比 dSafe 远
   │
   │            ←── 死区 10cm，速度不变 ──→
   │
   ↓ 39 ━━━━ dSafe ━━━━━━━━━━━━  「降档线」：该减速了
              SPEED_MID / LOW
```

- 要升到 `SPEED_HIGH`：距离 ≥ **49**（`dClear`）
- 要从 HIGH 掉下来：距离要明显 **< dSafe**（配合 `FORWARD` 里的判断）

**升档难、降档易** → 中间 10cm 谁也不动，车跑得稳。工业控制里水位、温控常用同一思路。

#### FORWARD ↔ BACKUP 在同一位置来回抖

典型链路：`FORWARD` 见 `cm<dStop` → `BACKUP` 约 350ms → `DECIDE` 单次测到 `cm≥dProbeOk` → 又 `FORWARD` 顶墙 → 再 `BACKUP`。

对策（v2 已实现）：

| 机制 | 作用 |
|------|------|
| `NEAR_CONFIRM` | 极近须连续 N 次才 BACKUP，滤单次误读 |
| `BACKUP_COOLDOWN_MS` | 刚退完不再立刻 BACKUP，改 `DECIDE` 转向 |
| `CLEAR_CONFIRM` | 回 FORWARD 须连续 N 次 `cm≥dProbeOk` |
| 冷却内禁止 DECIDE 阶段 0「未转就直行」 | 后退后必须先右/左探，不能测一次就 FORWARD |

| 想调 | 改哪 |
|------|------|
| 切档太跳、一顿一顿 | **加大** `dClear - dSafe`（公式里的 `+10`） |
| 反应慢、很久才全速 | **减小** `+10` |

#### `dProbeOk = dSafe + 5`：转向探测判通的「最低标准」

DECIDE 右探/左探后若用 `dSafe`（39）就判通：

- 转过去 → 立刻 `cm ≈ 39` → 马上又 `< dBrake` 或掉进 `SPEED_LOW`
- 等于「转过去又要想转」→ **频繁再进 DECIDE**

`+5` 后须 `cm ≥ 44` 才算这条路能走，转过去至少能**先走一段**再重新评估。

为什么是 5 而不是 10？探测时车已停着，比直行调速更不敏感；若跟 `dClear` 一样严，容易三面都「差一点」就进 ESCAPE。

| 想调 | 改哪 |
|------|------|
| 转过去又马上想再转 | **加大** `dProbeOk - dSafe`（公式里的 `+5`） |
| 探测太严、老进 ESCAPE | **减小** `+5` |

#### 三个「+余量」角色对比

| 阈值 | 相对 dSafe | 性质 | 作用 |
|------|------------|------|------|
| `dSafe` | +0 | **物理** | 该减速了 |
| `dClear` | +10 | **软件滞回** | 直行升全速，防加减速抖动 |
| `dProbeOk` | +5 | **软件滞回** | 转向探测判通，防转完立刻再 DECIDE |

代码里对应（`car_tb6612_avoid_v2.ino`）：

```cpp
inline int dClear()   { return dSafe() + 10; }  // 直行滞回，改 +10 调灵敏度
inline int dProbeOk() { return dSafe() + 5; }   // 探测判通，改 +5 调灵敏度
```

### 探头偏移补偿

阈值按「**车上最靠前部位**」推导。再统一加补偿：

```
sensorBackMargin = max(0, -SENSOR_OFFSET_CM)
```

| `SENSOR_OFFSET_CM` | 最靠前是谁 | margin |
|--------------------|------------|--------|
| 正（探头突出） | **探头**（按测距直接判断） | 0 |
| 0 | 探头/车头齐平 | 0 |
| 负（探头退后） | **车头**（测距比实际车头距大） | `|offset|` |

只有探头**退后**时才补；本车 `offset = -1` → `margin = 1`。

### 推导总览

```
车上最靠前部位的余量（仅供推导）：
  frontStop  = STOP_PAD + BACKUP_PAD            // 6+4   = 10
  frontBrake = CAR_WIDTH_CM + STOP_PAD          // 16+6  = 22
  frontSafe  = CAR_LENGTH_CM + BRAKE_PAD        // 26+12 = 38

与测距 cm 比较的阈值：
  dStop    = frontStop  + sensorBackMargin      // 10+1 = 11
  dBrake   = frontBrake + sensorBackMargin      // 22+1 = 23
  dSafe    = frontSafe  + sensorBackMargin      // 38+1 = 39
  dClear   = dSafe + 10                         //        49
  dProbeOk = dSafe + 5                          //        44
```

> 串口启动会打印 `d(cm) Stop/Brake/Safe/Clear/ProbeOk=...`，按本车参数核对即可。

### 距离区间与动作

```
0 ─── dStop(11) ─── dBrake(23) ─── dSafe(39) ─── dClear(49) ─── ∞
  极近            近             中             较通          通畅
  BACKUP         DECIDE         SPEED_LOW      SPEED_MID     SPEED_HIGH
```

| 区间 | 条件 | 动作 |
|------|------|------|
| 极近 | `cm < dStop` 连续 `NEAR_CONFIRM` 次 | `BACKUP` 后退，再 `DECIDE`（冷却内不再立刻 BACKUP） |
| 近 | `dStop ≤ cm < dBrake` | 进入 `DECIDE` 探测左/右/前 |
| 中 | `dBrake ≤ cm < dSafe` | `SPEED_LOW` 慢行 |
| 较通 | `dSafe ≤ cm < dClear` | `SPEED_MID` 中速 |
| 通畅 | `cm ≥ dClear` | `SPEED_HIGH` 全速，复位 `stuckLevel` |

### 想改哪一个

| 现象 | 改哪 |
|------|------|
| 撞墙太晚才刹 | 加大 `BRAKE_PAD_CM` 或 `STOP_PAD_CM` |
| 同一位置反复前进后退（FORWARD↔BACKUP） | 加大 `BACKUP_COOLDOWN_MS`；或加大 `CLEAR_CONFIRM` / `NEAR_CONFIRM` |
| 一直在墙边走停走停、加减速抖 | 加大 `dClear - dSafe`（公式里 `+10`，见上文滞回说明） |
| 转过去又马上想再转 | 加大 `dProbeOk - dSafe`（公式里 `+5`） |
| 已经很近才后退 | 加大 `BACKUP_PAD_CM` |
| 原地转弯蹭到桌腿 | 加大 `CAR_WIDTH_CM`（或直接给 `dBrake` 公式加常量） |
| 探头从车里改成挂在前面 | **只改** `SENSOR_OFFSET_CM`，阈值自动跟着变 |

公式只是"合理近似"，不是物理上 100% 精确的最小安全距离；实车跑两圈看到哪个值不合适，对应改一个 PAD 就行。

## v2 运行时状态变量

`car_tb6612_avoid_v2.ino` 里除状态机 `state` 外，还有一组 **跨 loop 记住的变量**（`static`，断电清零）：

| 变量 | 类型 | 初始 | 干什么 |
|------|------|------|--------|
| `stuckLevel` | `byte` | 0 | **脱困档位** 0～3；越高后退越久、ESCAPE 转角越大 |
| `lastTurnDirSign` | `int` | 1 | ESCAPE 时转向方向：**1=右转，-1=左转**；每次 ESCAPE 取反 |
| `lastEscapeMs` | `unsigned long` | 0 | 上一次进入 **ESCAPE** 的时刻（`millis()`） |
| `pathClearSinceMs` | `unsigned long` | 0 | 前方首次判定「通畅」的时刻；用于复位 `stuckLevel` |
| `blockedStreak` | `byte` | 0 | **连续近障**计数；够 `BLOCK_CONFIRM` 次才进 DECIDE |
| `clearStreak` | `byte` | 0 | **连续畅通**计数；`DECIDE` 里够 `CLEAR_CONFIRM` 次且 `cm≥dProbeOk` 才回 FORWARD |
| `nearStreak` | `byte` | 0 | **连续极近**计数；够 `NEAR_CONFIRM` 次才进 BACKUP |
| `lastBackupEndMs` | `unsigned long` | 0 | 上次 BACKUP/ESCAPE 结束时刻；冷却内 FORWARD 不再立刻 BACKUP |
| `lastValidCm` | `long` | 100 | 最近一次**有效**测距；`cm=-1` 时暂用此值决策 |

### `stuckLevel` — 脱困档位

**什么时候 +1**：进入 `ESCAPE`（三面皆堵）时，若距上次 ESCAPE 不足 `STUCK_REPEAT_MS`（2s），则 `stuckLevel++`（最高 `STUCK_MAX_LEVEL`=3）。

**影响什么**：

| 档位 | 后退（BACKUP） | ESCAPE 转角 |
|------|----------------|-------------|
| 0 | `MS_BACKUP_BASE`（350ms） | `ESCAPE_TURN_DEG`（90°） |
| 1 | +150ms | +20° |
| 2 | +300ms | +40° |
| 3 | 封顶 900ms | 封顶 180° |

**什么时候归零**：`FORWARD` 中 `cm ≥ dClear`（49cm）且持续超过 `STUCK_RESET_MS`（3s）→ `stuckLevel = 0`。

```
窄道卡死 → ESCAPE lvl0 → 仍堵 → 2s 内再 ESCAPE lvl1 → … → 开阔跑 3s → lvl 回 0
```

### `lastTurnDirSign` — ESCAPE 转向记忆

- **DECIDE** 里右探/左探**不用**这个变量（按右手法则固定顺序：先右后左）。
- 只在 **ESCAPE**（彻底堵死）时用：`lastTurnDirSign = -lastTurnDirSign`，再 `doTurnInPlace(lastTurnDirSign, 大角度)`。
- 初值 `1`：第一次 ESCAPE 会先 **左转**（因为先取反成 -1）… 等等，初值 1，取反后 -1 是左转。第二次 ESCAPE 取反成 +1 右转。

实际上：首次 ESCAPE：`1 → -1` → **左转**；再次 ESCAPE：`-1 → +1` → **右转**。与 v1「首次右转」不同，若要对齐可改初值为 `-1`。

### `lastEscapeMs` — 上次脱困时间戳

- 每次 `runEscape()` 末尾写入 `lastEscapeMs = millis()`。
- 与下一次 ESCAPE 比较：间隔 `< STUCK_REPEAT_MS` → 认为「上次脱困没用」→ `stuckLevel++`。
- 间隔够长再 ESCAPE → 不升档（可能已换环境）。

### `pathClearSinceMs` — 路通计时（复位 stuckLevel）

在 `FORWARD` 里，当 `cm ≥ dClear()`：

- 第一次满足：`pathClearSinceMs = millis()`（开始计时）
- 持续满足超过 `STUCK_RESET_MS`：`stuckLevel = 0`
- 中途又变近（`< dSafe`）：`pathClearSinceMs = 0`（计时清零，要重新「通畅 3 秒」）

比「连续 N 次测距远」更稳：不要求每一圈 loop 都远，只要**一段时间内整体通畅**即可。

### `blockedStreak` — 近障确认（防误触发 DECIDE）

在 `FORWARD` 里，当 `dBrake ≤ cm < dStop` 不成立且 `cm < dBrake()`：

- 每圈 `blockedStreak++`
- 达到 `BLOCK_CONFIRM`（默认 2）→ 才 `enterState(ST_DECIDE)`
- 否则仍 `SPEED_LOW` 慢慢蹭过去（给一次误读机会）

一旦 `cm ≥ dBrake()` 或进入其它分支：`blockedStreak = 0`。

```
单次误读近障 → streak=1 → 仍慢行，不进 DECIDE
连续 2 次近障 → streak=2 → 进 DECIDE
```

### `clearStreak` — 预留

代码里声明并在 `enterState` 时清零，**当前未累加、未判断**。  
`stuckLevel` 复位由 `pathClearSinceMs` + `STUCK_RESET_MS` 负责。后续若改回「连续 N 次远障才复位」可启用。

### `lastValidCm` — 无效测距时的替补距离

```cpp
long cm = readDistanceCmFiltered();
if (cm > 0) lastValidCm = cm;
long d = (cm > 0) ? cm : lastValidCm;   // -1 时用上次有效值
```

- 每次读到有效 `cm > 0`：更新 `lastValidCm`。
- 本圈 `cm == -1`：决策仍用**上一圈**的 `lastValidCm`，不会把距离当成 0。
- 初值 `100`：上电第一次无效时不会立刻触发极近 BACKUP。

#### `cm = -1` 在代码里表示什么

三次测距**全部无效**时返回 -1，常见原因：

| 原因 | 典型场景 |
|------|----------|
| `pulseIn` 超时（`us=0`） | 回声太弱、被吸收、线松、探头被挡死 |
| 算出 `< 2cm` | 进入 HC-SR04 **盲区** |
| 算出 `> 400cm` | 超出量程 |
| **车体晃动 / 急转** | 波束扫偏、回波来不及稳定、见下文 |

#### 晃动、急转为什么也会 `cm=-1`（前方未必有障碍）

实车调试里常见：**手里晃小车、或原地转向很快**时，串口偶发 `cm=-1`，眼前并没有东西挡住 HC-SR04。这是**物理层丢回波**，不是程序把距离算成 0。

HC-SR04 每帧流程是：Trig 发一串 40kHz → 等 Echo 高电平持续时间 → 换算距离。要成功，需要：

1. 探头与反射面**相对静止或接近静止**（至少在一帧测量时间内）；
2. 波束锥角（约 ±15°）内有一块**能反射**的平面；
3. Echo 线路上收到**完整、干净**的脉冲。

下面任一成立，就可能 `us=0`（超时）→ `cm=-1`：

| 现象 | 原因 |
|------|------|
| **手晃 / 颠簸** | 探头角度瞬间偏开，回波来自侧面地面/桌腿或干脆收不到；模块在细线、面包板上晃，Echo 边沿抖动 |
| **转向特别快**（尤其 `TURN_PROBE` / `ESCAPE` 原地转） | 波束在转动的几十～几百毫秒内扫过墙面时间很短；等效「目标在动」，回波弱或相位对不上；有时整帧都等不到有效 Echo |
| **加速 / 急停** | 车身俯仰、探头上下点，波束掠过地面或 sky，偶发丢帧 |
| **仅测距调试（旧行为）** | 若 `loop` 里只调 `printDistanceDebug()` 则只有 `cm=`；当前 v2 的 `DEBUG_SENSOR_ONLY=1` 改为**空跑状态机**（电机停、串口全量） |

因此：**转向越快、探头越松，`-1` 越容易出现**；静止对着白墙 20～50cm 若稳定，说明接线与供电大体正常，不必因「晃一下就 -1」去改 Trig/Echo。

与避障的关系：

- 单帧 `-1`：v2 用 `lastValidCm` 顶替，避免开阔地偶发超时就急刹（见上）。
- **连续多帧 `-1`**（急转、软物吸波、线松同时发生）：可能仍「以为前方很远」——见下文「设计取舍」与「以后可改进」。

实车缓解（不改代码也能做）：

- 雷达**拧紧、朝前**，减少随车身乱晃；
- 转向探测角 / 时间不要过大过快（`PROBE_TURN_DEG_*`、`DEG_PER_MS` 标定）；
- 调试时以**静止**读数为准；看动态行为用正常避障 + `SERIAL_DEBUG`，不要单靠手晃时的 `cm=`。

#### 手晃时的「连续 -1」≠ 正常跑动时的常态

实车对比（本车经验）：

| 场景 | 典型现象 | 要不要紧张 |
|------|----------|------------|
| **手里快速晃** | 很容易**连续多帧** `cm=-1` | 一般**不必**；角速度、加速度往往远大于正常行驶 |
| **正常直行 `FORWARD`** | 偶发单帧 `-1` 可能有，连续一长串少见 | 以静止标定 + 开阔地串口为准 |
| **程序里的原地转**（`TURN_PROBE` / `ESCAPE`） | 比直行更容易丢回波，但通常仍**慢于**手晃 | 若只有转向瞬间偶发 `-1`，多属预期 |

原因简述：手晃时探头在几十毫秒内可被甩出波束锥角，且 Echo 线随动产生杂波；**实际跑动**时车身有惯量、轮子贴地，角速度受电机与 `turnMsForDegrees` 限制，**达不到**手晃那种「来回扫射」的强度。因此：

- **不要**因为调试时「猛晃就一串 -1」就认定实车一定会盲冲，或急着上「连续 -1 就停车」的激进策略；
- **要**在 `DEBUG_SENSOR_ONLY` 改回 `0` 后，用真实避障 + `SERIAL_DEBUG` 看 `FORWARD` 直行、以及实际转向时的 `cm` / `d=` 是否可接受；
- 若**只有**快速转向瞬间偶发 `-1`，而直行稳定，优先拧紧探头、略减转角/转速，而不是按手晃标准改逻辑。

「连续 -1 不信任 `lastValidCm`」仍可作为 **v3** 选项，触发条件建议按**实车跑**统计（例如直行连续 N 次），而不是按手晃标定。

#### 软物（如卫生纸）会不会变成 -1？

**有可能，三种情况都有：**

| 情况 | 测距结果 | 程序行为 |
|------|----------|----------|
| 纸很近、吸收强、无稳定回波 | **-1** | 用 `lastValidCm`（可能是开阔时的 40～50cm）→ **仍按路通全速** ⚠️ |
| 纸有一定反射、距离有效 | **较小正数**（如 5～15cm） | 按真实近距减速 / DECIDE / BACKUP ✅ |
| 纸贴在盲区里（<2cm） | **-1** | 同上，可能误用旧的大距离 ⚠️ |

所以：**卫生纸既可能测出很近，也可能测成 -1**；测成 -1 时，当前 v2 **不会**因为「看不见」而自动减速，而是**相信上次还很远**——这是用 `lastValidCm` 换「别因单次误报 -1 就急刹」的代价。

#### 设计取舍（为什么仍用上次距离）

- **好处**：开阔地偶发一次超时（线抖、电磁干扰）不会突然 BACKUP。
- **风险**：前方突然变成**吸波软物**且连续 -1 时，会像「以为还很空」继续冲。

#### 以后可改进（v3 思路，当前未做）

连续多圈 `cm == -1` 时：**不信任** `lastValidCm`，改为 `SPEED_SLOW` 或短暂 `stopAll` 再测，例如：

```
连续 3 次 -1 → 当「前方不确定」处理，慢行或停车
```

#### 实车怎么判断是不是这个问题

串口看 `FWD cm=-1 d=48`：cm 无效但 d 仍很大 → 正在用旧距离。  
用手纸在探头前试：若 cm 在「小正数」和「-1」之间跳，就属此类。

### 与状态机 `state` 的关系

| 状态 | 主要读写的变量 |
|------|----------------|
| `FORWARD` | `blockedStreak`、`pathClearSinceMs`、`lastValidCm`、可能复位 `stuckLevel` |
| `DECIDE` | `lastValidCm`；用 `probePhase`（另见右手法则表） |
| `BACKUP` | 读 `stuckLevel` 定后退时长 |
| `ESCAPE` | 改 `stuckLevel`、`lastTurnDirSign`、`lastEscapeMs` |

串口 `SERIAL_DEBUG=1` 时，`FORWARD` 会打印 `cm`、`d`（决策用距离），可对照上述变量理解行为。

---

## `MS_LOOP_PAUSE` 为什么每圈 loop 要 `delay(30)`？

在 `loop()` 末尾（每个状态处理完之后）统一有：

```cpp
delay(MS_LOOP_PAUSE);  // 默认 30ms
```

### 它不是什么

- **不是**让车停 30ms：在 `FORWARD` 里，`driveForward()` 已经设好 PWM，这 30ms 内**电机仍在转**。
- **不是**超声波硬件强制要求的唯一间隔（测距函数内部还有自己的等待）。

### 它是什么：主循环的「节拍器」

把「测距 → 判断 → 改速度/换状态」这一圈，限制在**不要太密**的频率上。

#### 1. 给 HC-SR04 留恢复时间

一次 `readDistanceCmFiltered()` 要连测 **3 次**，中间已有 `delay(15)` × 2。  
若 loop 末尾不再停顿，下一圈立刻又测，相邻两组脉冲可能**太近**，偶发串扰、读数跳变。

30ms 是「这一圈决策结束，稍等再开始下一圈」的缓冲。

#### 2. 限制决策频率，避免反应过激

`FORWARD` 里若每圈只有几毫秒间隔：

- 距离在 `dBrake` / `dSafe` 边界附近抖动时，`blockedStreak`、`调速` 变化更快 → 更容易抖
- 串口调试时打印也会刷屏

有 30ms 后，直行时大约 **每秒几十次** 决策（还要加上 3 次测距耗时），对人眼和机械都更平滑。

#### 3. 和「阻塞状态」的关系

| 状态 | loop 里额外 30ms 的影响 |
|------|-------------------------|
| `FORWARD` | **明显**：决定两次测距之间隔多久 |
| `DECIDE` / `BACKUP` / `ESCAPE` | **几乎可忽略**：内部已有 `delay(120)`、转向几百 ms、后退几百 ms |

所以调 `MS_LOOP_PAUSE` **主要影响直行时的反应快慢**，不太影响脱困动作本身。

### 一圏 `FORWARD` 大概多久（帮助理解 30ms 占多少）

```
readDistanceCmFiltered()  ≈ 3×测距 + 2×15ms  ≈ 几十～上百 ms（视距离）
driveForward()            ≈ 立刻
delay(MS_LOOP_PAUSE)      ≈ 30ms
```

30ms 往往只占整圈时间的一小部分，但仍是**唯一专门留给「别测太勤」**的旋钮。

### 想改的时候

| 现象 | 建议 |
|------|------|
| 直行反应慢、刹得晚 | **减小**（如 10～20），或同时检查 `dBrake` 是否过大 |
| 读数跳、调速抖、串口刷太快 | **加大**（如 50～80） |
| 设为 `0` | 最快反应；可能增加超声误读，一般不推荐 |

---

## v2 转向标定

```cpp
DEG_PER_MS_AT_SPEED_LOW = 0.30   // ≈ 300°/s
turnMsForDegrees(deg) = deg / DEG_PER_MS_AT_SPEED_LOW
```

标定方法：架空或在地面用 `doTurnInPlace(+1, 90)`，看实际转角，按比例调整。

## v2 右手法则探测相位

| 相位 | 动作 | 通则进入 |
|------|------|----------|
| 0 | 测前方 | 通 → FORWARD |
| 1 | 右探 `PROBE_TURN_DEG_RIGHT`（默认 35°） | 通 → FORWARD（保持已转角度） |
| 2 | 回正再测前方 | 通 → FORWARD |
| 3 | 左探 `PROBE_TURN_DEG_LEFT`（默认 35°） | 通 → FORWARD |
| 4 | 三面皆堵 → 回正 → `ESCAPE` | — |

## v2 ESCAPE

- 升 `stuckLevel`（最高 3）
- 后退 `MS_BACKUP_BASE + (lvl+1)*MS_BACKUP_STEP`，封顶 900 ms
- **反向**大角度转向：`ESCAPE_TURN_DEG + lvl*20`，封顶 `MAX_TURN_DEG`
- 回 DECIDE 重测

## v2 已知保留

- 转向仍是**时间法**：电压、地面变化会偏；后续版本接编码器或 MPU 偏航角。
- DECIDE/ESCAPE 仍 **同步阻塞**（`delay` 等待转完）：转向时不能测距，不能响应中断。
- 暂未维护转向历史。

---

## 变更记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-05-27 | v1 | 固定 HC-SR04 + 自适应脱困（交替转向、stuckLevel 档位） |
| 2026-05-28 | v2 | 机械参数化 + 距离分段 + 状态机 + 右手法则探测 |
