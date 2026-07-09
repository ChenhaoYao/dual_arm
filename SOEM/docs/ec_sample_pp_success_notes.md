# ec_sample PP 模式成功运行经验记录

## 背景

本次调试目标是在 `samples/ec_sample/ec_sample.c` 中使用 SOEM 主站控制 YH052 关节模组，在 CiA402 Profile Position Mode，也就是 PP 模式下执行 30° 位置运动。

最初现象是：

- EtherCAT 能进入 OP。
- CiA402 状态机能进入 `Operation enabled`。
- 电机能听到使能时的“咔”声。
- 但是电机不转。
- 日志中出现明显不合理的 `Target Reached`。

典型异常日志：

```text
[MOTION] cycle=9581 触发 PP 定位 → 4368928 counts (30°)
[MOTION] cycle=9585 Target Reached, pos=-51 (期望 4368928, 误差 -4368979)
```

实际位置距离目标还差约 436 万 counts，但程序已经判断到位。

## 问题一：误用 0x6041 bit10 导致 Target Reached 误判

最初代码只用状态字 `0x6041` 的 bit10 判断目标到达：

```c
if (sw & 0x0400)
```

但在 PP 模式中，bit10 的含义不是“刚写入的新目标已经完成”，而是当前驱动器内部目标/轨迹处于 reached 状态。刚进入 `Operation enabled` 时，驱动器还没有接受新的 PP 目标，因此 bit10 可能本来就是 1。

典型状态：

```text
SW=0x0637
```

拆解：

```text
0x0637 = 0x0400 + 0x0200 + 0x0037
```

含义：

- `0x0037`：Operation enabled 相关基础状态位。
- `0x0200`：bit9 Remote。
- `0x0400`：bit10 Target reached。

因此，刚使能后看到 `0x0637` 并不代表本次目标已经执行完成，只能说明当前旧目标或当前位置被驱动器认为已经 reached。

### 修正经验

不能只依赖 bit10 判断本次运动完成，必须至少满足：

- 新 set-point 已被驱动器 acknowledge。
- bit10 曾经清零，说明新运动进入 active。
- bit10 再次置位。
- 实际位置误差进入允许范围。

当前代码增加了 `TARGET_TOLERANCE_COUNTS`，并要求误差在阈值内才记录到位。

## 问题二：PP set-point acknowledge 握手

PP 模式下，主站通过 `0x6040 Controlword` 的 bit4 发起新目标：

```text
bit4 = New set-point
```

驱动器通过 `0x6041 Statusword` 的 bit12 应答：

```text
bit12 = Set-point acknowledge
```

典型握手流程：

```text
主站：写 0x607A Target Position
主站：置位 0x6040 bit4
驱动器：置位 0x6041 bit12，表示收到新目标
主站：释放 0x6040 bit4
驱动器：释放 0x6041 bit12
```

本项目中增加了如下阶段：

- 保持 `0x000F` 若干周期。
- 写目标位置。
- 触发 PP set-point。
- 等待 `SW bit12 = 1`。
- 释放触发位。
- 等待 `SW bit12 = 0`。
- 等待运动完成。

## 问题三：仅使用 0x001F 会 acknowledge 但不运动

第一次完善握手后，控制字使用：

```text
CW=0x001F
```

即：

```text
0x000F + bit4 New set-point
```

日志显示驱动器能 acknowledge：

```text
[MOTION] cycle=25678 Set-point acknowledged, SW=0x1637 pos=-185 err=-4369266
[MOTION] cycle=31384 Set-point acknowledge released, SW=0x0637 pos=-185 err=-4369266
```

但实际位置始终不动，bit10 也没有进入预期的运动中状态。

说明该驱动器虽然接收了 set-point，但没有立即执行。

## 成功关键：使用 0x003F 触发 PP 目标

后续实验将 PP 触发控制字改为：

```text
CW=0x003F
```

即：

```text
0x000F + bit4 New set-point + bit5 Change set immediately
```

这个改动后，电机开始实际运动。

成功日志关键片段：

```text
[MOTION] cycle=31191 Set-point acknowledged, SW=0x1237 ModeDisp=1 pos=-278 err=-4369265
[TICK] cycle=31353 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=252417 Err=0x0000
[TICK] cycle=31556 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=691758 Err=0x0000
[TICK] cycle=31759 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=1135101 Err=0x0000
[TICK] cycle=31962 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=1578536 Err=0x0000
[TICK] cycle=32166 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=2024229 Err=0x0000
[TICK] cycle=32369 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=2467639 Err=0x0000
[TICK] cycle=32572 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=2911054 Err=0x0000
[TICK] cycle=32775 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=3354527 Err=0x0000
[TICK] cycle=32978 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=3797938 Err=0x0000
[TICK] cycle=33181 wkc=3 SW=0x1237 CW=0x000F ModeDisp=1 pos=4241424 Err=0x0000
[CHG] cycle=33193 SW:0x1237→0x1637 Err:0x0000→0x0000 CW=0x000F ModeDisp=1
[TICK] cycle=33384 wkc=3 SW=0x1637 CW=0x000F ModeDisp=1 pos=4366827 Err=0x0000
[TICK] cycle=33587 wkc=3 SW=0x1637 CW=0x000F ModeDisp=1 pos=4368726 Err=0x0000
[MOTION] cycle=33892 Set-point acknowledge released, SW=0x0637 ModeDisp=1 pos=4368793 err=-194
```

目标约为 `4368987` counts，最终实际位置约为 `4368793` counts，误差约 `-194` counts，已经在 `TARGET_TOLERANCE_COUNTS=1000` 范围内。

## 状态字解释

### 0x0637

```text
0x0637 = 0x0400 + 0x0200 + 0x0037
```

含义：

- Operation enabled。
- Remote。
- Target reached。

这是使能后或运动完成后的常见状态。

### 0x1637

```text
0x1637 = 0x1000 + 0x0400 + 0x0200 + 0x0037
```

含义：

- Set-point acknowledge = 1。
- Target reached = 1。
- Remote = 1。
- Operation enabled。

表示驱动器确认 set-point，同时仍认为目标 reached。之前使用 `0x001F` 时长期停在类似状态，说明没有真正运动。

### 0x1237

```text
0x1237 = 0x1000 + 0x0200 + 0x0037
```

含义：

- Set-point acknowledge = 1。
- Target reached = 0。
- Remote = 1。
- Operation enabled。

这是本次成功运行中最关键的状态，表示驱动器已确认新目标，并且 bit10 已清零，运动正在进行。

## ModeDisp 的作用

日志中增加了 `ModeDisp`，读取 `0x6061 Modes of Operation Display`。

成功日志显示：

```text
ModeDisp=1
```

说明驱动器实际工作模式是 PP 模式。

这排除了“写了 0x6060=1 但实际模式没有切换成功”的可能。

## 当前有效策略

当前成功策略为：

```text
进入 Operation enabled
保持 CW=0x000F 若干周期
写入 0x607A Target Position
触发 CW=0x003F
等待 SW bit12 = 1
释放 CW=0x000F
记录 SW bit10 是否曾经清零
等待 SW bit10 = 1 且位置误差进入阈值
完成
```

其中 `0x003F` 是关键：

```text
0x003F = 0x000F + bit4 + bit5
```

- bit4：New set-point。
- bit5：Change set immediately。

## 经验总结

1. CiA402 状态机进入 `Operation enabled` 只说明伺服使能成功，不代表 PP 目标会执行。
2. `0x6041 bit10` 不能单独作为本次运动完成依据。
3. PP 模式应使用 bit12 acknowledge 完成 set-point 握手。
4. 对该驱动器，仅 `0x001F` 不足以触发运动。
5. 该驱动器需要 `0x003F`，即同时置位 `New set-point` 和 `Change set immediately`。
6. `ModeDisp=1` 是确认实际 PP 模式的重要证据。
7. 运动完成判断应结合状态字和实际位置误差。

## 最终验证结果

最终测试中：

- EtherCAT WKC 正常。
- CiA402 状态机成功到 `Operation enabled`。
- `ModeDisp=1`。
- 使用 `CW=0x003F` 后出现 `SW=0x1237`。
- 实际位置从约 `-278` 增长到约 `4368793`。
- 最终误差约 `-194` counts。
- `Err=0x0000`，无驱动器错误。

结论：PP 模式位置运动已经成功。
