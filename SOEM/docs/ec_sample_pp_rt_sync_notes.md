# ec_sample PP 模式 RT 同步改进记录

## 背景

本次调试目标是在 `samples/ec_sample/ec_sample.c` 的 PP(Profile Position) 模式中加入与 `samples/test/csp_test/csp_test.c` 类似的 DC Sync0 同步，并解决加入同步后出现的状态机响应慢、进入 `Operation enabled` 后很久才运动的问题。

对比对象：

- `samples/test/csp_test/csp_test.c`
- `samples/ec_sample/ec_sample.c`

测试命令：

```bash
sudo ./build/samples/ec_sample/ec_sample enp0s31f6
```

## 问题现象

最初 `ec_sample` 在 PP 模式下存在两个明显问题：

- 进入 `Operation enabled` 需要较长时间。
- 进入 `Operation enabled` 后，`CW=0x003F` 已经持续输出很久，但电机迟迟不运动。

典型日志表现为：

```text
[TICK][AXIS1] ... SW=0x0637 (Operation enabled) CW=0x003F pos=21917 ...
...
[AXIS1] cycle=21713 Set-point acknowledged, SW=0x1237 ...
```

说明主站已经长时间尝试触发 PP set-point，但驱动很晚才置位 `6041h bit12 set-point acknowledged`。

## 根因分析

### 1. PP 状态机原来运行在主线程

原实现中：

- RT 线程只负责周期性 PDO 收发：

```text
receive -> mbxhandler -> send
```

- `axis_step()` 在 `run_motion()` 主线程中调用。

这会导致主线程写 `outputs`，RT 线程发送 `outputs`，两者不同步。

PP 模式依赖 `6040h bit4` 的上升沿触发新 set-point，如果 `Controlword`、`Target position`、`Mode of operation` 不是在同一个 PDO 周期内稳定写入并发送，驱动可能出现：

- 目标位置尚未稳定，`bit4` 已经被拉高。
- `bit4` 上升沿被驱动采样得很晚。
- `6041h bit12` 很久才置位。

### 2. 主线程节拍不等于 EtherCAT PDO 周期

原 `run_motion()` 中使用主线程 `osal_usleep(2000)` 推进状态机，实际调度周期受 Linux 调度、日志输出和 `tee` 影响，不能保证严格 1ms 或 2ms。

而 EtherCAT PDO RT 线程周期是：

```c
#define CYCLE_NS 1000000
```

即 1ms。

状态机节拍和 PDO 发送节拍不一致，会进一步放大 PP 触发不稳定的问题。

## 改进内容

### 1. 启用 DC Sync0

在 `ecx_configdc(&ctx)` 之后，对所有支持 DC 的从站启用 Sync0：

```c
for (int s = 1; s <= ctx.slavecount; s++)
   if (ctx.slavelist[s].hasdc)
   {
      ecx_dcsync0(&ctx, s, TRUE, CYCLE_NS, 0);
      printf("[BOOT] slave%d DC Sync0 enabled, period=%dns\n", s, CYCLE_NS);
   }
```

同步周期与 RT 线程周期保持一致：

```c
#define CYCLE_NS 1000000
```

### 2. 将 PP 状态机迁移到 RT 线程

新增全局状态：

```c
static axis_state_t         g_axes[MAX_MOTION_AXES];
static volatile int         run_axes = 0;
static int                  sync_hold_rt = 0;
```

RT 线程改为在 `receive` 后、`send` 前调用 `axis_step()`：

```text
ecx_receive_processdata()
ecx_mbxhandler()
axis_step()
ecx_send_processdata()
```

这样每个 1ms PDO 周期内都完成：

1. 读取最新状态字。
2. 推进 CiA402 / PP 状态机。
3. 写入 `Controlword`、`Target position`、`Mode of operation`。
4. 立即发送 PDO。

避免主线程和 RT 线程并发访问 `outputs` 造成的不同步问题。

### 3. `run_motion()` 改为观察者

修改后 `run_motion()` 不再直接调用 `axis_step()`，只负责：

- 初始化 `g_axes`。
- 置位 `run_axes = 1`。
- 每 1ms 检查 `reached_logged`。
- 到位后多保持 1 秒。
- 清除 `run_axes = 0`。

这样主线程不再直接写 PDO 输出缓冲区。

### 4. 增加 PP 目标预载周期

状态机迁移到 RT 线程后，进入 `Operation enabled` 变快，但曾出现电机不转的问题。

原因是 PP 触发过快：目标位置可能还没在 `CW=0x000F` 状态下稳定保持，`6040h bit4` 上升沿已经发出。

因此增加：

```c
#define PP_TARGET_PRELOAD_CYCLES 20
```

并在轴状态中增加：

```c
int preload_cnt;
```

流程变为：

```text
进入 Operation enabled
-> 写 Target position = final_target，同时保持 CW=0x000F
-> 预载 20 个 1ms RT 周期
-> 所有轴预载完成后再等 5 个 RT 周期
-> 拉高 CW bit4，发送 0x003F
-> 等待 6041h bit12 set-point acknowledged
-> 释放 bit4，回到 0x000F
-> 等待 bit12 释放、bit10 target reached
```

## 最终验证结果

修改后测试日志显示状态机和运动响应明显改善：

```text
[CHG][AXIS1] cycle=503 SW:0xFFFF→0x0670 ... CW=0x0006
[CHG][AXIS1] cycle=506 SW:0x0670→0x0631 ... CW=0x0007
[CHG][AXIS1] cycle=508 SW:0x0631→0x0633 ... CW=0x000F
[AXIS1] cycle=510 Op Enabled, base=4390362 → final=8759428
[AXIS1] cycle=535 同步触发 PP 定位 → 8759428 counts (30°), preload=20ms
[AXIS1] cycle=537 Set-point acknowledged, SW=0x1237 ...
[AXIS1] cycle=540 Set-point acknowledge released, SW=0x0237 ...
```

关键结果：

- 从开始推进状态机到 `Operation enabled` 只用了约 7 个 RT 周期。
- 从 `Operation enabled` 到 PP 触发为 25ms 左右。
- 触发后 2ms 左右收到 `set-point acknowledged`。
- 电机随后正常运动。
- 最终到位：

```text
[AXIS1] cycle=4798 Target Reached, pos=8758932 (期望 8759428, 误差 -496)
```

## 当前实现的关键行为

### RT 线程职责

RT 线程现在负责：

- 周期性接收 PDO。
- 周期性处理邮箱。
- 1ms 推进 PP 状态机。
- 周期性发送 PDO。

### 主线程职责

主线程现在负责：

- EtherCAT 初始化。
- PDO/DC/Sync0 配置。
- 进入 OP。
- 启动运动任务。
- 等待运动完成。
- 退出到 SAFE-OP / INIT。

## 注意事项

### PP 模式下不要过早拉高 bit4

对于当前驱动，建议保持以下顺序：

```text
Target position 写入
CW=0x000F 保持若干周期
CW=0x003F 触发 bit4 上升沿
等待 SW bit12
CW=0x000F 释放 bit4
等待 SW bit12 清零
等待 SW bit10 + 位置误差满足阈值
```

### `PP_TARGET_PRELOAD_CYCLES` 可作为调试参数

当前值：

```c
#define PP_TARGET_PRELOAD_CYCLES 20
```

如果后续多轴或不同驱动上仍出现触发不稳定，可以适当增大，例如 50ms。

如果确认驱动响应稳定，也可以尝试减小。

### RT 线程内不要做过重操作

当前仍保留日志打印，便于调试。

如果后续用于正式控制，应降低日志频率，避免 RT 线程被 `printf` 影响周期稳定性。

## 构建验证

修改后已通过构建：

```bash
cmake --build build --target ec_sample -j2
```

结果：

```text
[100%] Built target ec_sample
```

## 总结

本次改进的核心不是单纯启用 Sync0，而是让 PP 状态机与 PDO 发送形成确定的 1ms 闭环：

```text
receive -> axis_step -> send
```

最终效果：

- `Operation enabled` 进入速度明显提升。
- PP set-point acknowledge 响应从秒级/十几秒级缩短到毫秒级。
- 电机能够稳定启动并到位。
