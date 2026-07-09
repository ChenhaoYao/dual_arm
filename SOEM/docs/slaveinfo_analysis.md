# `slaveinfo` 输出分析文档

> 记录命令：`sudo ./build/samples/slaveinfo/slaveinfo enp0s31f6 -map -sdo`
> 运行环境：Linux + SOEM v2.0.0
> 网卡：`enp0s31f6`
> 总线设备：2 台 DC 伺服驱动器（链型拓扑）

---

## 一、原始终端输出

```
SOEM (Simple Open EtherCAT Master)
Slaveinfo
Starting slaveinfo
ecx_init on enp0s31f6 succeeded.
2 slaves found and configured.
Calculated workcounter 6

Slave:1
 Name:DC Servo Drive
 Output size: 64bits
 Input size: 152bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:1.1.0.0
 Configured address: 1001
 Man: 00004321 ID: 00000036 Rev: 00010000
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1100 L: 128 F:00010022 Type:2
 SM2 A:1200 L:   8 F:00010064 Type:3
 SM3 A:1400 L:  19 F:00010020 Type:4
 FMMU0 Ls:00000000 Ll:   8 Lsb:0 Leb:7 Ps:1200 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000010 Ll:  19 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:01 Act:01
 FMMU2 Ls:00000036 Ll:   1 Lsb:0 Leb:7 Ps:080d Psb:0 Ty:01 Act:01
 FMMUfunc 0:2 1:1 2:3 3:0
 MBX length wr: 128 rd: 128 MBX protocols : 0c
 CoE details: 0f FoE details: 01 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Control Word
  [0x0002.0] 0x607A:0x00 0x20 INTEGER32    Profile Target Position
  [0x0006.0] 0x60B8:0x00 0x10 UNSIGNED16   Touch Probe Function
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0010.0] 0x603F:0x00 0x10 UNSIGNED16   Last Error Code
  [0x0012.0] 0x6041:0x00 0x10 UNSIGNED16   Status Word
  [0x0014.0] 0x6061:0x00 0x08 INTEGER8     Modes of Operation Display
  [0x0015.0] 0x6064:0x00 0x20 INTEGER32    Actual Motor Position
  [0x0019.0] 0x60B9:0x00 0x10 UNSIGNED16   Touch Probe Status
  [0x001B.0] 0x60BA:0x00 0x20 INTEGER32    Touch Probe 1 Positive Value
  [0x001F.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital Inputs

Slave:2
 Name:DC Servo Drive
 Output size: 64bits
 Input size: 152bits
 State: 4
 Delay: 660[ns]
 Has DC: 1
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1002
 Man: 00004321 ID: 00000036 Rev: 00010000
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1100 L: 128 F:00010022 Type:2
 SM2 A:1200 L:   8 F:00010064 Type:3
 SM3 A:1400 L:  19 F:00010020 Type:4
 FMMU0 Ls:00000008 Ll:   8 Lsb:0 Leb:7 Ps:1200 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000023 Ll:  19 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:01 Act:01
 FMMU2 Ls:00000037 Ll:   1 Lsb:0 Leb:7 Ps:080d Psb:0 Ty:01 Act:01
 FMMUfunc 0:2 1:1 2:3 3:0
 MBX length wr: 128 rd: 128 MBX protocols : 0c
 CoE details: 0f FoE details: 01 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0008.0] 0x6040:0x00 0x10 UNSIGNED16   Control Word
  [0x000A.0] 0x607A:0x00 0x20 INTEGER32    Profile Target Position
  [0x000E.0] 0x60B8:0x00 0x10 UNSIGNED16   Touch Probe Function
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0023.0] 0x603F:0x00 0x10 UNSIGNED16   Last Error Code
  [0x0025.0] 0x6041:0x00 0x10 UNSIGNED16   Status Word
  [0x0027.0] 0x6061:0x00 0x08 INTEGER8     Modes of Operation Display
  [0x0028.0] 0x6064:0x00 0x20 INTEGER32    Actual Motor Position
  [0x002C.0] 0x60B9:0x00 0x10 UNSIGNED16   Touch Probe Status
  [0x002E.0] 0x60BA:0x00 0x20 INTEGER32    Touch Probe 1 Positive Value
  [0x0032.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital Inputs
End slaveinfo, close socket
End program
```

---

## 二、启动与初始化

```
ecx_init on enp0s31f6 succeeded.
2 slaves found and configured.
Calculated workcounter 6
```

| 字段 | 含义 |
|---|---|
| `ecx_init ... succeeded` | SOEM 在网卡 `enp0s31f6` 上成功打开原始套接字，初始化主站。 |
| `2 slaves found and configured` | 通过自动扫描，在 EtherCAT 总线上识别到 2 个从站，并完成 INIT→PRE-OP 阶段的基础配置。 |
| `Calculated workcounter 6` | **预期工作计数器（Expected WKC）**。每个从站对一个 LRW/LRD/LWR 命令的成功响应都会让 WKC 累加：读 +1、写 +2、读写 +3。这里 2 个从站各贡献 3（有读有写），合计 **6**。运行时实际收到的 WKC 必须等于此值才能确认所有 IO 同步成功。 |

---

## 三、从站 1 —— 基本属性

```
Slave:1
 Name:DC Servo Drive
 Output size: 64bits
 Input size: 152bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:1.1.0.0
 Configured address: 1001
 Man: 00004321 ID: 00000036 Rev: 00010000
```

| 字段 | 含义 |
|---|---|
| `Name` | 从站名（来自 EEPROM）。`DC Servo Drive` 表示直流伺服驱动器。 |
| `Output size: 64bits` | **主站→从站** 过程数据大小：64 bit = 8 字节（对应下方 SM2 outputs：Control Word 16 + Target Position 32 + Touch Probe Function 16 = 64 bit）。 |

# Touch Probe Function 是什么？
Touch Probe Function 是一个用于触觉探针（Touch Probe）功能的控制字，用于控制探针的触发和状态。

| `Input size: 152bits` | **从站→主站** 过程数据：152 bit = 19 字节（对应 SM3 inputs 各对象之和）。 |
| `State: 4` | EtherCAT 状态机当前状态，值含义：`1=INIT, 2=PRE-OP, 4=SAFE-OP, 8=OPERATIONAL`。**4 = SAFE-OP**（输入有效、输出尚未启用）。`slaveinfo` 默认只升到 SAFE-OP，不会进入 OP。 |
| `Delay: 0[ns]` | DC（分布式时钟）传播延迟。第一台从站为参考点，所以是 0 ns。 |
| `Has DC: 1` | 从站支持分布式时钟。 |
| `DCParentport: 0` | 此从站通过 **父节点的 port 0** 接入网络（拓扑信息）。 |
| `Activeports: 1.1.0.0` | 从站 4 个端口（A/B/C/D 即 0/1/2/3）的连接状态：1=有链路、0=无。`1.1.0.0` 表示端口 0 和 1 接通，是链型拓扑中段节点的典型表现。 |
| `Configured address: 1001` | 主站给从站分配的 **Station Address**（递增分配，1001、1002…）。所有后续点对点报文用它寻址。 |
| `Man / ID / Rev` | 厂商 ID、产品码、版本号（来自 EEPROM 的 vendor/product/revision）。`Man:00004321 ID:00000036` 用于 ENI 匹配。 |

---

## 四、从站 1 —— 同步管理器（SyncManager, SM）

```
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1100 L: 128 F:00010022 Type:2
 SM2 A:1200 L:   8 F:00010064 Type:3
 SM3 A:1400 L:  19 F:00010020 Type:4
```

`SMx` 是 EtherCAT 从站芯片（ESC）中的内存通道，用来在主站与从站之间交换数据。每行字段：

- `A`：起始地址（从站 ESC 内的物理地址，十六进制）
- `L`：长度（字节）
- `F`：控制寄存器值（含方向、模式、中断使能等位）
- `Type`：用途，`1=Mailbox Out (主→从)`, `2=Mailbox In (从→主)`, `3=Process Output`, `4=Process Input`

| 通道 | 含义 |
|---|---|
| `SM0 @0x1000 L=128 Type=1` | **邮箱输出**。主站通过它发送 CoE/FoE 等异步请求，128 字节（与下方 `MBX length wr:128` 一致）。 |
| `SM1 @0x1100 L=128 Type=2` | **邮箱输入**。从站把响应放在这里。 |
| `SM2 @0x1200 L=8  Type=3` | **过程数据输出（PDO Out）**。8 字节正好对应 64 bit Output。 |
| `SM3 @0x1400 L=19 Type=4` | **过程数据输入（PDO In）**。19 字节对应 152 bit Input。 |

---

## 五、从站 1 —— FMMU 映射

```
 FMMU0 Ls:00000000 Ll:   8 Lsb:0 Leb:7 Ps:1200 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000010 Ll:  19 Lsb:0 Leb:7 Ps:1400 Psb:0 Ty:01 Act:01
 FMMU2 Ls:00000036 Ll:   1 Lsb:0 Leb:7 Ps:080d Psb:0 Ty:01 Act:01
 FMMUfunc 0:2 1:1 2:3 3:0
```

**FMMU**（Fieldbus Memory Management Unit）负责把主站的"逻辑地址空间"映射到从站本地的"物理地址空间"，是 LRW/LRD/LWR 报文寻址的核心。

| 字段 | 含义 |
|---|---|
| `Ls` | **L**ogical **s**tart：在主站逻辑地址空间中的起始地址 |
| `Ll` | **L**ogical **l**ength：长度（字节） |
| `Lsb / Leb` | 起始位偏移 / 结束位偏移（0~7，0–7 表整字节） |
| `Ps` | **P**hysical **s**tart：从站本地物理地址 |
| `Psb` | 物理起始位偏移 |
| `Ty` | 类型：`01=读（输入）`、`02=写（输出）`、`03=读写` |
| `Act` | 是否激活 |

逐项含义：

- **FMMU0**：主站逻辑 `0x0000~0x0007`（共 8 B）→ 从站 `0x1200`（即 SM2 输出区），方向"写"。即主站把 8 字节输出 PDO 写到从站。
- **FMMU1**：主站逻辑 `0x0010~0x0022`（19 B）→ 从站 `0x1400`（SM3 输入区），方向"读"。读取输入 PDO。
- **FMMU2**：主站逻辑 `0x0036` 1 字节 → 从站 `0x080D`，方向"读"。这是从站状态映射位，常用于把状态压缩进 LRW 报文，便于主站一次扫描所有从站状态。
- **FMMUfunc 0:2 1:1 2:3 3:0**：4 个 FMMU 通道分别配置为功能 2/1/3/0（输出/输入/状态/未用）。

---

## 六、从站 1 —— 邮箱与协议支持

```
 MBX length wr: 128 rd: 128 MBX protocols : 0c
 CoE details: 0f FoE details: 01 EoE details: 00 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR: 0
```

| 字段 | 含义 |
|---|---|
| `MBX length` | 邮箱缓冲区大小（写/读各 128 字节）。 |
| `MBX protocols: 0c` | 支持的邮箱协议位掩码。`0x0C = 0b00001100` → 第 2 位 **CoE**（CANopen over EtherCAT）+ 第 3 位 **FoE**（File over EtherCAT）。其它位：AoE=0x01, EoE=0x02, SoE=0x10, VoE=0x20。 |
| `CoE details: 0f` | CoE 子功能位：`0x0F = 0b00001111` → 支持 SDO、SDO Info、PDO Assign、PDO Configure。功能完备。 |
| `FoE details: 01` | 支持 FoE（用于固件更新）。 |
| `EoE/SoE: 00` | 不支持 Ethernet/Servo over EtherCAT。 |
| `Ebus current: 0[mA]` | 该从站从 E-Bus 取电的电流（这里 0 mA，说明它是独立供电的伺服驱动，非 E-Bus 取电模块）。 |
| `only LRD/LWR: 0` | 是否仅支持单向逻辑读/写。0 表示也支持 LRW（读写合并），有助于减少报文数。 |

---

## 七、从站 1 —— PDO 映射（核心）

```
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Control Word
  [0x0002.0] 0x607A:0x00 0x20 INTEGER32    Profile Target Position
  [0x0006.0] 0x60B8:0x00 0x10 UNSIGNED16   Touch Probe Function
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0010.0] 0x603F:0x00 0x10 UNSIGNED16   Last Error Code
  [0x0012.0] 0x6041:0x00 0x10 UNSIGNED16   Status Word
  [0x0014.0] 0x6061:0x00 0x08 INTEGER8     Modes of Operation Display
  [0x0015.0] 0x6064:0x00 0x20 INTEGER32    Actual Motor Position
  [0x0019.0] 0x60B9:0x00 0x10 UNSIGNED16   Touch Probe Status
  [0x001B.0] 0x60BA:0x00 0x20 INTEGER32    Touch Probe 1 Positive Value
  [0x001F.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital Inputs
```

这是通过 **CoE（CANopen over EtherCAT）** 读取的 PDO（Process Data Object）映射表，告诉你**主站 IOmap 中每个字节对应哪个对象字典条目**。

字段含义：

- `[0x0000.0]`：在主站 IOmap 中的偏移 `字节.位`（即 FMMU0 的 `Ls` 起点）
- `0x6040:0x00`：CANopen 对象字典索引:子索引
- `0x10` / `0x20` / `0x08`：位长度（16/32/8 bit）
- `data_type` / `name`：数据类型与人类可读的名称

### 输出 PDO（主站写给从站，8 字节）

| IOmap 偏移 | 对象 | 含义 |
|---|---|---|
| `0x0000` (2 B) | `0x6040` Control Word | **CiA402 控制字**：用于操作伺服状态机（使能、运行、急停等）。例如写 `0x06→0x07→0x0F` 可使驱动器从 `Switch on disabled` 走到 `Operation enabled`。 |
| `0x0002` (4 B) | `0x607A` Profile Target Position | **目标位置**（PP/CSP 模式下使用）。 |
| `0x0006` (2 B) | `0x60B8` Touch Probe Function | **探针功能配置**：使能哪一路探针、触发沿等。 |

### 输入 PDO（从站送给主站，19 字节）

| IOmap 偏移 | 对象 | 含义 |
|---|---|---|
| `0x0010` (2 B) | `0x603F` Last Error Code | 最近一次错误码 |
| `0x0012` (2 B) | `0x6041` Status Word | **CiA402 状态字**：反映伺服当前状态机位置 |
| `0x0014` (1 B) | `0x6061` Modes of Operation Display | 当前运行模式：1=PP, 6=Homing, 8=CSP, 9=CSV, 10=CST… |
| `0x0015` (4 B) | `0x6064` Actual Motor Position | 实际位置反馈（编码器单位） |
| `0x0019` (2 B) | `0x60B9` Touch Probe Status | 探针状态 |
| `0x001B` (4 B) | `0x60BA` Touch Probe 1 Positive Value | 探针 1 正沿锁存位置 |
| `0x001F` (4 B) | `0x60FD` Digital Inputs | 数字输入状态（通常包括限位/原点/急停信号） |

> 注意：输入 PDO 偏移从 `0x0010` 开始，而非 `0x0008`。这是因为 SOEM 默认将每个从站的输入和输出在 IOmap 中**分别成块**布局，且会做对齐。两个从站的输出连续放在前面，两个从站的输入连续放在后面。

---

## 八、从站 2 —— 与从站 1 的差异

```
Slave:2
 ...
 Delay: 660[ns]
 DCParentport:1
 Activeports:1.0.0.0
 Configured address: 1002
 FMMU0 Ls:00000008 Ll:   8 ...
 FMMU1 Ls:00000023 Ll:  19 ...
 FMMU2 Ls:00000037 Ll:   1 ...
```

Activeports 一个电机最多可以接4个口，接上了是1,没接上是0

- **`Delay: 660 ns`**：相对于参考时钟（从站 1）的传输延迟为 660 纳秒。DC 校时时主站会把这 660 ns 补偿到从站 2 的本地时钟，让两台伺服时钟同步。线越长 / 经过的从站越多，延迟越大。
- **`DCParentport: 1`**：通过父节点（即从站 1）的 **port 1** 进入。
- **`Activeports: 1.0.0.0`**：仅 port 0 有链路，是**链尾节点**。
- **`Configured address: 1002`**：递增分配。
- **FMMU 逻辑地址递进**：
  - 输出 FMMU0 起 `0x0008`（接在从站 1 的 8 B 输出后）
  - 输入 FMMU1 起 `0x0023`（= `0x0010` + 19，接在从站 1 输入后）
  - 状态 FMMU2 起 `0x0037`

整个 **IOmap 布局**：

```
偏移          内容
0x0000~0x0007  从站1 输出 PDO  (8B)
0x0008~0x000F  从站2 输出 PDO  (8B)
0x0010~0x0022  从站1 输入 PDO  (19B)
0x0023~0x0035  从站2 输入 PDO  (19B)
0x0036~0x0036  从站1 状态压缩位
0x0037~0x0037  从站2 状态压缩位
```

---

## 九、结尾

```
End slaveinfo, close socket
End program
```

`slaveinfo` 退出前关闭原始套接字，正常结束。

---

## 十、总体结论与建议

1. **总线健康**：2 台同型号 DC 伺服驱动 (`Man:0x4321 ID:0x36`)，链型拓扑（主站 → 从站1 → 从站2），DC 同步可用。
2. **可以直接进入 OP**：所有 SM/FMMU/PDO 已配置完毕，IO 大小一致，下一步可用 `simple_ng` 或 `ec_sample` 把状态升到 OPERATIONAL，开始周期性 IO 通信：
   ```bash
   sudo ./build/samples/simple_ng/simple_ng enp0s31f6
   ```
3. **后续控制电机的关键 PDO**：写 `0x6040 Control Word` + `0x607A Target Position`，读 `0x6041 Status Word` + `0x6064 Actual Position` 即可实现 PP/CSP 模式的位置控制。
4. **`-sdo` 没有打印 SDO 列表的原因**：`slaveinfo` 的 `-sdo` 选项需要从站支持 **SDO Info Service**。日志里 `CoE details: 0f` 是包含 SDO Info 的，但有些固件不返回完整对象字典，可单独再跑 `slaveinfo enp0s31f6 -sdo` 确认。
