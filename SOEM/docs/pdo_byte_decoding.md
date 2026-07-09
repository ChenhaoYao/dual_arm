# PDO 字节流解读笔记：字节序与 CiA 402 Status Word

> 背景：`ec_sample` 运行时打印一行  
> `I: 00 00 50 02 00 2b ff ff`  
> 看起来 Status Word 的字节是 `50 02`，但解码后是 `0x0250`。  
> 这份笔记把"为什么倒过来"和"位含义出自哪里"两件事讲清楚。

---

## 一、两件不同层次的事

| 问题 | 涉及层次 |
|---|---|
| `50 02` 为何要解读成 `0x0250` | **硬件存储层**：字节序 (Endianness) |
| `0x0250` 每一位代表什么状态 | **协议应用层**：CiA 402 设备行为标准 |

这两件事相互独立，下面分别说明。

---

## 二、字节序：人写数字 vs 机器存数字

### 人类写法（高位在左）

```
0x  02   50
    ↑    ↑
   高位  低位
```

### 计算机存法（小端，低位在前）

```c
uint16_t x = 0x0250;
```

内存里：

```
地址:  +0   +1
字节:  50   02
       ↑    ↑
      低位  高位
```

小端 = **L**ittle-end **first** = 低字节先存。

### `printf` 按地址顺序打印 → 你看到的就是字节顺序

```
I:  00 00  50 02  00  2b ff ff
    └─┬─┘ └─┬─┘  └┬┘ └────┬────┘
    地址 0  地址 2 地址 4    地址 5..
```

所以 `50 02` 是**字节序列**，不是数值。还原成数值要按"高位在左"的写法重排：

```
50 02   →   02 50   →   0x0250
(字节序)    (高位在左)      (数值)
```

**注意**：这个"反过来"只发生在你脑子里。
机器从伺服 MCU → 网线 → 内存 → 屏幕全程都是 `50, 02` 这个顺序，没有任何字节翻转。

---

## 三、全链路都是小端，没有任何翻转操作

```
伺服 MCU (小端 ARM)
   Status Word = 0x0250
        │
        │ 写入 ESC RAM:  addr+0 = 0x50, addr+1 = 0x02
        ▼
EtherCAT 帧 (小端规范, 字节按地址顺序串行发出)
        │
        │ 线缆字节流:  0x50 → 0x02
        ▼
主站网卡 / SOEM
   IOmap[2] = 0x50, IOmap[3] = 0x02
        │
        │ printf 按地址打印
        ▼
屏幕显示: "50 02"
        │
        │ 你心算: 倒读 = 0x0250    ← 唯一的"反向"操作, 只在你脑子里
        ▼
得到数值 0x0250
```

SOEM 源码里 `etohs / etohl / htoes / htoel` 这些宏在小端主机上是空操作，
见 `@/home/dell/SOEM/include/soem/ec_type.h:513-518`：

```c
#define htoes(A)  (A)
#define htoel(A)  (A)
#define etohs(A)  (A)
#define etohl(A)  (A)
```

只有在大端主机（如 PowerPC）上才会真的执行字节翻转。

---

## 四、字节序对照表（参考用，与本工程无关）

| 字节序 | 代表 |
|---|---|
| **小端 (Little Endian)** | x86 / x86_64 / ARM (默认) / EtherCAT / CANopen |
| 大端 (Big Endian) | PowerPC / 早期 MIPS / TCP/IP 协议头 / Profinet |

你的整条链路（x86 Linux 主机 + EtherCAT + CiA402 伺服）**全程小端**，
PowerPC / TCP 字节序与本工程无关，列在这里只是字节序"反例"对照。

---

## 五、Status Word 的位含义：来自 CiA 402 标准

`0x6041 Status Word` 的每一位含义由 **CiA 402** 规定，与 EtherCAT 本身无关。

### 标准来源

| 标准号 | 名称 | 发布方 |
|---|---|---|
| **CiA 402-2** | CANopen device profile for drives and motion control | CAN in Automation |
| **IEC 61800-7-201** | 工业标准等价编号 | IEC |

任何品牌的 CiA 402 伺服（汇川 / 台达 / 安川 / 贝加莱 / ELMO / Maxon ...）
`0x6041` 的位定义都一样。

### 协议栈位置

```
┌─────────────────────────────────────────────────┐
│  CiA 402 (设备行为: 伺服状态机、对象 0x6040…)    │  ← Status Word 位定义在这层
├─────────────────────────────────────────────────┤
│  CiA 301 (CANopen 对象字典模型)                  │  ← 索引/子索引结构
├─────────────────────────────────────────────────┤
│  传输层: EtherCAT + CoE  (或裸 CAN bus)          │  ← 数据怎么搬
├─────────────────────────────────────────────────┤
│  物理层: 100BASE-TX 以太网                       │
└─────────────────────────────────────────────────┘
```

---

## 六、0x0250 的位解码

```
0x0250 = 0000 0010 0101 0000 (二进制)
         FEDC BA98 7654 3210 (位号)
```

| 位 | 名称 | 值 | 含义 |
|---|---|---|---|
| 0 | Ready to switch on | 0 | 未准备好 |
| 1 | Switched on | 0 | 未上电 |
| 2 | Operation enabled | 0 | 未使能 |
| 3 | Fault | 0 | 无故障 ✓ |
| 4 | Voltage enabled | **1** | 主回路有电 ✓ |
| 5 | Quick stop | 0 | — |
| 6 | Switch on disabled | **1** | **当前状态** |
| 7 | Warning | 0 | 无警告 |
| 8 | Manufacturer-specific | 0 | — |
| 9 | Remote | **1** | 接受远程控制 ✓ |
| 10 | Target reached | 0 | — |
| 11 | Internal limit active | 0 | — |
| 12-15 | Mode-specific | 0 | — |

综合状态：**"Switch on disabled"**，等待主站发 Control Word 把它推进到
`Ready to switch on → Switched on → Operation enabled`。

CiA 402 标准伺服状态机的推进命令（写入 `0x6040 Control Word`）：

| Control Word | 目标状态 |
|---|---|
| `0x06` | Ready to switch on |
| `0x07` | Switched on |
| `0x0F` | Operation enabled (使能) |
| `0x80` | Fault reset (清故障) |

---

## 七、完整字节流解码示例

```
I:  00 00  50 02  00  2b ff ff ...
```

| 偏移 | 字节 | 对象 | 类型 | 倒读数值 | 含义 |
|---|---|---|---|---|---|
| 0-1 | `00 00` | `0x603F` Last Error Code | UINT16 | `0x0000` | 无错误 |
| 2-3 | `50 02` | `0x6041` Status Word | UINT16 | **`0x0250`** | Switch on disabled |
| 4 | `00` | `0x6061` Modes Display | INT8 | `0x00` | 无工作模式 |
| 5-8 | `2b ff ff ??` | `0x6064` Actual Position | INT32 | `0xFFFFFF2B` ≈ -213 | 电机停在原点附近 |

---

## 八、记忆口诀

> **小端字节流，倒着拼数值**
> **位定义查 CiA 402，与 EtherCAT 无关**
> **机器全程小端，"倒过来"只在脑子里**

---

## 相关文件

- 字节序宏定义：`@/home/dell/SOEM/include/soem/ec_type.h:513-518`
- PDO 解析示例：`@/home/dell/SOEM/samples/ec_sample/ec_sample.c`
- 默认 PDO 映射表：`@/home/dell/SOEM/docs/slaveinfo_analysis.md`
- PO2SOconfig 钩子机制：`@/home/dell/SOEM/docs/PO2SO.txt`
