# YH052 EtherCAT CiA‑402 关键对照表 + 日志与故障分析 + 操作建议

更新时间：自动生成于当前测试会话

## 1. CiA‑402 关键对象与语义对照

- **对象字典 (常用)**
  - 0x6040 控制字 CW (Controlword)
  - 0x6041 状态字 SW (Statusword)
  - 0x6060 工作模式 (Mode of Operation)
  - 0x6061 模式显示 (Mode of Operation Display)
  - 0x607A 目标位置 (Target Position)
  - 0x6064 实际位置 (Position Actual Value)
  - 0x606C 实际速度 (Velocity Actual Value)
  - 0x603F 错误码 (Error Code)
  - 0x60FD 数字输入 (Digital Inputs) – 位义需参考伺服手册

- **控制字 CW(0x6040) 常用位/组合**
  - bit0 Switch on
  - bit1 Enable voltage
  - bit2 Quick stop（置1=Quick stop 被激活，置0=释放）
  - bit3 Enable operation
  - bit4 New set-point（PP模式触发脉冲，需上升沿）
  - bit6 Relative（PP相对/绝对，0=绝对，1=相对）
  - bit7 Fault reset（上升沿复位故障）
  - 组合（本项目使用）：
    - 0x0080 Fault Reset
    - 0x0006 Shutdown（bit1=1, bit2=1）
    - 0x0007 Switch on（在 RSO 后）
    - 0x000F Enable operation（在 SO 后）
    - 0x001F PP 触发（在 OE 后，bit4 上升沿）

- **状态字 SW(0x6041) 关键位**
  - bit0 Ready to switch on (RSO)
  - bit1 Switched on (SO)
  - bit2 Operation enabled (OE)
  - bit3 Fault (F)
  - bit4 Voltage enabled (VE)
  - bit5 Quick stop (QS)（0=激活/触发中，1=释放）
  - bit6 Switch on disabled (SD)
  - bit7 Warning (WRN)
  - bit9 Remote (REM)
  - bit10 Target reached (TR)
  - 常用判别：
    - Switch on disabled: (SW & 0x004F) == 0x0040
    - Ready to switch on: (SW & 0x006F) == 0x0021
    - Switched on:        (SW & 0x006F) == 0x0023
    - Operation enabled:  (SW & 0x006F) == 0x0027

- **工作模式 (0x6060/0x6061)**
  - 1 = PP (Profile Position)
  - 8 = CSP (Cyclic Synchronous Position)
  - 本项目当前采用 PP=1；CSP 在设备上位机/固件条件不足时可能自动降级。

- **PDO 映射（本项目重建）**
  - RxPDO 0x1600: 0x6040:16, 0x607A:32, 0x6060:8（顺序即字节布局）
  - TxPDO 0x1A00: 0x6041:16, 0x6064:32, 0x606C:32, 0x603F:16, 0x6061:8
  - 字节序：小端（LE），16/32位按低字节在前依次写入/读取。

## 2. 近期终端关键输出片段（缩略）

```text
SAFE-OP 预初始化: SDO 写 0x6060=1 (wkc=1), 0x6040=0x80 (wkc=1)
SAFE-OP 回读: ModeDisp=1, SW=0x0650
EtherCAT OP

=== cycle= 1019 Wck=6 ... ===
  Slave1 RxPDO[ 7]: 06 00 2e ff ff ff 01
  Slave1 TxPDO[13]: 50 06 2e ff ff ff 00 00 00 00 00 00 01
    → CW=0x0006 SW=0x0650 状态=Switch on disabled pos=-210 Mode=1 Err=0x0000
       SW bits: RSO=0 SO=0 OE=0 F=0 VE=1 QS=0 SD=1 WRN=0 REM=1 TR=1
       DI(0x60FD)=0x00000000 (wkc=1)
```

要点：
- SW=0x0650，且解码位显示：
  - VE=1（电压已使能）、QS=0（Quick stop 激活/未释放）、SD=1（仍处于禁用）、REM=1（总线控制）。
- 数字输入 0x60FD=0x00000000（需参考手册位义），表现为所有输入均为低电平。

## 3. 与 CiA‑402 标准的符合性复核

- 控制字组合与状态机掩码使用均符合 CiA‑402（见第1节）。
- PDO 解码/编码使用小端序，偏移与映射一致。
- PP 模式触发流程正确：在 OE 后保持 0x000F 多周期，再用 0x001F 触发上升沿；目标位置采用绝对值，避免相对模式干扰。
- 进入 OP 前已通过 SDO 写 0x6060（模式）与 0x6040=0x80（故障复位），并预填 outputs 避免 PDO 覆盖。

结论：软件侧变量赋值与流程符合 CiA‑402 标准规范，但设备仍停在 Switch on disabled。

## 4. 错误成因可能性分析（按可能性排序）

- **硬件/安全回路未满足（最高可能）**
  - 由 SW 位：QS=0（Quick stop 激活/未释放）+ SD=1 可直接解释为何无论如何都停在 Switch on disabled。
  - 0x60FD 全零暗示：外部 24V I/O 未供电，或急停/使能/伺服 ON/限位/安全通道等输入全部为低。
- **STO/伺服ON 未闭合或未被参数允许**
  - 若需要外部短接/上拉，未满足将永久禁止使能。
- **限位触发或报警未显示到 0x603F**
  - 有些厂商报警通过面板/上位机展示，而 0x603F 只给最后错误码，且可能为 0。
- **CSP 同步导致自动降级（次要）**
  - 早期试验在 CSP 下见到设备回落至 PP，怀疑固件不完整或不同步；当前已改回 PP，仍卡住，说明核心在硬件/安全输入。

## 5. 建议的下一步操作（强烈建议按顺序执行）

1) **上位机直接“伺服 ON”测试（最快判定硬件状态）**
- 用厂家上位机/DriverStudio 连接，查看监视/报警页面：
  - 若也不能“伺服 ON”，则100%为硬件/参数问题。
  - 记录报警号与 DI 输入位状态（对应 0x60FD 位义）。

2) **检查/满足外部输入与安全回路**
- 确认 24V I/O 电源供给正常。
- 释放急停（E‑STOP/Quick‑Stop）：QS 应变为 1（释放）。
- 关闭/桥接 STO 安全通道（仅测试工装场景）。
- 拉高伺服 ON（S‑ON）/使能输入；解除限位输入。

3) **再次观察 SW 位与 0x60FD**
- QS 由 0→1 后，按 CiA‑402：
  - 发送 0x0006 → SW 应到 Ready to switch on (0x0021 掩码)。
  - 发送 0x0007 → Switched on。
  - 发送 0x000F → Operation enabled。
  - 发送 0x001F → 触发定位。

4) **若上位机能 ON 而 SOEM 不能**
- 提供上位机中 0x60FD 每一位的含义对照，我将把这些位打印到日志中逐一核对。
- 我也可在代码里增加对 0x6070（扭矩限制等，如有）/ 厂商自定义对象的轮询（需手册信息）。

5) **单片机回归测试（快速二分）**
- 用你的单片机再次接入当前这台伺服：
  - 若也不能 ON/转动：硬件/参数问题无疑。
  - 若能 ON：再对照 0x60FD 位与使能顺序差异。

## 6. 参考命令（不自动执行）

- 运行样例（需 sudo）：

```bash
sudo ./build/samples/ec_sample/ec_sample enp0s31f6
```

- 日志文件：`./ec_sample_run.log`

---

如需，我可以：
- 在日志中继续加入更多对象（你提供地址与位义）。
- 在“SW 位变化”触发时打印即时提示，便于和硬件操作配合。
