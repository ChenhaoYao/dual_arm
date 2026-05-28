/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

/** \file
 * \brief
 * Headerfile for ec_main.c
 */

#ifndef _ec_main_
#define _ec_main_

#ifdef __cplusplus
extern "C" {
#endif

#include "soem/ec_options.h"

/** 网络适配器结构体
 * 用于存储网络适配器的信息
 */
typedef struct ec_adapter ec_adaptert;
struct ec_adapter
{
   /** 适配器名称 */
   char name[EC_MAXLEN_ADAPTERNAME];
   /** 适配器描述 */
   char desc[EC_MAXLEN_ADAPTERNAME];
   /** 指向下一个适配器的指针 */
   ec_adaptert *next;
};

/** FMMU（现场总线内存管理单元）记录结构体
 * 
 * FMMU是ESC芯片内的硬件单元，是第二层桥梁的核心：
 *   第一层: CANopen索引 → SM物理寄存器    (MCU固件完成，理解数据含义)
 *   第二层: SM物理寄存器 ↔ 逻辑地址       (FMMU硬件完成，只搬运字节) ← 本结构体
 *   第三层: 逻辑地址 → IOmap指针          (SOEM软件完成)
 * 
 * FMMU不理解数据的含义，只做地址翻译：逻辑地址范围 ↔ SM物理寄存器范围
 * 帧经过ESC时，FMMU硬件自动按此配置执行地址匹配和数据搬运（纳秒级，无需CPU参与）
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_fmmu
{
   /** 逻辑起始地址（如电机1输出=0x00，电机2输出=0x07，电机1输入=0x0E） */
   uint32 LogStart;
   /** 逻辑长度，单位字节（如输出PDO=7，输入PDO=13） */
   uint16 LogLength;
   /** 逻辑起始位（位级映射时使用，字节对齐从站通常为0） */
   uint8 LogStartbit;
   /** 逻辑结束位（字节对齐从站通常为7） */
   uint8 LogEndbit;
   /** 物理起始地址，即SM缓冲区在ESC内的地址（如SM2=0x0C00，SM3=0x1000） */
   uint16 PhysStart;
   /** 物理起始位（通常为0） */
   uint8 PhysStartBit;
   /** FMMU类型：1=读（输入，从站→主站），2=写（输出，主站→从站） */
   uint8 FMMUtype;
   /** FMMU激活标志：1=激活，0=未使用 */
   uint8 FMMUactive;
   /** 未使用 */
   uint8 unused1;
   /** 未使用 */
   uint16 unused2;
} ec_fmmut;
OSAL_PACKED_END

/** 同步管理器（SM）记录结构体
 * 用于配置从站的同步管理器
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_sm
{
   /** 起始地址 */
   uint16 StartAddr;
   /** SM 长度 */
   uint16 SMlength;
   /** SM 标志 */
   uint32 SMflags;
} ec_smt;
OSAL_PACKED_END

/** 状态和状态码结构体
 * 用于存储从站的状态和 AL 状态码
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_state_status
{
   /** 从站状态 */
   uint16 State;
   /** 未使用 */
   uint16 Unused;
   /** AL（Application Layer）状态码 */
   uint16 ALstatuscode;
} ec_state_status;
OSAL_PACKED_END

/** mailbox buffer array */
typedef uint8 ec_mbxbuft[EC_MAXMBX + 1];

/** 邮箱输入使能标志，用于指示邮箱输入缓冲区已启用 */
#define EC_MBXINENABLE (uint8 *)1

/** 邮箱池结构体
 * 用于管理邮箱缓冲区的池
 */
typedef struct
{
   /** 列表头、列表尾、列表计数 */
   int listhead, listtail, listcount;
   /** 邮箱空列表 */
   int mbxemptylist[EC_MBXPOOLSIZE];
   /** 邮箱互斥锁 */
   osal_mutext *mbxmutex;
   /** 邮箱缓冲区数组 */
   ec_mbxbuft mbx[EC_MBXPOOLSIZE];
} ec_mbxpoolt;

/** 邮箱队列状态：无操作 */
#define EC_MBXQUEUESTATE_NONE 0
/** 邮箱队列状态：请求中 */
#define EC_MBXQUEUESTATE_REQ  1
/** 邮箱队列状态：失败 */
#define EC_MBXQUEUESTATE_FAIL 2
/** 邮箱队列状态：完成 */
#define EC_MBXQUEUESTATE_DONE 3

/** 邮箱队列结构体
 * 用于管理邮箱的发送队列
 */
typedef struct
{
   /** 列表头、列表尾、列表计数 */
   int listhead, listtail, listcount;
   /** 邮箱指针数组 */
   ec_mbxbuft *mbx[EC_MBXPOOLSIZE];
   /** 邮箱状态数组 */
   int mbxstate[EC_MBXPOOLSIZE];
   /** 邮箱移除标志数组 */
   int mbxremove[EC_MBXPOOLSIZE];
   /** 邮箱票据数组 */
   int mbxticket[EC_MBXPOOLSIZE];
   /** 邮箱对应的从站编号数组 */
   uint16 mbxslave[EC_MBXPOOLSIZE];
   /** 邮箱互斥锁 */
   osal_mutext *mbxmutex;
} ec_mbxqueuet;

/** 邮箱协议：ADS over EtherCAT */
#define ECT_MBXPROT_AOE      0x0001
/** 邮箱协议：Ethernet over EtherCAT */
#define ECT_MBXPROT_EOE      0x0002
/** 邮箱协议：CANopen over EtherCAT */
#define ECT_MBXPROT_COE      0x0004
/** 邮箱协议：File over EtherCAT */
#define ECT_MBXPROT_FOE      0x0008
/** 邮箱协议：Servo over EtherCAT */
#define ECT_MBXPROT_SOE      0x0010
/** 邮箱协议：VoE (Vendor over EtherCAT) */
#define ECT_MBXPROT_VOE      0x0020

/** CoE 检测标志：支持 SDO */
#define ECT_COEDET_SDO       0x01
/** CoE 检测标志：支持 SDO 信息服务 */
#define ECT_COEDET_SDOINFO   0x02
/** CoE 检测标志：支持 PDO 分配 */
#define ECT_COEDET_PDOASSIGN 0x04
/** CoE 检测标志：支持 PDO 配置 */
#define ECT_COEDET_PDOCONFIG 0x08
/** CoE 检测标志：支持上传 */
#define ECT_COEDET_UPLOAD    0x10
/** CoE 检测标志：支持 SDO 完全访问 */
#define ECT_COEDET_SDOCA     0x20

/** 同步管理器使能掩码，用于清除使能位 */
#define EC_SMENABLEMASK      0xfffeffff

typedef struct ecx_context ecx_contextt; // 定义一个别称，避免直接使用struct ecx_context

/** 邮箱处理程序状态：无 */
#define ECT_MBXH_NONE   0
/** 邮箱处理程序状态：循环任务 */
#define ECT_MBXH_CYCLIC 1
/** 邮箱处理程序状态：从站丢失 */
#define ECT_MBXH_LOST   2

/** 从站状态结构体
 * 包含从站的所有信息，用于大多数与从站交互的用户操作
 * TODO 函数内部不应该用DOXYGON的注释格式
 */
typedef struct ec_slave
{
   /** 从站状态 */
   uint16 state;
   /** AL（Application Layer）状态码 */
   uint16 ALstatuscode;
   /** 配置地址 */
   uint16 configadr;
   /** 别名地址 */
   uint16 aliasadr;
   /** 制造商ID（来自EEPROM） */
   uint32 eep_man;
   /** 设备ID（来自EEPROM） */
   uint32 eep_id;
   /** 版本号（来自EEPROM） */
   uint32 eep_rev;
   /** 序列号（来自EEPROM） */
   uint32 eep_ser;
   /** 接口类型 */
   uint16 Itype;
   /** 设备类型 */
   uint16 Dtype;
   /** 输出位数 */
   uint16 Obits;
   /** 输出字节数，如果 Obits < 8 则 Obytes = 0 */
   uint32 Obytes;
   /** 输出指针，指向 IOmap 缓冲区 */
   uint8 *outputs;
   /** 输出偏移量，在 IOmap 缓冲区中的偏移 */
   uint32 Ooffset;
   /** 第一个输出字节的起始位 */
   uint8 Ostartbit;
   /** 输入位数 */
   uint16 Ibits;
   /** 输入字节数，如果 Ibits < 8 则 Ibytes = 0 */
   uint32 Ibytes;
   /** 输入指针，指向 IOmap 缓冲区 */
   uint8 *inputs;
   /** 输入偏移量，在 IOmap 缓冲区中的偏移 */
   uint32 Ioffset;
   /** 第一个输入字节的起始位 */
   uint8 Istartbit;
   /** 同步管理器（SM）结构体 */
   ec_smt SM[EC_MAXSM];
   /** SM类型：0=未使用 1=邮箱写 2=邮箱读 3=输出 4=输入 */
   uint8 SMtype[EC_MAXSM];
   /** FMMU（现场总线内存管理单元）结构体 */
   ec_fmmut FMMU[EC_MAXFMMU];
   /** FMMU0功能：0=未使用 1=输出 2=输入 3=SM状态 */
   uint8 FMMU0func;
   /** FMMU1功能 */
   uint8 FMMU1func;
   /** FMMU2功能 */
   uint8 FMMU2func;
   /** FMMU3功能 */
   uint8 FMMU3func;
   /** 写邮箱长度（字节），如果没有邮箱则为 0 */
   uint16 mbx_l;
   /** 邮箱写偏移 */
   uint16 mbx_wo;
   /** 读邮箱长度（字节） */
   uint16 mbx_rl;
   /** 邮箱读偏移 */
   uint16 mbx_ro;
   /** 邮箱支持的协议 */
   uint16 mbx_proto;
   /** 邮箱链路层协议计数器值，范围 1..7 */
   uint8 mbx_cnt;
   /** 是否支持分布式时钟（DC） */
   boolean hasdc;
   /** 物理类型：Ebus、EtherNet 组合 */
   uint8 ptype;
   /** 拓扑结构：1 到 3 个链路 */
   uint8 topology;
   /** 活动端口位图：....3210，相应端口活动时置位 */
   uint8 activeports;
   /** 消耗端口位图：....3210，用于内部延迟测量 */
   uint8 consumedports;
   /** 父从站编号，0=主站 */
   uint16 parent;
   /** 此从站连接到的父从站端口号 */
   uint8 parentport;
   /** 父从站连接到此从站的端口号 */
   uint8 entryport;
   /** 端口 A 的 DC 接收时间 */
   int32 DCrtA;
   /** 端口 B 的 DC 接收时间 */
   int32 DCrtB;
   /** 端口 C 的 DC 接收时间 */
   int32 DCrtC;
   /** 端口 D 的 DC 接收时间 */
   int32 DCrtD;
   /** 传播延迟 */
   int32 pdelay;
   /** 下一个 DC 从站 */
   uint16 DCnext;
   /** 上一个 DC 从站 */
   uint16 DCprevious;
   /** DC 周期时间（纳秒） */
   int32 DCcycle;
   /** DC 相对于时钟模边界的偏移 */
   int32 DCshift;
   /** DC 同步激活：0=关闭，1=开启 */
   uint8 DCactive;
   /** 链接到 SII 配置 */
   uint16 SIIindex;
   /** 1 = 每次读取 8 字节，0 = 每次读取 4 字节 */
   uint8 eep_8byte;
   /** 0 = eeprom 到主站，1 = eeprom 到 PDI */
   uint8 eep_pdi;
   /** CoE（CANopen over EtherCAT）详细信息 */
   uint8 CoEdetails;
   /** FoE（File over EtherCAT）详细信息 */
   uint8 FoEdetails;
   /** EoE（Ethernet over EtherCAT）详细信息 */
   uint8 EoEdetails;
   /** SoE（Servo over EtherCAT）详细信息 */
   uint8 SoEdetails;
   /** E-bus 总线电流 */
   int16 Ebuscurrent;
   /** 如果 >0，则在过程数据中阻止使用 LRW */
   uint8 blockLRW;
   /** 组号 */
   uint8 group;
   /** 第一个未使用的 FMMU */
   uint8 FMMUunused;
   /** 布尔值，用于跟踪从站是否（不）响应，SOEM 库不使用/设置此值 */
   boolean islost;
   /** 注册的配置函数，从 PRE_OP 到 SAFE_OP 
           PO2SOconfig                  ← 变量名
       *PO2SOconfig                  ← 它是一个指针
      (*PO2SOconfig)(...)            ← 这个指针指向一个"可以被调用"的东西 = 函数
  int (*PO2SOconfig)(...)            ← 那个函数返回 int
  int (*PO2SOconfig)(ecx_contextt *, uint16);  ← 函数接收这两个参数
  如果没有（），就会变成声明一个返回INT*的函数
   */
   int (*PO2SOconfig)(ecx_contextt *context, uint16 slave);
   /** 邮箱处理程序状态：0 = 无处理程序，1 = 循环任务邮箱处理程序，2 = 从站丢失 */
   int mbxhandlerstate;
   /** 邮箱处理程序稳健邮箱协议状态 */
   int mbxrmpstate;
   /** 邮箱处理程序 RMP 扩展邮箱输入状态 */
   uint16 mbxinstateex;
   /** 指向缓冲区中 CoE 邮箱的指针 */
   uint8 *coembxin;
   /** CoE 邮箱输入标志，true = 邮箱满 */
   boolean coembxinfull;
   /** CoE 邮箱输入溢出计数器 */
   int coembxoverrun;
   /** 指向缓冲区中 SoE 邮箱的指针 */
   uint8 *soembxin;
   /** SoE 邮箱输入标志，true = 邮箱满 */
   boolean soembxinfull;
   /** SoE 邮箱输入溢出计数器 */
   int soembxoverrun;
   /** 指向缓冲区中 FoE 邮箱的指针 */
   uint8 *foembxin;
   /** FoE 邮箱输入标志，true = 邮箱满 */
   boolean foembxinfull;
   /** FoE 邮箱输入溢出计数器 */
   int foembxoverrun;
   /** 指向缓冲区中 EoE 邮箱的指针 */
   uint8 *eoembxin;
   /** EoE 邮箱输入标志，true = 邮箱满 */
   boolean eoembxinfull;
   /** EoE 邮箱输入溢出计数器 */
   int eoembxoverrun;
   /** 指向缓冲区中 VoE 邮箱的指针 */
   uint8 *voembxin;
   /** VoE 邮箱输入标志，true = 邮箱满 */
   boolean voembxinfull;
   /** VoE 邮箱输入溢出计数器 */
   int voembxoverrun;
   /** 指向缓冲区中 AoE 邮箱的指针 */
   uint8 *aoembxin;
   /** AoE 邮箱输入标志，true = 邮箱满 */
   boolean aoembxinfull;
   /** AoE 邮箱输入溢出计数器 */
   int aoembxoverrun;
   /** 指向输出邮箱状态寄存器缓冲区的指针 */
   uint8 *mbxstatus;
   /** 可读的从站名称 */
   char name[EC_MAXNAME + 1];
} ec_slavet;

/** EtherCAT 从站组结构体
 * 用于组织和管理从站组的过程数据映射
 */
typedef struct ec_group
{
   /** 该组的逻辑起始地址 */
   uint32 logstartaddr;
   /** 输出字节数，如果 Obits < 8 则 Obytes = 0 */
   uint32 Obytes;
   /** 输出指针，指向 IOmap 缓冲区 */
   uint8 *outputs;
   /** 输入字节数，如果 Ibits < 8 则 Ibytes = 0 */
   uint32 Ibytes;
   /** 输入指针，指向 IOmap 缓冲区 */
   uint8 *inputs;
   /** 是否支持分布式时钟（DC） */
   boolean hasdc;
   /** 下一个 DC 从站 */
   uint16 DCnext;
   /** E-bus 总线电流 */
   int16 Ebuscurrent;
   /** 如果 >0，则在过程数据中阻止使用 LRW */
   uint8 blockLRW;
   /** 使用的 IO 段数量 */
   uint16 nsegments;
   /** 第一个输入段 */
   uint16 Isegment;
   /** 输入段中的偏移量 */
   uint16 Ioffset;
   /** 预期的输出工作计数器 */
   uint16 outputsWKC;
   /** 预期的输入工作计数器 */
   uint16 inputsWKC;
   /** 检查从站状态 */
   boolean docheckstate;
   /** IO 分段列表。数据报不得将 SM 分成两部分 */
   uint32 IOsegment[EC_MAXIOSEGMENTS];
   /** 指向输出邮箱状态寄存器缓冲区的指针 */
   uint8 *mbxstatus;
   /** 邮箱状态寄存器缓冲区长度 */
   int32 mbxstatuslength;
   /** 邮箱状态查找表 */
   uint16 mbxstatuslookup[EC_MAXSLAVE];
   /** 邮箱处理程序中最后处理的邮箱 */
   uint16 lastmbxpos;
   /** 邮箱发送队列结构体 */
   ec_mbxqueuet mbxtxqueue;
} ec_groupt;

/** ESM 状态转换：Init 到 Pre-Op */
#define ECT_ESMTRANS_IP 0x0001
/** ESM 状态转换：Pre-Op 到 Safe-Op */
#define ECT_ESMTRANS_PS 0x0002
/** ESM 状态转换：Pre-Op 到 Init */
#define ECT_ESMTRANS_PI 0x0004
/** ESM 状态转换：Safe-Op 到 Pre-Op */
#define ECT_ESMTRANS_SP 0x0008
/** ESM 状态转换：Safe-Op 到 Op */
#define ECT_ESMTRANS_SO 0x0010
/** ESM 状态转换：Safe-Op 到 Init */
#define ECT_ESMTRANS_SI 0x0020
/** ESM 状态转换：Op 到 Safe-Op */
#define ECT_ESMTRANS_OS 0x0040
/** ESM 状态转换：Op 到 Pre-Op */
#define ECT_ESMTRANS_OP 0x0080
/** ESM 状态转换：Op 到 Init */
#define ECT_ESMTRANS_OI 0x0100
/** ESM 状态转换：Init 到 Boot */
#define ECT_ESMTRANS_IB 0x0200
/** ESM 状态转换：Boot 到 Init */
#define ECT_ESMTRANS_BI 0x0400
/** ESM 状态转换：Init 到 Init */
#define ECT_ESMTRANS_II 0x0800
/** ESM 状态转换：Pre-Op 到 Pre-Op */
#define ECT_ESMTRANS_PP 0x1000
/** ESM 状态转换：Safe-Op 到 Safe-Op */
#define ECT_ESMTRANS_SS 0x2000

/** ENI（EtherCAT Network Information）CoE 命令结构体
 * 用于在状态转换期间自动发送 CoE 命令
 */
typedef struct ec_enicoecmd
{
   /** 发送命令时的状态转换 */
   uint16 Transition;
   /** 完全访问标志 */
   boolean CA;
   /** 客户端命令规范（1 = 读取，2 = 写入） */
   uint8 Ccs;
   /** 对象索引 */
   uint16 Index;
   /** 对象子索引 */
   uint8 SubIdx;
   /** 超时时间（微秒） */
   int Timeout;
   /** 参数缓冲区大小（字节） */
   int DataSize;
   /** 指向参数缓冲区的指针 */
   void *Data;
} ec_enicoecmdt;

/** ENI 从站结构体
 * 用于存储 ENI 文件中的从站配置信息
 */
typedef struct ec_enislave
{
   /** 从站编号 */
   uint16 Slave;
   /** 制造商 ID */
   uint32 VendorId;
   /** 产品代码 */
   uint32 ProductCode;
   /** 版本号 */
   uint32 RevisionNo;
   /** CoE 命令数组指针 */
   ec_enicoecmdt *CoECmds;
   /** CoE 命令数量 */
   int CoECmdCount;
} ec_enislavet;

/** ENI 结构体
 * 用于存储 EtherCAT 网络信息
 */
typedef struct ec_eni
{
   /** 从站数组指针 */
   ec_enislavet *slave;
   /** 从站数量 */
   int slavecount;
} ec_enit;

/** SII（Slave Information Interface）FMMU 结构体
 * 用于从 EEPROM 读取 FMMU 配置
 */
typedef struct ec_eepromFMMU
{
   /** 起始位置 */
   uint16 Startpos;
   /** FMMU 数量 */
   uint8 nFMMU;
   /** FMMU0 功能 */
   uint8 FMMU0;
   /** FMMU1 功能 */
   uint8 FMMU1;
   /** FMMU2 功能 */
   uint8 FMMU2;
   /** FMMU3 功能 */
   uint8 FMMU3;
} ec_eepromFMMUt;

/** SII（Slave Information Interface）SM 结构体
 * 用于从 EEPROM 读取同步管理器配置
 */
typedef struct ec_eepromSM
{
   /** 起始位置 */
   uint16 Startpos;
   /** SM 数量 */
   uint8 nSM;
   /** 物理起始地址 */
   uint16 PhStart;
   /** 物理长度 */
   uint16 Plength;
   /** 控制寄存器 */
   uint8 Creg;
   /** 状态寄存器（不关心） */
   uint8 Sreg;
   /** 激活标志 */
   uint8 Activate;
   /** PDI 控制（不关心） */
   uint8 PDIctrl;
} ec_eepromSMt;

/** EEPROM PDO 记录结构体
 * 用于存储从 EEPROM 读取的 RX/TX PDO 表
 */
typedef struct ec_eepromPDO
{
   /** 起始位置 */
   uint16 Startpos;
   /** 长度 */
   uint16 Length;
   /** PDO 数量 */
   uint16 nPDO;
   /** PDO 索引数组 */
   uint16 Index[EC_MAXEEPDO];
   /** 同步管理器数组 */
   uint16 SyncM[EC_MAXEEPDO];
   /** 位大小数组 */
   uint16 BitSize[EC_MAXEEPDO];
   /** SM 位大小数组 */
   uint16 SMbitsize[EC_MAXSM];
} ec_eepromPDOt;

/** 标准 EtherCAT 邮箱头结构体
 * 用于邮箱通信的头部信息
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_mbxheader
{
   /** 长度 */
   uint16 length;
   /** 地址 */
   uint16 address;
   /** 优先级 */
   uint8 priority;
   /** 邮箱类型 */
   uint8 mbxtype;
} ec_mbxheadert;
OSAL_PACKED_END

/** AL 状态和 AL 状态码结构体
 * 用于存储从站的 AL 状态信息
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_alstatus
{
   /** AL 状态 */
   uint16 alstatus;
   /** 未使用 */
   uint16 unused;
   /** AL 状态码 */
   uint16 alstatuscode;
} ec_alstatust;
OSAL_PACKED_END

/** 栈结构体，用于存储分段 LRD/LWR/LRW 构造
 * 用于处理分段的过程数据读写操作
 */
typedef struct ec_idxstack
{
   /** 已推送计数 */
   uint8 pushed;
   /** 已拉取计数 */
   uint8 pulled;
   /** 索引数组 */
   uint8 idx[EC_MAXBUF];
   /** 数据指针数组 */
   void *data[EC_MAXBUF];
   /** 长度数组 */
   uint16 length[EC_MAXBUF];
   /** DC 偏移数组 */
   uint16 dcoffset[EC_MAXBUF];
   /** 类型数组 */
   uint8 type[EC_MAXBUF];
} ec_idxstackT;

/** 错误存储环形缓冲区结构体
 * 用于存储 EtherCAT 错误信息
 */
typedef struct ec_ering
{
   /** 头指针 */
   int16 head;
   /** 尾指针 */
   int16 tail;
   /** 错误数组 */
   ec_errort Error[EC_MAXELIST + 1];
} ec_eringt;

/** 同步管理器通信类型结构体（用于完全访问）
 * 用于存储 SM 的通信类型信息
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_SMcommtype
{
   /** 数量 */
   uint8 n;
   /** 未使用 */
   uint8 nu1;
   /** SM 类型数组 */
   uint8 SMtype[EC_MAXSM];
} ec_SMcommtypet;
OSAL_PACKED_END

/** SDO 分配结构体（用于完全访问）
 * 用于存储 PDO 分配信息
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_PDOassign
{
   /** 数量 */
   uint8 n;
   /** 未使用 */
   uint8 nu1;
   /** 索引数组 */
   uint16 index[256];
} ec_PDOassignt;
OSAL_PACKED_END

/** SDO 描述结构体（用于完全访问）
 * 用于存储 PDO 描述信息
 */
OSAL_PACKED_BEGIN
typedef struct OSAL_PACKED ec_PDOdesc
{
   /** 数量 */
   uint8 n;
   /** 未使用 */
   uint8 nu1;
   /** PDO 数组 */
   uint32 PDO[256];
} ec_PDOdesct;
OSAL_PACKED_END

/** 
 * EtherCAT 上下文结构体
 * 被所有的 ecx 函数引用，包含 EtherCAT 主站的所有状态信息
 */
struct ecx_context
{
   /** @publicsection */
   /* 网络状态 */

   /** 端口信息，可能包含冗余端口（red_port） */
   ecx_portt port;
   /** 检测到的从站列表，最多 EC_MAXSLAVE 个从站 */
   ec_slavet slavelist[EC_MAXSLAVE];
   /** 配置中找到的从站数量 */
   int slavecount;
   /** 从站组列表，最多 EC_MAXGROUP 个组 */
   ec_groupt grouplist[EC_MAXGROUP];
   /** EtherCAT 错误状态标志 */
   boolean ecaterror;
   /** 从站的最后一次分布式时钟（DC）时间 */
   int64 DCtime;

   /** @privatesection */
   /* 内部状态 */

   /** 内部使用，EEPROM 缓存缓冲区 */
   uint8 esibuf[EC_MAXEEPBUF];
   /** 内部使用，EEPROM 缓存映射位图 */
   uint32 esimap[EC_MAXEEPBITMAP];
   /** 内部使用，当前 EEPROM 缓存对应的从站号 */
   uint16 esislave;
   /** 内部使用，错误列表 */
   ec_eringt elist;
   /** 内部使用，过程数据栈缓冲区信息 */
   ec_idxstackT idxstack;
   /** 内部使用，同步管理器（SM）通信类型缓冲区 */
   ec_SMcommtypet SMcommtype[EC_MAX_MAPT];
   /** 内部使用，PDO 分配列表 */
   ec_PDOassignt PDOassign[EC_MAX_MAPT];
   /** 内部使用，PDO 描述列表 */
   ec_PDOdesct PDOdesc[EC_MAX_MAPT];
   /** 内部使用，从 EEPROM 读取的 SM 列表 */
   ec_eepromSMt eepSM;
   /** 内部使用，从 EEPROM 读取的 FMMU 列表 */
   ec_eepromFMMUt eepFMMU;
   /** 内部使用，邮箱池 */
   ec_mbxpoolt mbxpool;

   /** @publicsection */
   /* 可配置设置 */

   /** 网络信息钩子，指向 ENI（EtherCAT Network Information）结构 */
   ec_enit *ENI;
   /** 注册的 FoE（File over EtherCAT）钩子函数 */
   int (*FOEhook)(uint16 slave, int packetnumber, int datasize);
   /** 注册的 EoE（Ethernet over EtherCAT）钩子函数 */
   int (*EOEhook)(ecx_contextt *context, uint16 slave, void *eoembx);
   /** 标志位，控制使用传统的自动状态改变还是手动状态改变 */
   int manualstatechange;
   /** 不透明指针，指向应用程序的用户数据，SOEM 不会使用此指针 */
   void *userdata;
   /** 重叠模式标志：在此模式下，输入数据将在接收帧中替换输出数据
    * 此模式用于 TI ESC（Texas Instruments EtherCAT Slave Controller）
    * 过程数据始终按字节边界对齐 */
   boolean overlappedMode;
   /** 紧凑模式标志：不将每个从站映射到字节边界
    * 这可能导致更小的帧大小。在重叠模式下无效 */
   boolean packedMode;
};

/** 查找可用的网络适配器
 * @return 适配器链表指针 */
ec_adaptert *ec_find_adapters(void);
/** 释放适配器链表
 * @param adapter 要释放的适配器链表指针 */
void ec_free_adapters(ec_adaptert *adapter);
/** 获取下一个邮箱计数器值
 * @param cnt 当前计数器值
 * @return 下一个计数器值 */
uint8 ec_nextmbxcnt(uint8 cnt);
/** 清除邮箱缓冲区
 * @param Mbx 邮箱缓冲区指针 */
void ec_clearmbx(ec_mbxbuft *Mbx);
/** 将错误推入错误环形缓冲区
 * @param context EtherCAT 上下文指针
 * @param Ec 错误结构体指针 */
void ecx_pusherror(ecx_contextt *context, const ec_errort *Ec);
/** 从错误环形缓冲区弹出错误
 * @param context EtherCAT 上下文指针
 * @param Ec 错误结构体指针
 * @return TRUE 如果成功弹出，FALSE 如果缓冲区为空 */
boolean ecx_poperror(ecx_contextt *context, ec_errort *Ec);
/** 检查是否有错误
 * @param context EtherCAT 上下文指针
 * @return TRUE 如果有错误，FALSE 如果没有错误 */
boolean ecx_iserror(ecx_contextt *context);
/** 报告数据包错误
 * @param context EtherCAT 上下文指针
 * @param Slave 从站编号
 * @param Index 对象索引
 * @param SubIdx 对象子索引
 * @param ErrorCode 错误码 */
void ecx_packeterror(ecx_contextt *context, uint16 Slave, uint16 Index, uint8 SubIdx, uint16 ErrorCode);
/** 初始化 EtherCAT 主站
 * @param context EtherCAT 上下文指针
 * @param ifname 网络接口名称
 * @return 工作计数器，>0 表示成功 */
int ecx_init(ecx_contextt *context, const char *ifname);
/** 使用冗余模式初始化 EtherCAT 主站
 * @param context EtherCAT 上下文指针
 * @param redport 冗余端口结构体指针
 * @param ifname 主网络接口名称
 * @param if2name 备用网络接口名称
 * @return 工作计数器，>0 表示成功 */
int ecx_init_redundant(ecx_contextt *context, ecx_redportt *redport, const char *ifname, char *if2name);
/** 关闭 EtherCAT 主站
 * @param context EtherCAT 上下文指针 */
void ecx_close(ecx_contextt *context);
/** 从 SII 缓冲区获取一个字节
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param address SII 地址
 * @return 读取的字节 */
uint8 ecx_siigetbyte(ecx_contextt *context, uint16 slave, uint16 address);
/** 在 SII 中查找类别
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param cat 类别
 * @return 找到的位置，<0 表示未找到 */
int16 ecx_siifind(ecx_contextt *context, uint16 slave, uint16 cat);
/** 从 SII 读取字符串
 * @param context EtherCAT 上下文指针
 * @param str 字符串缓冲区
 * @param slave 从站编号
 * @param Sn 字符串编号 */
void ecx_siistring(ecx_contextt *context, char *str, uint16 slave, uint16 Sn);
/** 从 SII 读取 FMMU 配置
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param FMMU FMMU 结构体指针
 * @return 读取的字节数 */
uint16 ecx_siiFMMU(ecx_contextt *context, uint16 slave, ec_eepromFMMUt *FMMU);
/** 从 SII 读取 SM 配置
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param SM SM 结构体指针
 * @return 读取的字节数 */
uint16 ecx_siiSM(ecx_contextt *context, uint16 slave, ec_eepromSMt *SM);
/** 从 SII 读取下一个 SM 配置
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param SM SM 结构体指针
 * @param n SM 编号
 * @return TRUE 如果成功，FALSE 如果失败 */
uint16 ecx_siiSMnext(ecx_contextt *context, uint16 slave, ec_eepromSMt *SM, uint16 n);
/** 从 SII 读取 PDO 配置
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param PDO PDO 结构体指针
 * @param t PDO 类型（0=输入，1=输出）
 * @return PDO 总位数 */
uint32 ecx_siiPDO(ecx_contextt *context, uint16 slave, ec_eepromPDOt *PDO, uint8 t);
/** 读取所有从站的状态
 * @param context EtherCAT 上下文指针
 * @return 工作计数器 */
int ecx_readstate(ecx_contextt *context);
/** 写入从站的状态
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @return 工作计数器 */
int ecx_writestate(ecx_contextt *context, uint16 slave);
/** 检查从站状态
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param reqstate 请求的状态
 * @param timeout 超时时间（微秒）
 * @return 请求的状态，或超时时的当前状态 */
uint16 ecx_statecheck(ecx_contextt *context, uint16 slave, uint16 reqstate, int timeout);
/** 邮箱处理程序
 * @param context EtherCAT 上下文指针
 * @param group 组号
 * @param limit 处理限制
 * @return 处理的邮箱数量 */
int ecx_mbxhandler(ecx_contextt *context, uint8 group, int limit);
/** 等待邮箱为空
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param timeout 超时时间（微秒）
 * @return >0 表示成功，0 表示超时 */
int ecx_mbxempty(ecx_contextt *context, uint16 slave, int timeout);
/** 发送邮箱数据
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param mbx 邮箱缓冲区指针
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_mbxsend(ecx_contextt *context, uint16 slave, ec_mbxbuft *mbx, int timeout);
/** 接收邮箱数据
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param mbx 邮箱缓冲区指针的指针
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_mbxreceive(ecx_contextt *context, uint16 slave, ec_mbxbuft **mbx, int timeout);
/** 执行 ENI 初始化命令
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param transition 状态转换
 * @return 工作计数器 */
int ecx_mbxENIinitcmds(ecx_contextt *context, uint16 slave, uint16_t transition);
/** 转储 EEPROM 内容
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param esibuf EEPROM 缓冲区指针 */
void ecx_esidump(ecx_contextt *context, uint16 slave, uint8 *esibuf);
/** 从 EEPROM 读取数据
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param eeproma EEPROM 地址
 * @param timeout 超时时间（微秒）
 * @return 读取的数据 */
uint32 ecx_readeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, int timeout);
/** 向 EEPROM 写入数据
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param eeproma EEPROM 地址
 * @param data 要写入的数据
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_writeeeprom(ecx_contextt *context, uint16 slave, uint16 eeproma, uint16 data, int timeout);
/** 将 EEPROM 控制权转移给主站
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @return 工作计数器 */
int ecx_eeprom2master(ecx_contextt *context, uint16 slave);
/** 将 EEPROM 控制权转移给 PDI
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @return 工作计数器 */
int ecx_eeprom2pdi(ecx_contextt *context, uint16 slave);
/** 使用自动增量地址从 EEPROM 读取数据
 * @param context EtherCAT 上下文指针
 * @param aiadr 自动增量地址
 * @param eeproma EEPROM 地址
 * @param timeout 超时时间（微秒）
 * @return 读取的数据 */
uint64 ecx_readeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, int timeout);
/** 使用自动增量地址向 EEPROM 写入数据
 * @param context EtherCAT 上下文指针
 * @param aiadr 自动增量地址
 * @param eeproma EEPROM 地址
 * @param data 要写入的数据
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_writeeepromAP(ecx_contextt *context, uint16 aiadr, uint16 eeproma, uint16 data, int timeout);
/** 使用配置地址从 EEPROM 读取数据
 * @param context EtherCAT 上下文指针
 * @param configadr 配置地址
 * @param eeproma EEPROM 地址
 * @param timeout 超时时间（微秒）
 * @return 读取的数据 */
uint64 ecx_readeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, int timeout);
/** 使用配置地址向 EEPROM 写入数据
 * @param context EtherCAT 上下文指针
 * @param configadr 配置地址
 * @param eeproma EEPROM 地址
 * @param data 要写入的数据
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_writeeepromFP(ecx_contextt *context, uint16 configadr, uint16 eeproma, uint16 data, int timeout);
/** EEPROM 读取的第一阶段（启动读取）
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param eeproma EEPROM 地址 */
void ecx_readeeprom1(ecx_contextt *context, uint16 slave, uint16 eeproma);
/** EEPROM 读取的第二阶段（完成读取）
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @param timeout 超时时间（微秒）
 * @return 读取的数据 */
uint32 ecx_readeeprom2(ecx_contextt *context, uint16 slave, int timeout);
/** 接收指定组的过程数据
 * @param context EtherCAT 上下文指针
 * @param group 组号
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_receive_processdata_group(ecx_contextt *context, uint8 group, int timeout);
/** 发送过程数据到所有从站
 * @param context EtherCAT 上下文指针
 * @return 工作计数器 */
int ecx_send_processdata(ecx_contextt *context);
/** 接收所有从站的过程数据
 * @param context EtherCAT 上下文指针
 * @param timeout 超时时间（微秒）
 * @return 工作计数器 */
int ecx_receive_processdata(ecx_contextt *context, int timeout);
/** 发送指定组的过程数据
 * @param context EtherCAT 上下文指针
 * @param group 组号
 * @return 工作计数器 */
int ecx_send_processdata_group(ecx_contextt *context, uint8 group);
/** 从邮箱池获取邮箱缓冲区
 * @param context EtherCAT 上下文指针
 * @return 邮箱缓冲区指针 */
ec_mbxbuft *ecx_getmbx(ecx_contextt *context);
/** 将邮箱缓冲区归还到邮箱池
 * @param context EtherCAT 上下文指针
 * @param mbx 邮箱缓冲区指针
 * @return 0 表示成功 */
int ecx_dropmbx(ecx_contextt *context, ec_mbxbuft *mbx);
/** 初始化邮箱池
 * @param context EtherCAT 上下文指针
 * @return 0 表示成功 */
int ecx_initmbxpool(ecx_contextt *context);
/** 初始化邮箱队列
 * @param context EtherCAT 上下文指针
 * @param group 组号
 * @return 0 表示成功 */
int ecx_initmbxqueue(ecx_contextt *context, uint8 group);
/** 循环处理从站邮箱
 * @param context EtherCAT 上下文指针
 * @param slave 从站编号
 * @return 处理的邮箱数量 */
int ecx_slavembxcyclic(ecx_contextt *context, uint16 slave);

#ifdef __cplusplus
}
#endif

#endif
