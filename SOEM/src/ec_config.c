/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

/**
 * \file
 * \brief EtherCAT 主站配置模块
 *
 * 本文件包含 EtherCAT 主站的配置功能，用于在成功初始化（通过 ec_init() 或 ec_init_redundant()）
 * 之后自动配置从站。
 *
 * 主要功能：
 * 1. 从站枚举和初始化
 *    - 扫描并检测网络上的所有 EtherCAT 从站
 *    - 读取从站的 EEPROM 信息（制造商 ID、产品代码、序列号等）
 *    - 为每个从站分配配置地址
 *
 * 2. PDO 映射配置
 *    - 将从站的 PDO（Process Data Object）映射到 I/O 内存区域
 *    - 支持默认布局和自定义布局
 *    - 支持重叠模式和紧凑模式
 *    - 处理分段传输
 *
 * 3. 分布式时钟（DC）配置
 *    - 配置从站的分布式时钟同步
 *    - 设置参考时钟
 *    - 计算传播延迟
 *
 * 4. 状态管理
 *    - 监控从站状态变化
 *    - 恢复丢失的从站
 *    - 重新配置异常从站
 *
 * 5. 邮箱通信配置
 *    - 配置从站的邮箱参数（读/写邮箱地址和大小）
 *    - 配置邮箱协议（CoE, FoE, EoE, SoE）
 *
 * 6. 同步管理器（SM）配置
 *    - 从 EEPROM 读取 SM 配置
 *    - 配置 SM 的类型和方向
 *
 * 7. FMMU（Fieldbus Memory Management Unit）配置
 *    - 从 EEPROM 读取 FMMU 配置
 *    - 配置 FMMU 的映射
 *
 * 配置流程：
 * 1. ecx_config_init() - 枚举并初始化所有从站
 * 2. ecx_config_map_group() - 将 PDO 映射到 I/O 内存
 * 3. ecx_configdc() - 配置分布式时钟
 * 4. 状态转换：INIT -> PRE_OP -> SAFE_OP -> OPERATIONAL
 *
 * 注意事项：
 * - 必须在 ec_init() 或 ec_init_redundant() 成功后调用
 * - 配置过程中会读取从站的 EEPROM，需要一定的超时时间
 * - 配置完成后，从站才能进入 OPERATIONAL 状态进行过程数据交换
 */

#include "soem/soem.h"
#include <stdio.h>
#include <string.h>
#include "osal.h"
#include "oshw.h"

typedef struct
{
   int thread_n;
   int running;
   ecx_contextt *context;
   uint16 slave;
} ecx_mapt_t;

ecx_mapt_t ecx_mapt[EC_MAX_MAPT];
#if EC_MAX_MAPT > 1
OSAL_THREAD_HANDLE ecx_threadh[EC_MAX_MAPT];
#endif

/** standard SM0 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM0 0x00010026
/** standard SM1 flags configuration for mailbox slaves */
#define EC_DEFAULTMBXSM1 0x00010022
/** standard SM0 flags configuration for digital output slaves */
#define EC_DEFAULTDOSM0  0x00010044

void ecx_init_context(ecx_contextt *context)
{
   int lp;
   context->slavecount = 0;
   /* clean ec_slave array */
   memset(context->slavelist, 0x00, sizeof(context->slavelist));
   /* TODO: 默认memset将slavelist[slave].group清零，所有从站属于group 0。
    * 如需手动分组，应在ec_config_init()之后、ec_config_map()之前，
    * 为每个从站设置 ec_slave[slave].group = 目标组号(1,2,...)。
    * 不同组的IOmap可分开映射：ec_config_map_group(&IOmap1, 1); ec_config_map_group(&IOmap2, 2);
    */
   memset(context->grouplist, 0x00, sizeof(context->grouplist));
   /* clear slave eeprom cache, does not actually read any eeprom */
   ecx_siigetbyte(context, 0, EC_MAXEEPBUF);
   for (lp = 0; lp < EC_MAXGROUP; lp++)
   {
      /* default start address per group entry */
      context->grouplist[lp].logstartaddr = lp << EC_LOGGROUPOFFSET;
      ecx_initmbxqueue(context, lp);
   }
}

int ecx_detect_slaves(ecx_contextt *context)
{
   uint8 b;
   uint16 w;
   int wkc;

   /* make special pre-init register writes to enable MAC[1] local administered bit *
    * setting for old netX100 slaves */
   b = 0x00;
   ecx_BWR(&context->port, 0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET3); /* Ignore Alias register */
   w = htoes(EC_STATE_INIT | EC_STATE_ACK);
   ecx_BWR(&context->port, 0x0000, ECT_REG_ALCTL, sizeof(w), &w, EC_TIMEOUTRET3); /* Reset all slaves to Init */
   /* netX100 should now be happy */
   ecx_BWR(&context->port, 0x0000, ECT_REG_ALCTL, sizeof(w), &w, EC_TIMEOUTRET3);      /* Reset all slaves to Init */
   wkc = ecx_BRD(&context->port, 0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE); /* detect number of slaves */
   if (wkc > 0)
   {
      /* this is strictly "less than" since the master is "slave 0" */
      if (wkc < EC_MAXSLAVE)
      {
         context->slavecount = wkc;
      }
      else
      {
         EC_PRINT("Error: too many slaves on network: num_slaves=%d, max_slaves=%d\n",
                  wkc, EC_MAXSLAVE);
         return EC_SLAVECOUNTEXCEEDED;
      }
   }
   return wkc;
}

/**
 * 将所有从站重置为默认配置状态
 *
 * 功能：
 * - 在配置初始化前将所有从站恢复到已知的默认状态
 * - 清除之前的配置（FMMU、Sync Manager、DC 等）
 * - 重置所有从站到 INIT 状态
 * - 配置 EEPROM 访问权限
 *
 * 执行流程：
 * 1. 禁用手动环回模式
 * 2. 设置中断掩码
 * 3. 重置 CRC 错误计数器
 * 4. 清除 FMMU（现场总线内存管理单元）配置
 * 5. 清除 Sync Manager（同步管理器）配置
 * 6. 重置分布式时钟（DC）相关寄存器
 * 7. 忽略别名寄存器
 * 8. 将所有从站重置到 INIT 状态
 * 9. 配置 EEPROM 访问权（先给 PDI，再还给主站）
 *
 * @param[in] context 上下文结构体指针，包含端口和从站信息
 */
static void ecx_set_slaves_to_default(ecx_contextt *context)
{
   uint8 b;           /* 通用字节变量，用于单字节寄存器操作 */
   uint16 w;          /* 通用16位变量，用于16位寄存器操作 */
   uint8 zbuf[64];    /* 零缓冲区，用于清零多个寄存器 */

   /* 初始化零缓冲区，所有字节置为0 */
   memset(&zbuf, 0x00, sizeof(zbuf));

   /* 1. 禁用手动环回模式
    * DLPORT（Data Link Port Control Register）：数据链路端口控制寄存器
    * 写入 0x00 禁用手动环回，使端口进入正常自动模式 */
   b = 0x00;
   ecx_BWR(&context->port, 0x0000, ECT_REG_DLPORT, sizeof(b), &b, EC_TIMEOUTRET3);

   /* 2. 设置中断掩码
    * IRQMASK（Interrupt Mask Register）：中断屏蔽寄存器
    * 0x0004：启用链路变化中断，当端口连接状态改变时产生中断 */
   w = htoes(0x0004);
   ecx_BWR(&context->port, 0x0000, ECT_REG_IRQMASK, sizeof(w), &w, EC_TIMEOUTRET3);

   /* 3. 重置 CRC 错误计数器
    * RXERR（Receive Error Counters Register）：接收错误计数器寄存器
    * 写入 8 字节的零，清除所有端口的 CRC 错误计数
    * 这可以清除之前通信累积的错误统计 */
   ecx_BWR(&context->port, 0x0000, ECT_REG_RXERR, 8, &zbuf, EC_TIMEOUTRET3);

   /* 4. 重置 FMMU（现场总线内存管理单元）
    * FMMU0（Fieldbus Memory Management Unit Register）：FMMU 配置寄存器
    * 每个从站最多有 3 个 FMMU，每个 FMMU 占用 16 字节
    * 写入 48 字节（16 * 3）的零，清除所有 FMMU 配置
    * FMMU 用于将主站内存映射到从站内存，清除后需要重新配置 */
   ecx_BWR(&context->port, 0x0000, ECT_REG_FMMU0, 16 * 3, &zbuf, EC_TIMEOUTRET3);

   /* 5. 重置 Sync Manager（同步管理器）
    * SM0（Sync Manager Register）：同步管理器配置寄存器
    * 每个从站最多有 4 个 Sync Manager，每个 SM 占用 8 字节
    * 写入 32 字节（8 * 4）的零，清除所有 SM 配置
    * SM 用于管理邮箱和 PDO 数据传输，清除后需要重新配置 */
   ecx_BWR(&context->port, 0x0000, ECT_REG_SM0, 8 * 4, &zbuf, EC_TIMEOUTRET3);

   /* 6. 重置分布式时钟（DC）激活寄存器
    * DCSYNCACT（DC Sync Activation Register）：DC 同步激活寄存器
    * 写入 0x00 禁用 DC 同步功能 */
   b = 0x00;
   ecx_BWR(&context->port, 0x0000, ECT_REG_DCSYNCACT, sizeof(b), &b, EC_TIMEOUTRET3);

   /* 7. 重置分布式时钟系统时间和偏移量
    * DCSYSTIME（DC System Time Register）：DC 系统时间寄存器
    * 写入 4 字节的零，清除系统时间和偏移量
    * 这将清除之前的 DC 时间同步设置 */
   ecx_BWR(&context->port, 0x0000, ECT_REG_DCSYSTIME, 4, &zbuf, EC_TIMEOUTRET3);

   /* 8. 设置 DC 速度计数器起始值
    * DCSPEEDCNT（DC Speed Counter Start Register）：DC 速度计数器起始寄存器
    * 0x1000：设置速度计数器的起始值
    * 用于分布式时钟的速度监控功能 */
   w = htoes(0x1000);
   ecx_BWR(&context->port, 0x0000, ECT_REG_DCSPEEDCNT, sizeof(w), &w, EC_TIMEOUTRET3);

   /* 9. 设置 DC 时间滤波器指数
    * DCTIMEFILT（DC Time Filter Register）：DC 时间滤波器寄存器
    * 0x0c00：设置滤波器指数，用于平滑分布式时钟的时间测量
    * 滤波器可以减少时间测量的抖动 */
   w = htoes(0x0c00);
   ecx_BWR(&context->port, 0x0000, ECT_REG_DCTIMEFILT, sizeof(w), &w, EC_TIMEOUTRET3);

   /* 10. 忽略别名寄存器
    * DLALIAS（Data Link Alias Register）：数据链路别名寄存器
    * 写入 0x00 禁用别名功能
    * 别名是用户可配置的替代地址，用于热插拔等场景
    * 在初始化阶段忽略别名，使用默认地址 */
   b = 0x00;
   ecx_BWR(&context->port, 0x0000, ECT_REG_DLALIAS, sizeof(b), &b, EC_TIMEOUTRET3);

   /* 11. 将所有从站重置到 INIT 状态
    * ALCTL（AL Control Register）：应用层控制寄存器
    * EC_STATE_INIT | EC_STATE_ACK：请求 INIT 状态并设置确认位
    * 这将所有从站的状态机重置到初始状态
    * INIT 状态是配置的起点，不允许任何数据传输 */
   w = htoes(EC_STATE_INIT | EC_STATE_ACK);
   ecx_BWR(&context->port, 0x0000, ECT_REG_ALCTL, sizeof(w), &w, EC_TIMEOUTRET3);

   /* 12. 强制 EEPROM 访问权交给 PDI（Process Data Interface）
    * EEPCFG（EEPROM Configuration Register）：EEPROM 配置寄存器
    * 写入 0x02：将 EEPROM 访问权授予 PDI
    * PDI 是从站的数据接口，某些从站在状态转换期间需要 PDI 访问 EEPROM
    * 这是为了兼容某些特殊的从站硬件 */
   b = 2;
   ecx_BWR(&context->port, 0x0000, ECT_REG_EEPCFG, sizeof(b), &b, EC_TIMEOUTRET3);

   /* 13. 将 EEPROM 访问权还给主站
    * EEPCFG（EEPROM Configuration Register）：EEPROM 配置寄存器
    * 写入 0x00：将 EEPROM 访问权还给主站
    * 主站需要访问 EEPROM 来读取从站的配置信息（制造商 ID、产品代码等）
    * 先给 PDI 再还给主站，是为了确保从站硬件处于正确的状态 */
   b = 0;
   ecx_BWR(&context->port, 0x0000, ECT_REG_EEPCFG, sizeof(b), &b, EC_TIMEOUTRET3);
}

/* If slave has SII and same slave ID done before, use previous data.
 * This is safe because SII is constant for same slave ID.
 */
static int ecx_lookup_prev_sii(ecx_contextt *context, uint16 slave)
{
   int i, nSM;
   if ((slave > 1) && (context->slavecount > 0))
   {
      i = 1;
      while (((context->slavelist[i].eep_man != context->slavelist[slave].eep_man) ||
              (context->slavelist[i].eep_id != context->slavelist[slave].eep_id) ||
              (context->slavelist[i].eep_rev != context->slavelist[slave].eep_rev)) &&
             (i < slave))
      {
         i++;
      }
      if (i < slave)
      {
         context->slavelist[slave].CoEdetails = context->slavelist[i].CoEdetails;
         context->slavelist[slave].FoEdetails = context->slavelist[i].FoEdetails;
         context->slavelist[slave].EoEdetails = context->slavelist[i].EoEdetails;
         context->slavelist[slave].SoEdetails = context->slavelist[i].SoEdetails;
         if (context->slavelist[i].blockLRW > 0)
         {
            context->slavelist[slave].blockLRW = 1;
            context->slavelist[0].blockLRW++;
         }
         context->slavelist[slave].Ebuscurrent = context->slavelist[i].Ebuscurrent;
         context->slavelist[0].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
         memcpy(context->slavelist[slave].name, context->slavelist[i].name, EC_MAXNAME + 1);
         for (nSM = 0; nSM < EC_MAXSM; nSM++)
         {
            context->slavelist[slave].SM[nSM].StartAddr = context->slavelist[i].SM[nSM].StartAddr;
            context->slavelist[slave].SM[nSM].SMlength = context->slavelist[i].SM[nSM].SMlength;
            context->slavelist[slave].SM[nSM].SMflags = context->slavelist[i].SM[nSM].SMflags;
         }
         context->slavelist[slave].FMMU0func = context->slavelist[i].FMMU0func;
         context->slavelist[slave].FMMU1func = context->slavelist[i].FMMU1func;
         context->slavelist[slave].FMMU2func = context->slavelist[i].FMMU2func;
         context->slavelist[slave].FMMU3func = context->slavelist[i].FMMU3func;
         EC_PRINT("Copy SII slave %d from %d.\n", slave, i);
         return 1;
      }
   }
   return 0;
}

/** 
 * 枚举并初始化所有从站
 * 功能：
 * - 扫描并检测网络上的所有 EtherCAT 从站
 * - 为每个从站分配配置地址
 * - 读取每个从站的 EEPROM 信息（制造商 ID、产品代码、序列号等）
 * - 配置从站的邮箱参数
 * - 初始化从站的同步管理器（SM）
 * 
 * 执行流程：
 * 1. 初始化上下文
 * 2. 检测从站数量
 * 3. 设置从站默认参数
 * 4. 为每个从站分配节点地址
 * 5. 读取 EEPROM 信息（制造商、ID、版本、序列号等）
 * 6. 配置邮箱参数
 * 7. 配置同步管理器
 * 8. 创建互斥锁
 * 
 * @param[in] context 上下文结构体指针
 * @return 从站发现数据报的工作计数器 = 找到的从站数量
 */
int ecx_config_init(ecx_contextt *context)
{
   /* 变量声明 */
   uint16 slave;          /* 从站编号循环变量 */
   uint16 ADPh;           /* 自动递增物理地址，用于寻址未配置的从站 */
   uint16 configadr;      /* 配置地址，从站的唯一节点地址 */
   uint16 ssigen;         /* SII（Slave Information Interface）通用段偏移量 */
   uint16 topology;       /* 拓扑信息，描述从站端口连接状态 */
   uint16 estat;          /* EEPROM状态寄存器值 */
   int16 topoc;           /* 拓扑计数器，用于查找父节点 */
   int16 slavec;          /* 从站计数器，用于遍历查找父节点 */
   int16 aliasadr;        /* 别名地址 */
   uint8 b;               /* 通用字节变量，用于各种标志和配置 */
   uint8 h;               /* 端口计数器，记录活跃端口数量 */
   uint8 SMc;             /* 同步管理器计数器 */
   uint32 eedat;          /* EEPROM数据，32位 */
   int wkc;               /* 工作计数器，表示响应的从站数量 */
   int nSM;               /* 同步管理器数量 */
   uint16 val16;          /* 通用16位变量，用于读取寄存器值 */

   /* 打印调试信息，表示开始配置初始化 */
   EC_PRINT("ec_config_init\n");
   
   ecx_init_context(context);
   
   wkc = ecx_detect_slaves(context);
   
   /* 如果检测到从站，则继续配置 */
   if (wkc > 0)
   {
      ecx_set_slaves_to_default(context);
      /* 遍历每个从站进行配置 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 计算自动递增物理地址（ADPh）
          * ADPh = 1 - slave，例如：slave=1时ADPh=0，slave=2时ADPh=-1（0xFFFF）
          * 这种寻址方式用于在从站尚未配置节点地址时进行通信 */
         ADPh = (uint16)(1 - slave);
         
         /* 读取从站的接口类型
          * 通过APRD（自动递增物理地址读取）命令读取PDI控制寄存器
          * 接口类型标识从站的物理接口类型 */
         val16 = ecx_APRDw(&context->port, ADPh, ECT_REG_PDICTL, EC_TIMEOUTRET3);
         context->slavelist[slave].Itype = etohs(val16);
         /* 为从站设置节点地址（配置地址）
          * 使用节点偏移量（EC_NODEOFFSET）提高网络帧的可读性
          * 这不会影响可寻址从站的数量（地址会自动回绕）
          * 例如：slave=1时，地址=1+0x1000=0x1001 */
         ecx_APWRw(&context->port, ADPh, ECT_REG_STADR, htoes(slave + EC_NODEOFFSET), EC_TIMEOUTRET3);
         
         /* 配置非EtherCAT帧的处理行为
          * 对于第一个从站（slave==1）：丢弃非EtherCAT帧（b=1）
          * 对于后续从站：允许所有帧通过（b=0）
          * 这样可以防止非EtherCAT帧在网络中传播 */
         if (slave == 1)
         {
            b = 1; /* 第一个从站丢弃非ECAT帧 */
         }
         else
         {
            b = 0; /* 后续从站转发所有帧 */
         }
         ecx_APWRw(&context->port, ADPh, ECT_REG_DLCTL, htoes(b), EC_TIMEOUTRET3);
         /* 读取并保存从站的配置地址
          * 使用APRD读取刚设置的节点地址，确认配置成功 */
         configadr = ecx_APRDw(&context->port, ADPh, ECT_REG_STADR, EC_TIMEOUTRET3);
         configadr = etohs(configadr);
         context->slavelist[slave].configadr = configadr;
         
         /* 读取从站的别名地址
          * 别名地址是用户可配置的替代地址，用于热插拔等场景
          * 使用配置地址（FPRD）进行读取 */
         ecx_FPRD(&context->port, configadr, ECT_REG_ALIAS, sizeof(aliasadr), &aliasadr, EC_TIMEOUTRET3);
         context->slavelist[slave].aliasadr = etohs(aliasadr);
         
         /* 读取EEPROM状态寄存器
          * 检查EEPROM是否就绪以及支持的读取模式
          * 使用配置地址进行读取 */
         ecx_FPRD(&context->port, configadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET3);
         estat = etohs(estat);
         
         /* 检查从站是否支持8字节块读取
          * 如果EEPSTAT寄存器的R64位被置位，则支持8字节读取
          * 这可以提高EEPROM读取效率 */
         if (estat & EC_ESTAT_R64)
         {
            context->slavelist[slave].eep_8byte = 1;
         }
         
         /* 开始读取EEPROM中的制造商ID
          * 使用两阶段读取：
          * 1. ecx_readeeprom1() - 启动读取请求（异步）
          * 2. ecx_readeeprom2() - 获取读取结果
          * 这样可以提高多个从站的读取效率 */
         ecx_readeeprom1(context, slave, ECT_SII_MANUF);
      }
      /* 第二阶段：获取所有从站的制造商ID并启动读取产品ID */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 获取制造商ID的读取结果 */
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_man = etohl(eedat);
         
         /* 启动读取产品ID（Product ID） */
         ecx_readeeprom1(context, slave, ECT_SII_ID);
      }
      /* 第三阶段：获取所有从站的产品ID并启动读取版本号 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 获取产品ID的读取结果 */
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_id = etohl(eedat);
         
         /* 启动读取版本号（Revision Number） */
         ecx_readeeprom1(context, slave, ECT_SII_REV);
      }
      /* 第四阶段：获取所有从站的版本号并启动读取序列号 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 获取版本号的读取结果 */
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_rev = etohl(eedat);
         
         /* 启动读取序列号（Serial Number） */
         ecx_readeeprom1(context, slave, ECT_SII_SER);
      }
      /* 第五阶段：获取所有从站的序列号并启动读取写邮箱地址 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 获取序列号的读取结果 */
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].eep_ser = etohl(eedat);
         
         /* 启动读取写邮箱地址和大小
          * 写邮箱：主站向从站写入数据的邮箱 */
         ecx_readeeprom1(context, slave, ECT_SII_RXMBXADR);
      }
      /* 第六阶段：获取所有从站的写邮箱地址并启动读邮箱地址 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 获取写邮箱地址和大小
          * 低16位：写邮箱偏移地址（mbx_wo）
          * 高16位：写邮箱长度（mbx_l） */
         eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
         context->slavelist[slave].mbx_wo = (uint16)LO_WORD(etohl(eedat));
         context->slavelist[slave].mbx_l = (uint16)HI_WORD(etohl(eedat));
         
         /* 如果从站有邮箱（长度>0），则启动读取读邮箱地址
          * 读邮箱：从站向主站发送数据的邮箱 */
         if (context->slavelist[slave].mbx_l > 0)
         {
            ecx_readeeprom1(context, slave, ECT_SII_TXMBXADR);
         }
      }
      /* 第七阶段：获取所有从站的读邮箱地址并启动读取邮箱协议 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         /* 仅处理有邮箱的从站 */
         if (context->slavelist[slave].mbx_l > 0)
         {
            /* 获取读邮箱偏移地址和长度
             * 低16位：读邮箱偏移地址（mbx_ro）
             * 高16位：读邮箱长度（mbx_rl） */
            eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
            context->slavelist[slave].mbx_ro = (uint16)LO_WORD(etohl(eedat));
            context->slavelist[slave].mbx_rl = (uint16)HI_WORD(etohl(eedat));
            
            /* 如果读邮箱长度为0，则使用写邮箱长度作为默认值 */
            if (context->slavelist[slave].mbx_rl == 0)
            {
               context->slavelist[slave].mbx_rl = context->slavelist[slave].mbx_l;
            }
            
            /* 启动读取邮箱协议支持信息
             * 邮箱协议包括：CoE、FoE、EoE、SoE等 */
            ecx_readeeprom1(context, slave, ECT_SII_MBXPROTO);
         }
         
         /* 读取从站支持的功能
          * ESCSUP寄存器指示从站支持的特性
          * 位2（0x04）：表示是否支持分布式时钟（DC） */
         configadr = context->slavelist[slave].configadr;
         val16 = ecx_FPRDw(&context->port, configadr, ECT_REG_ESCSUP, EC_TIMEOUTRET3);
         
         /* 检查是否支持分布式时钟（DC） */
         if ((etohs(val16) & 0x04) > 0)
         {
            context->slavelist[slave].hasdc = TRUE;
         }
         else
         {
            context->slavelist[slave].hasdc = FALSE;
         }
         /* 读取数据链路状态寄存器以提取拓扑信息
          * DLSTAT寄存器包含各端口的连接状态
          * 拓扑信息用于确定从站在网络中的位置和连接关系 */
         topology = ecx_FPRDw(&context->port, configadr, ECT_REG_DLSTAT, EC_TIMEOUTRET3);
         topology = etohs(topology);
         
         /* 初始化端口计数器 */
         h = 0;  /* 活跃端口数量 */
         b = 0;  /* 活跃端口位掩码 */
         
         /* 检查端口0状态
          * 位[9:8] = 0x02 表示端口0打开且通信已建立 */
         if ((topology & 0x0300) == 0x0200)
         {
            h++;
            b |= 0x01;
         }
         /* 检查端口1状态
          * 位[11:10] = 0x08 表示端口1打开且通信已建立 */
         if ((topology & 0x0c00) == 0x0800)
         {
            h++;
            b |= 0x02;
         }
         /* 检查端口2状态
          * 位[13:12] = 0x20 表示端口2打开且通信已建立 */
         if ((topology & 0x3000) == 0x2000)
         {
            h++;
            b |= 0x04;
         }
         /* 检查端口3状态
          * 位[15:14] = 0x80 表示端口3打开且通信已建立 */
         if ((topology & 0xc000) == 0x8000)
         {
            h++;
            b |= 0x08;
         }
         /* 读取端口描述寄存器，获取物理端口类型
          * ptype标识端口的物理类型（如MII、EBUS等） */
         val16 = ecx_FPRDw(&context->port, configadr, ECT_REG_PORTDES, EC_TIMEOUTRET3);
         context->slavelist[slave].ptype = LO_BYTE(etohs(val16));
         
         /* 保存拓扑信息
          * topology: 活跃端口总数（0-4）
          * activeports: 活跃端口位掩码 */
         context->slavelist[slave].topology = h;
         context->slavelist[slave].activeports = b;
         /* 拓扑类型说明：
          * 0 = 无连接（不可能）
          * 1 = 1个连接，线路末端
          * 2 = 2个连接，一个在前一个在后
          * 3 = 3个连接，分支点
          * 4 = 4个连接，交叉点 */
         
         /* 查找父节点（上游从站）
          * 父节点是数据从主站到达当前从站必须经过的从站
          * 对于第一个从站，父节点是主站（parent=0） */
         context->slavelist[slave].parent = 0;
         
         /* 对于非第一个从站，向前遍历查找父节点 */
         if (slave > 1)
         {
            topoc = 0;      /* 拓扑计数器，用于追踪分支深度 */
            slavec = slave - 1;  /* 从当前从站的前一个从站开始查找 */
            
            do
            {
               /* 获取前一个从站的拓扑信息 */
               topology = context->slavelist[slavec].topology;
               
               /* 根据拓扑类型调整计数器 */
               if (topology == 1)
               {
                  topoc--; /* 找到端点，分支深度减1 */
               }
               if (topology == 3)
               {
                  topoc++; /* 找到分支点，分支深度加1 */
               }
               if (topology == 4)
               {
                  topoc += 2; /* 找到交叉点，分支深度加2 */
               }
               
               /* 判断是否找到父节点
                * 条件1：topoc >= 0 且 topology > 1（非端点）
                * 条件2：slavec == 1（到达第一个从站） */
               if (((topoc >= 0) && (topology > 1)) ||
                   (slavec == 1))
               {
                  context->slavelist[slave].parent = slavec;
                  slavec = 1;  /* 设置为1以退出循环 */
               }
               slavec--;  /* 继续向前查找 */
            } while (slavec > 0);
         }
         /* 检查从站状态是否为INIT状态
          * 确保从站已正确初始化 */
         (void)ecx_statecheck(context, slave, EC_STATE_INIT, EC_TIMEOUTSTATE);

         /* 如果从站有邮箱，则设置默认邮箱配置
          * 同步管理器（SM）用于管理邮箱和PDO数据传输
          * SM0: 写邮箱（主站->从站）
          * SM1: 读邮箱（从站->主站）
          * SM2: 输出PDO（主站->从站）
          * SM3: 输入PDO（从站->主站） */
         if (context->slavelist[slave].mbx_l > 0)
         {
            /* 设置同步管理器类型
             * 1 = 写邮箱
             * 2 = 读邮箱
             * 3 = 输出PDO
             * 4 = 输入PDO */
            context->slavelist[slave].SMtype[0] = 1;
            context->slavelist[slave].SMtype[1] = 2;
            context->slavelist[slave].SMtype[2] = 3;
            context->slavelist[slave].SMtype[3] = 4;
            
            /* 配置SM0（写邮箱）
             * StartAddr: 写邮箱起始地址
             * SMlength: 邮箱长度
             * SMflags: 同步管理器标志（包含方向、模式等） */
            context->slavelist[slave].SM[0].StartAddr = htoes(context->slavelist[slave].mbx_wo);
            context->slavelist[slave].SM[0].SMlength = htoes(context->slavelist[slave].mbx_l);
            context->slavelist[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);
            
            /* 配置SM1（读邮箱） */
            context->slavelist[slave].SM[1].StartAddr = htoes(context->slavelist[slave].mbx_ro);
            context->slavelist[slave].SM[1].SMlength = htoes(context->slavelist[slave].mbx_rl);
            context->slavelist[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);
            
            /* 获取邮箱协议支持信息
             * 之前启动的读取操作现在完成 */
            eedat = ecx_readeeprom2(context, slave, EC_TIMEOUTEEP);
            context->slavelist[slave].mbx_proto = (uint16)etohl(eedat);
         }
         /* 通过SII（Slave Information Interface）查找配置信息
          * 首先检查是否有相同ID的从站已配置过
          * 如果有，则复用之前的配置（提高效率）
          * 如果没有，则从EEPROM读取完整的SII信息 */
         if (!ecx_lookup_prev_sii(context, slave))
         {
            /* 查找SII通用段（General Section）的偏移量
             * 通用段包含从站的基本配置信息 */
            ssigen = ecx_siifind(context, slave, ECT_SII_GENERAL);
            
            /* 如果找到通用段，则读取其中的配置信息 */
            if (ssigen)
            {
               /* 读取各种协议的详细支持信息
                * CoEdetails: CANopen over EtherCAT 协议详情
                * FoEdetails: File over EtherCAT 协议详情
                * EoEdetails: Ethernet over EtherCAT 协议详情
                * SoEdetails: Servo over EtherCAT 协议详情 */
               context->slavelist[slave].CoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x07);
               context->slavelist[slave].FoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x08);
               context->slavelist[slave].EoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x09);
               context->slavelist[slave].SoEdetails = ecx_siigetbyte(context, slave, ssigen + 0x0a);
               
               /* 检查是否支持阻塞LRW（逻辑读写）操作
                * 位0x02表示支持阻塞LRW */
               if ((ecx_siigetbyte(context, slave, ssigen + 0x0d) & 0x02) > 0)
               {
                  context->slavelist[slave].blockLRW = 1;
                  context->slavelist[0].blockLRW++;  /* 主站也记录此信息 */
               }
               
               /* 读取从站从E-Bus消耗的电流
                * Ebuscurrent: 以mA为单位的电流值（16位）
                * 低字节：电流低8位
                * 高字节：电流高8位 */
               context->slavelist[slave].Ebuscurrent = ecx_siigetbyte(context, slave, ssigen + 0x0e);
               context->slavelist[slave].Ebuscurrent += ecx_siigetbyte(context, slave, ssigen + 0x0f) << 8;
               /* 累加到主站的E-Bus电流总量 */
               context->slavelist[0].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
            }
            /* 读取SII字符串段（Strings Section）
             * 字符串段包含从站的名称等文本信息 */
            if (ecx_siifind(context, slave, ECT_SII_STRING) > 0)
            {
               /* 读取第一个字符串（通常是设备名称） */
               ecx_siistring(context, context->slavelist[slave].name, slave, 1);
            }
            /* 如果未找到从站名称，则构造一个名称
             * 格式：? M:制造商ID I:产品ID */
            else
            {
               snprintf(context->slavelist[slave].name, EC_MAXNAME, "? M:%8.8x I:%8.8x",
                       (unsigned int)context->slavelist[slave].eep_man,
                       (unsigned int)context->slavelist[slave].eep_id);
            }
            /* 读取SII同步管理器段（SM Section）
             * SM段包含同步管理器的配置信息 */
            nSM = ecx_siiSM(context, slave, &context->eepSM);
            
            /* 如果找到SM配置，则更新SM信息 */
            if (nSM > 0)
            {
               /* 配置第一个SM（SM0）
                * PhStart: 物理起始地址
                * Plength: 长度
                * Creg: 控制寄存器
                * Activate: 激活标志 */
               context->slavelist[slave].SM[0].StartAddr = htoes(context->eepSM.PhStart);
               context->slavelist[slave].SM[0].SMlength = htoes(context->eepSM.Plength);
               context->slavelist[slave].SM[0].SMflags =
                   htoel((context->eepSM.Creg) + (context->eepSM.Activate << 16));
               
               /* 读取并配置剩余的SM（SM1到SM7）
                * EC_MAXSM是最大支持的SM数量 */
               SMc = 1;
               while ((SMc < EC_MAXSM) && ecx_siiSMnext(context, slave, &context->eepSM, SMc))
               {
                  context->slavelist[slave].SM[SMc].StartAddr = htoes(context->eepSM.PhStart);
                  context->slavelist[slave].SM[SMc].SMlength = htoes(context->eepSM.Plength);
                  context->slavelist[slave].SM[SMc].SMflags =
                       htoel((context->eepSM.Creg) + (context->eepSM.Activate << 16));
                  SMc++;
               }
            }
            /* 读取SII FMMU段（Fieldbus Memory Management Unit）
             * FMMU用于将主站内存映射到从站内存
             * 每个FMMU可以映射一个PDO区域 */
            if (ecx_siiFMMU(context, slave, &context->eepFMMU))
            {
               /* 保存每个FMMU的功能代码
                * 0xff表示该FMMU未使用 */
               if (context->eepFMMU.FMMU0 != 0xff)
               {
                  context->slavelist[slave].FMMU0func = context->eepFMMU.FMMU0;
               }
               if (context->eepFMMU.FMMU1 != 0xff)
               {
                  context->slavelist[slave].FMMU1func = context->eepFMMU.FMMU1;
               }
               if (context->eepFMMU.FMMU2 != 0xff)
               {
                  context->slavelist[slave].FMMU2func = context->eepFMMU.FMMU2;
               }
               if (context->eepFMMU.FMMU3 != 0xff)
               {
                  context->slavelist[slave].FMMU3func = context->eepFMMU.FMMU3;
               }
            }
         }

         /* 验证并修复邮箱配置
          * 如果从站有邮箱但SM配置无效，则使用默认配置 */
         if (context->slavelist[slave].mbx_l > 0)
         {
            /* 检查SM0（写邮箱）配置是否有效
             * 起始地址为0表示配置无效（这种情况不应发生） */
            if (context->slavelist[slave].SM[0].StartAddr == 0x0000)
            {
               EC_PRINT("Slave %d has no proper mailbox in configuration, try default.\n", slave);
               /* 使用默认写邮箱配置
                * 地址：0x1000
                * 长度：128字节 */
               context->slavelist[slave].SM[0].StartAddr = htoes(0x1000);
               context->slavelist[slave].SM[0].SMlength = htoes(0x0080);
               context->slavelist[slave].SM[0].SMflags = htoel(EC_DEFAULTMBXSM0);
               context->slavelist[slave].SMtype[0] = 1;
            }
            
            /* 检查SM1（读邮箱）配置是否有效 */
            if (context->slavelist[slave].SM[1].StartAddr == 0x0000)
            {
               EC_PRINT("Slave %d has no proper mailbox out configuration, try default.\n", slave);
               /* 使用默认读邮箱配置
                * 地址：0x1080（紧接写邮箱之后）
                * 长度：128字节 */
               context->slavelist[slave].SM[1].StartAddr = htoes(0x1080);
               context->slavelist[slave].SM[1].SMlength = htoes(0x0080);
               context->slavelist[slave].SM[1].SMflags = htoel(EC_DEFAULTMBXSM1);
               context->slavelist[slave].SMtype[1] = 2;
            }
            
            /* 将SM0和SM1配置写入从站
             * 使用一个数据报同时写入两个SM
             * 这可以解决旧版NETX芯片的时序问题 */
            ecx_FPWR(&context->port, configadr, ECT_REG_SM0, sizeof(ec_smt) * 2,
                     &(context->slavelist[slave].SM[0]), EC_TIMEOUTRET3);
         }
         /* 将EEPROM访问权交给PDI（Process Data Interface）
          * 某些从站在INIT到PRE_OP状态转换期间需要PDI访问EEPROM
          * 这允许从站在状态转换时读取其配置 */
         ecx_eeprom2pdi(context, slave);
         
         /* 请求从站转换到PRE_OP状态
          * PRE_OP状态允许邮箱通信，但不允许过程数据交换
          * 用户可以通过设置manualstatechange标志禁用自动状态转换 */
         if (context->manualstatechange == 0)
         {
            /* 写入ALCTL寄存器请求PRE_OP状态
             * EC_STATE_PRE_OP: 目标状态
             * EC_STATE_ACK: 确认位 */
            ecx_FPWRw(&context->port,
                      configadr,
                      ECT_REG_ALCTL,
                      htoes(EC_STATE_PRE_OP | EC_STATE_ACK),
                      EC_TIMEOUTRET3);
         }
      }
   }
   
   /* 返回工作计数器（从站数量）
    * 如果没有检测到从站，返回0 */
   return wkc;
}

/** 查找先前已配置过的相同从站映射（缓存复用机制）
 * @param[in]  context 上下文结构体指针
 * @param[in]  slave   当前从站编号
 * @param[out] Osize   输出映射位大小
 * @param[out] Isize   输入映射位大小
 * @return 找到相同从站并复制映射返回 1，否则返回 0
 *
 * 功能：
 * - 在已处理的从站中搜索与当前从站具有相同 ID 的从站
 * - 如果找到，直接复制其映射配置（SM 长度、类型、Obits、Ibits）
 * - 避免对相同型号的从站重复发起 EEPROM/CoE 读取，节省配置时间
 *
 * 原理：
 * - SII 映射对于相同从站 ID 是恒定不变的（EEPROM 内容相同）
 * - 相同厂商 ID + 产品代码 + 修订号 = 完全相同的映射配置
 * - 适用于总线上有多个相同型号从站的场景（如多轴伺服系统）
 */
static int ecx_lookup_mapping(ecx_contextt *context, uint16 slave, uint32 *Osize, uint32 *Isize)
{
   int i, nSM;

   /* 只有从站编号大于1（不是第一个从站）且网络上有从站时才需要查找 */
   if ((slave > 1) && (context->slavecount > 0))
   {
      /* 从第1个从站开始遍历，查找与当前从站具有相同身份的从站 */
      /* 身份判定三要素：厂商ID(eep_man) + 产品代码(eep_id) + 修订号(eep_rev) */
      i = 1;
      while (((context->slavelist[i].eep_man != context->slavelist[slave].eep_man) ||
              (context->slavelist[i].eep_id != context->slavelist[slave].eep_id) ||
              (context->slavelist[i].eep_rev != context->slavelist[slave].eep_rev)) &&
             (i < slave))
      {
         i++;
      }

      /* 如果找到匹配的从站（i < slave 表示在[1, slave-1]范围内找到） */
      if (i < slave)
      {
         /* 复制所有同步管理器的配置 */
         for (nSM = 0; nSM < EC_MAXSM; nSM++)
         {
            /* 复制 SM 长度：决定该 SM 对应的 PDO 数据缓冲区大小 */
            context->slavelist[slave].SM[nSM].SMlength = context->slavelist[i].SM[nSM].SMlength;
            /* 复制 SM 类型：1=邮箱读/写，2=邮箱读，3=PDO输出(RX)，4=PDO输入(TX) */
            context->slavelist[slave].SMtype[nSM] = context->slavelist[i].SMtype[nSM];
         }

         /* 复制输入/输出映射的位大小 */
         *Osize = context->slavelist[i].Obits;
         *Isize = context->slavelist[i].Ibits;
         context->slavelist[slave].Obits = (uint16)*Osize;
         context->slavelist[slave].Ibits = (uint16)*Isize;

         EC_PRINT("Copy mapping slave %d from %d.\n", slave, i);
         return 1;  /* 成功复用映射 */
      }
   }

   /* 未找到匹配的先前从站，需要自行读取映射 */
   return 0;
}

/** 通过 CoE 或 SoE 读取从站的 PDO 映射
 * @param[in] context  上下文结构体指针
 * @param[in] slave    从站编号
 * @param[in] thread_n 线程编号（用于多线程版本）
 * @return 成功返回 1
 * 
 * 功能：
 * - 检查从站是否处于 PRE_OP 状态
 * - 执行从站配置钩子（PO2SOconfig）
 * - 如果支持 CoE，通过 CoE 读取 PDO 映射
 *   - 优先使用 Complete Access (CA) 模式
 *   - 如果 CA 不可用，使用普通 CoE 模式
 * - 如果支持 SoE 且 CoE 未找到映射，通过 SoE 读取 IDN 映射
 * - 将映射大小保存到从站结构中
 */
static int ecx_map_coe_soe(ecx_contextt *context, uint16 slave, int thread_n)
{
   uint32 Isize, Osize;  /* 本从站输入 / 输出过程数据的总位大小（bits） */
   int rval;              /* CoE/SoE 读取函数的返回值，非 0 表示成功 */

   /* 步骤1：等待从站进入 PRE-OP 状态
    * 只有在 PRE-OP 下才能访问 CoE 邮箱（INIT 阶段邮箱还未开启，
    * SAFE-OP/OP 下修改 PDO 映射不被允许）。
    * EC_TIMEOUTSTATE 默认 2s，超时则继续往下走，但后续 SDO 大概率失败。 */
   ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */

   EC_PRINT(" >Slave %d, configadr %x, state %2.2x\n",
            slave, context->slavelist[slave].configadr, context->slavelist[slave].state);

   /* 步骤2：如果用户加载了 ENI（EtherCAT Network Information）XML 配置文件，
    * 则按照 ENI 中定义的 PS（Pre-Op → Safe-Op）阶段初始化命令序列，
    * 通过邮箱下发这些 SDO 写操作（如修改 PDO 分配、设置 DC 等）。 */
   if (context->ENI)
   {
      (void)ecx_mbxENIinitcmds(context, slave, ECT_ESMTRANS_PS);
   }
   /* 步骤3：执行用户注册的从站配置钩子（PO2SOconfig）
    * 这是应用层修改 PDO 映射、写 SDO 参数的标准时机。
    * 只有在用户给该从站注册了回调时才会调用。
    * execute slave configuration hook Pre-Op to Safe-OP */
   if (context->slavelist[slave].PO2SOconfig) /* only if registered */
   {
      context->slavelist[slave].PO2SOconfig(context, slave);
   }

   /* 步骤4：读取从站的 PDO 映射（即输入 / 输出过程数据的大小与结构）
    * 优先级：CoE (CA) → CoE (普通) → SoE
    * Find IO mapping in slave */
   Isize = 0;
   Osize = 0;
   /* 4a：从站支持 CoE（CANopen over EtherCAT） */
   if (context->slavelist[slave].mbx_proto & ECT_MBXPROT_COE) /* has CoE */
   {
      rval = 0;
      /* 4a-1：优先使用 Complete Access (CA) 模式
       * CA 模式可以一次性读取 0x1C12/0x1C13 及其所有子索引，效率最高，
       * 且可读出完整的 PDO 映射结构 (0x1600 / 0x1A00 ...)。
       * 只有 CoEdetails 位 ECT_COEDET_SDOCA 为 1 的从站才支持。 */
      if (context->slavelist[slave].CoEdetails & ECT_COEDET_SDOCA) /* has Complete Access */
      {
         /* read PDO mapping via CoE and use Complete Access */
         rval = ecx_readPDOmapCA(context, slave, thread_n, &Osize, &Isize);
      }
      /* 4a-2：CA 不支持或读取失败时，回退到普通 SDO 单条读取 */
      if (!rval) /* CA not available or not succeeded */
      {
         /* read PDO mapping via CoE */
         rval = ecx_readPDOmap(context, slave, &Osize, &Isize);
      }
      EC_PRINT("  CoE Osize:%u Isize:%u\n", Osize, Isize);
   }
   /* 4b：如果 CoE 没读出任何映射（Isize==Osize==0），
    * 且从站支持 SoE（Servo Drive Profile over EtherCAT，通常是伺服），
    * 则通过 SoE 读取 AT / MDT（从站→主站 / 主站→从站）的 IDN 映射。
    * 与 CoE 不同，SoE 需要这里手动把大小写回 SM2 (输出) / SM3 (输入) 的长度寄存器。 */
   if ((!Isize && !Osize) && (context->slavelist[slave].mbx_proto & ECT_MBXPROT_SOE)) /* has SoE */
   {
      /* read AT / MDT mapping via SoE */
      rval = ecx_readIDNmap(context, slave, &Osize, &Isize);
      /* 位数向上取整为字节数，并转网络字节序后写入 SM 长度寄存器 */
      context->slavelist[slave].SM[2].SMlength = htoes((uint16)((Osize + 7) / 8));
      context->slavelist[slave].SM[3].SMlength = htoes((uint16)((Isize + 7) / 8));
      EC_PRINT("  SoE Osize:%u Isize:%u\n", Osize, Isize);
   }

   /* 步骤5：把最终得到的位大小保存到从站结构中，
    * 供后续 ecx_config_map_group 在主站侧进行 IOmap 布局与 FMMU 配置时使用。
    * 若此处仍为 0，调用方 (ecx_config_map_group) 会回退调用 ecx_map_sii
    * 从 EEPROM (SII) 读取静态 PDO 描述。 */
   context->slavelist[slave].Obits = (uint16)Osize;
   context->slavelist[slave].Ibits = (uint16)Isize;

   return 1;
}

/** 从 SII（Slave Information Interface）读取 PDO 映射
 * @param[in] context 上下文结构体指针
 * @param[in] slave   从站编号
 * @return 成功返回 1
 *
 * 功能：
 * - 如果 CoE/SoE 未找到映射，尝试从 SII（EEPROM）读取
 * - 首先查找具有相同 ID 的先前从站的映射（缓存复用）
 * - 如果未找到，从 EEPROM 读取 PDO 映射信息
 * - 根据 PDO 位大小计算 SM 长度（字节对齐）
 * - 设置 SM 类型：3=输出（RXPDO），4=输入（TXPDO）
 * - 将映射大小保存到从站结构中
 *
 * 背景：
 * - SII 中的 PDO 信息是静态的，存放在从站 EEPROM 中
 * - 每个 PDO 包含：索引(Index)、所属 SM、entry 列表及总位大小
 * - t=0 时读取 TXPDO（输入，从站→主站），t=1 时读取 RXPDO（输出，主站→从站）
 */
static int ecx_map_sii(ecx_contextt *context, uint16 slave)
{
   uint32 Isize, Osize;  /* 输入/输出映射的总位大小 */
   int nSM;               /* 同步管理器索引循环变量 */
   ec_eepromPDOt eepPDO;  /* 存放从 SII 解析出的 PDO 信息 */

   /* 从从站结构中获取当前已知的映射大小（可能已由 CoE/SoE 设置） 
   一般来说，如果之前没有手动设置，这里读出来应该是0 */
   Osize = context->slavelist[slave].Obits;
   Isize = context->slavelist[slave].Ibits;

   /* 步骤1：如果当前没有输入也没有输出映射，尝试复用先前从站的映射 */
   if (!Isize && !Osize) /* find PDO in previous slave with same ID */
   {
      (void)ecx_lookup_mapping(context, slave, &Osize, &Isize);
   }

   /* 步骤2：如果复用也失败，从 EEPROM 的 SII 区域读取 PDO 映射 */
   if (!Isize && !Osize) /* find PDO mapping by SII */
   {
      /* 清零 eepPDO 结构体，避免残留数据干扰 */
      memset(&eepPDO, 0, sizeof(eepPDO));

      /* 步骤2a：读取 TXPDO（t=0，输入方向，从站→主站） */
      /* ecx_siiPDO 从 EEPROM 的 ECT_SII_PDO 段解析 PDO 列表，
       * 返回所有 TXPDO 的总位大小，并填充 eepPDO 结构体 */
      Isize = ecx_siiPDO(context, slave, &eepPDO, 0);
      EC_PRINT("  SII Isize:%u\n", Isize);

      /* 遍历所有 SM，为存在 TXPDO 映射的 SM 设置长度和类型 */
      for (nSM = 0; nSM < EC_MAXSM; nSM++)
      {
         if (eepPDO.SMbitsize[nSM] > 0)  /* 该 SM 有 TXPDO 数据 */
         {
            /* 位大小转字节数，向上取整：(bits + 7) / 8 */
            context->slavelist[slave].SM[nSM].SMlength = htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
            /* SMtype=4 表示输入（TXPDO）：从站产生数据，主站读取 */
            context->slavelist[slave].SMtype[nSM] = 4;
            EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
         }
      }

      /* 步骤2b：读取 RXPDO（t=1，输出方向，主站→从站） */
      /* 注意：此时 eepPDO 会被重新填充，覆盖之前的 TXPDO 数据 */
      Osize = ecx_siiPDO(context, slave, &eepPDO, 1);
      EC_PRINT("  SII Osize:%u\n", Osize);

      /* 遍历所有 SM，为存在 RXPDO 映射的 SM 设置长度和类型 */
      for (nSM = 0; nSM < EC_MAXSM; nSM++)
      {
         if (eepPDO.SMbitsize[nSM] > 0)  /* 该 SM 有 RXPDO 数据 */
         {
            /* 位大小转字节数，向上取整 */
            context->slavelist[slave].SM[nSM].SMlength = htoes((eepPDO.SMbitsize[nSM] + 7) / 8);
            /* SMtype=3 表示输出（RXPDO）：主站发送数据，从站接收 */
            context->slavelist[slave].SMtype[nSM] = 3;
            EC_PRINT("    SM%d length %d\n", nSM, eepPDO.SMbitsize[nSM]);
         }
      }
   }

   /* 步骤3：将最终的位大小保存到从站结构中 */
   context->slavelist[slave].Obits = (uint16)Osize;
   context->slavelist[slave].Ibits = (uint16)Isize;
   EC_PRINT("     ISIZE:%d %d OSIZE:%d\n",
            context->slavelist[slave].Ibits, Isize, context->slavelist[slave].Obits);

   return 1;
}

/** 配置同步管理器（Sync Manager）并写入 ESC 寄存器
 * @param[in] context 上下文结构体指针
 * @param[in] slave   从站编号
 * @return 成功返回 1
 *
 * 功能：
 * - 配置 SM0 和 SM1（邮箱 SM，如果使用本地邮箱配置）
 * - 配置 SM2 到 SMx（PDO SM，过程数据同步管理器）
 * - 根据 SM 长度设置或清除使能标志（enable/disable）
 * - 通过 FPWR 命令将 SM 配置写入从站 ESC 寄存器
 * - 计算并保存从站的输入/输出字节数
 *
 * 背景：
 * - SM（Sync Manager）是 ESC 内部的"门控"机制，管理对 PD RAM 区域的访问
 * - 每个 SM 由一组配置寄存器描述：StartAddr（起始地址）、Length（长度）、Flags（标志）
 * - SM0/SM1 通常用于邮箱通信（CoE/SDO），SM2/SM3 通常用于 PDO 数据传输
 * - 本函数将软件层面计算好的 SM 参数，真正写入 ESC 硬件寄存器
 */
static int ecx_map_sm(ecx_contextt *context, uint16 slave)
{
   uint16 configadr;  /* 从站的配置地址（已分配的配置站点地址） */
   int nSM;            /* 同步管理器索引循环变量 */

   /* 获取从站的配置地址，后续所有寄存器写入都通过该地址进行 */
   configadr = context->slavelist[slave].configadr;

   EC_PRINT("  SM programming\n");

   /* ==================== 配置邮箱 SM0（写邮箱，主站→从站）==================== */
   /* mbx_l = 0 表示使用本地邮箱（非外部邮箱），SM[0].StartAddr != 0 表示已配置 */
   if (!context->slavelist[slave].mbx_l && context->slavelist[slave].SM[0].StartAddr)
   {
      /* FPWR = Configured Physical Write，使用配置地址写入从站寄存器 */
      /* 将 SM0 的完整配置（ec_smt 结构体，8字节）写入 ESC 的 ECT_REG_SM0 寄存器 */
      ecx_FPWR(&context->port, configadr, ECT_REG_SM0,
               sizeof(ec_smt), &(context->slavelist[slave].SM[0]), EC_TIMEOUTRET3);
      EC_PRINT("    SM0 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
               context->slavelist[slave].SMtype[0],
               etohs(context->slavelist[slave].SM[0].StartAddr),
               etohl(context->slavelist[slave].SM[0].SMflags));
   }

   /* ==================== 配置邮箱 SM1（读邮箱，从站→主站）==================== */
   /* SM1 是从站回复主站的邮箱通道 */
   if (!context->slavelist[slave].mbx_l && context->slavelist[slave].SM[1].StartAddr)
   {
      ecx_FPWR(&context->port, configadr, ECT_REG_SM1,
               sizeof(ec_smt), &context->slavelist[slave].SM[1], EC_TIMEOUTRET3);
      EC_PRINT("    SM1 Type:%d StartAddr:%4.4x Flags:%8.8x\n",
               context->slavelist[slave].SMtype[1],
               etohs(context->slavelist[slave].SM[1].StartAddr),
               etohl(context->slavelist[slave].SM[1].SMflags));
   }

   /* ==================== 配置 PDO SM（SM2 ~ SMx）==================== */
   for (nSM = 2; nSM < EC_MAXSM; nSM++)
   {
      /* 仅处理已配置起始地址的 SM（StartAddr != 0） */
      if (context->slavelist[slave].SM[nSM].StartAddr)
      {
         /* 步骤1：根据 SM 长度决定是否使能该 SM */
         /* SMlength == 0 表示该 SM 没有数据，需要禁用（清除 enable 标志位） */
         if (context->slavelist[slave].SM[nSM].SMlength == 0)
         {
            /* EC_SMENABLEMASK 是使能标志的位掩码，&= 操作清除 enable 位 */
            context->slavelist[slave].SM[nSM].SMflags =
                htoel(etohl(context->slavelist[slave].SM[nSM].SMflags) & EC_SMENABLEMASK);
         }
         /* SMlength != 0 表示有数据需要传输，需要使能（设置 enable 标志位） */
         else
         {
            /* |= ~EC_SMENABLEMASK 实际上是设置 enable 位（EC_SMENABLEMASK 设计上是取反后或运算） */
            context->slavelist[slave].SM[nSM].SMflags =
                htoel(etohl(context->slavelist[slave].SM[nSM].SMflags) | ~EC_SMENABLEMASK);
         }

         /* 步骤2：将 SM 配置写入 ESC 寄存器 */
         /* ECT_REG_SM0 + nSM * sizeof(ec_smt) 计算第 nSM 个 SM 寄存器的地址 */
         /* 例如：SM2 = 0x0800 + 2*8 = 0x0810，SM3 = 0x0800 + 3*8 = 0x0818 */
         ecx_FPWR(&context->port, configadr, (uint16)(ECT_REG_SM0 + (nSM * sizeof(ec_smt))),
                  sizeof(ec_smt), &context->slavelist[slave].SM[nSM], EC_TIMEOUTRET3);
         EC_PRINT("    SM%d Type:%d StartAddr:%4.4x Flags:%8.8x\n", nSM,
                  context->slavelist[slave].SMtype[nSM],
                  etohs(context->slavelist[slave].SM[nSM].StartAddr),
                  etohl(context->slavelist[slave].SM[nSM].SMflags));
      }
   }

   /* ==================== 计算并保存输入/输出字节数 ==================== */
   /* 如果位大小大于7（即至少1个字节），计算字节数 = (bits + 7) / 8 */
   /* 这是供上层应用快速获取 IO 数据大小的便利字段 */
   if (context->slavelist[slave].Ibits > 7)
   {
      context->slavelist[slave].Ibytes = (context->slavelist[slave].Ibits + 7) / 8;
   }
   if (context->slavelist[slave].Obits > 7)
   {
      context->slavelist[slave].Obytes = (context->slavelist[slave].Obits + 7) / 8;
   }

   return 1;
}

#if EC_MAX_MAPT > 1
/** 映射线程函数（多线程版本）
 * @param param 指向 ecx_mapt_t 结构体的指针，包含上下文、从站编号和线程编号
 * 0
 * 功能：
 * - 调用 ecx_map_coe_soe 执行 CoE/SoE 映射
 * - 完成后设置 running 标志为 0
 */
OSAL_THREAD_FUNC ecx_mapper_thread(void *param)
{
   ecx_mapt_t *maptp;
   maptp = param;
   ecx_map_coe_soe(maptp->context, maptp->slave, maptp->thread_n);
   maptp->running = 0;
}

/** 查找可用的映射线程槽位
 * @return 可用的线程槽位索引，如果没有可用槽位返回 -1
 * 
 * 功能：
 * - 遍历所有映射线程槽位
 * - 查找第一个未运行的槽位（running == 0）
 * - 返回该槽位的索引或 -1（如果全部都在运行）
 */
static int ecx_find_mapt(void)
{
   int p;
   p = 0;
   while ((p < EC_MAX_MAPT) && ecx_mapt[p].running)
   {
      p++;
   }
   if(p < EC_MAX_MAPT)
   {
      return p;
   }
   else
   {
      return -1;
   }
}
#endif

/** 获取当前运行的映射线程数量
 * @return 正在运行的线程数量
 * 
 * 功能：
 * - 遍历所有映射线程槽位
 * - 统计 running 标志为 1 的线程数量
 * - 用于等待所有映射线程完成
 */
static int ecx_get_threadcount(void)
{
   int thrc, thrn;
   thrc = 0;
   for (thrn = 0; thrn < EC_MAX_MAPT; thrn++)
   {
      thrc += ecx_mapt[thrn].running;
   }
   return thrc;
}

/**
 * 查找并配置从站的PDO映射关系
 *
 * 功能：
 * - 查找从站的CoE（CANopen over EtherCAT）和SoE（Servo over EtherCAT）映射
 * - 查找SII（Slave Information Interface）映射
 * - 配置同步管理器（Sync Manager, SM）
 *
 * 执行流程：
 * 1. 初始化所有映射线程的运行标志
 * 2. 遍历指定组（或所有组）的从站，查找CoE/SoE映射
 *    - 多线程版本：创建线程并行处理每个从站的映射
 *    - 单线程版本：串行处理每个从站的映射
 * 3. 等待所有映射线程完成
 * 4. 遍历从站，查找SII映射并编程同步管理器
 *
 * @param[in] context 上下文结构体指针，包含端口和从站信息
 * @param[in] group 组编号，0表示处理所有组
 */
static void ecx_config_find_mappings(ecx_contextt *context, uint8 group)
{
   int thrn, thrc;  /* thrn: 线程编号，thrc: 当前运行的线程数量 */
   uint16 slave;    /* 从站编号循环变量 */

   /* 初始化所有映射线程的运行标志为0（未运行） */
   for (thrn = 0; thrn < EC_MAX_MAPT; thrn++)
   {
      ecx_mapt[thrn].running = 0;
   }
   /* 查找指定组（或所有组）中从站的CoE和SoE映射
    * 可以使用多个线程并行处理以提高效率 */
   for (slave = 1; slave <= context->slavecount; slave++)
   {
      /* 如果group为0（处理所有组）或从站属于指定组，则处理该从站 */
      if (!group || (group == context->slavelist[slave].group))
      {
#if EC_MAX_MAPT > 1
         /* 多线程版本：为每个从站创建独立的映射线程 */
         /* 查找可用的映射线程槽位，如果没有可用则等待1ms */
         while ((thrn = ecx_find_mapt()) < 0)
         {
            osal_usleep(1000);
         }
         /* 设置线程参数 */
         ecx_mapt[thrn].context = context;  /* 上下文指针 */
         ecx_mapt[thrn].slave = slave;      /* 要处理的从站编号 */
         ecx_mapt[thrn].thread_n = thrn;    /* 线程编号 */
         ecx_mapt[thrn].running = 1;        /* 标记线程为运行状态 */
         /* 创建映射线程，栈大小为128KB */
         osal_thread_create(&(ecx_threadh[thrn]), 128000,
                            &ecx_mapper_thread, &(ecx_mapt[thrn]));
#else
         /* 单线程版本：串行处理每个从站的CoE和SoE映射 */
         ecx_map_coe_soe(context, slave, 0);
#endif
      }
   }
   /* 等待所有映射线程完成工作
    * 循环检查当前运行的线程数量，直到所有线程完成 */
   do
   {
      thrc = ecx_get_threadcount();  /* 获取当前运行的线程数量 */
      if (thrc)
      {
         osal_usleep(1000);  /* 如果还有线程在运行，等待1ms */
      }
   } while (thrc); // do while和while的区别是这里是先执行后判断，确保至少执行一次
   /* 查找从站的SII映射并编程同步管理器（SM）
    * 这部分必须串行执行，因为涉及从站寄存器的写入操作 */
   for (slave = 1; slave <= context->slavecount; slave++)
   {
      /* 如果group为0（处理所有组）或从站属于指定组，则处理该从站 */
      if (!group || (group == context->slavelist[slave].group))
      {
         ecx_map_sii(context, slave);  /* 查找SII映射 */
         ecx_map_sm(context, slave);   /* 编程同步管理器 */
      }
   }
}

/** 创建输入映射并配置 FMMU
 * @param[in]  context  上下文结构体指针
 * @param[out] pIOmap   指向 IOmap 的指针
 * @param[in]  group    组编号
 * @param[in]  slave    从站编号
 * @param[in,out] LogAddr 逻辑地址指针，用于计算映射地址
 * @param[in,out] BitPos  位位置指针，用于位级映射
 * 
 * 功能：
 * - 查找贡献于输入映射的同步管理器（SM）
 * - 配置 FMMU（现场总线内存管理单元）将逻辑地址映射到物理地址
 * - 处理位级和字节级映射
 * - 配置从站的输入指针指向 IOmap
 */
/* 【第二层桥梁：SM物理寄存器 ↔ EtherCAT逻辑地址】（输入方向）
 *
 * 与ecx_config_create_output_mappings对称，本函数配置输入PDO的FMMU映射。
 * 区别：搜索SMtype==4（输入PDO），FMMUtype=1（读）。
 *
 * FMMUtype=1(读)：帧经过时ESC从SM缓冲区读取数据填入帧的对应位置，
 * 帧返回主站后数据被写入IOmap的对应区域。
 */
static void ecx_config_create_input_mappings(ecx_contextt *context, void *pIOmap,
                                             uint8 group, int16 slave, uint32 *LogAddr, uint8 *BitPos)
{
   int BitCount = 0;
   int FMMUdone = 0;
   uint16 ByteCount = 0;
   uint16 FMMUsize = 0;
   uint8 SMc = 0;
   uint16 EndAddr;
   uint16 SMlength;
   uint16 configadr;
   uint8 FMMUc;

   EC_PRINT(" =Slave %d, INPUT MAPPING\n", slave);

   configadr = context->slavelist[slave].configadr;
   FMMUc = context->slavelist[slave].FMMUunused;
   if (context->slavelist[slave].Obits) /* find free FMMU */
   {
      while (context->slavelist[slave].FMMU[FMMUc].LogStart)
      {
         FMMUc++;
      }
   }
   /* 搜索贡献于输入映射的SM（SMtype==4表示输入PDO） */
   while ((SMc < EC_MAXSM) && (FMMUdone < ((context->slavelist[slave].Ibits + 7) / 8)))
   {
      EC_PRINT("    FMMU %d\n", FMMUc);
      while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 4))
      {
         SMc++;
      }
      EC_PRINT("      SM%d\n", SMc);
      /* FMMU.PhysStart = SM3缓冲区的物理起始地址（如0x1000），
       * MCU将编码器值等数据按TXPDO映射清单排列在此地址 */
      context->slavelist[slave].FMMU[FMMUc].PhysStart =
          context->slavelist[slave].SM[SMc].StartAddr;
      SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
      ByteCount += SMlength;
      BitCount += SMlength * 8;
      EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      while ((BitCount < context->slavelist[slave].Ibits) && (SMc < (EC_MAXSM - 1))) /* more SM for input */
      {
         SMc++;
         while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 4))
         {
            SMc++;
         }
         /* if addresses from more SM connect use one FMMU otherwise break up in multiple FMMU */
         if (etohs(context->slavelist[slave].SM[SMc].StartAddr) > EndAddr)
         {
            break;
         }
         EC_PRINT("      SM%d\n", SMc);
         SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
         ByteCount += SMlength;
         BitCount += SMlength * 8;
         EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      }

      /* bit oriented slave */
      if (!context->slavelist[slave].Ibytes)
      {
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos += context->slavelist[slave].Ibits - 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
         FMMUsize = (uint16)(*LogAddr - etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) + 1);
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos += 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
      }
      /* byte oriented slave */
      else
      {
         if (*BitPos)
         {
            *LogAddr += 1;
            *BitPos = 0;
         }
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos = 7;
         FMMUsize = ByteCount;
         if ((FMMUsize + FMMUdone) > (int)context->slavelist[slave].Ibytes)
         {
            FMMUsize = (uint16)(context->slavelist[slave].Ibytes - FMMUdone);
         }
         *LogAddr += FMMUsize;
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos = 0;
      }
      FMMUdone += FMMUsize;
      if (context->slavelist[slave].FMMU[FMMUc].LogLength)
      {
         context->slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
         /* FMMUtype=1表示读（输入）：从站→主站，ESC从SM缓冲区读取数据填入帧 */
         context->slavelist[slave].FMMU[FMMUc].FMMUtype = 1;
         context->slavelist[slave].FMMU[FMMUc].FMMUactive = 1;
         /* 将FMMU配置写入ESC寄存器，配置立即生效 */
         ecx_FPWR(&context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
                  sizeof(ec_fmmut), &(context->slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);
      }
      if (!context->slavelist[slave].inputs)
      {
         /* 【第三层桥梁：逻辑地址 → IOmap指针】（输入方向）
          * 与outputs指针计算方式相同：IOmap偏移 = 逻辑地址 - logstartaddr
          * 如电机1输入：逻辑地址0x0E → &IOmap[14] */
         if (group)
         {
            context->slavelist[slave].inputs =
                (uint8 *)(pIOmap) +
                etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) -
                context->grouplist[group].logstartaddr;
         }
         else
         {
            context->slavelist[slave].inputs =
                (uint8 *)(pIOmap) +
                etohl(context->slavelist[slave].FMMU[FMMUc].LogStart);
         }
         context->slavelist[slave].Istartbit =
             context->slavelist[slave].FMMU[FMMUc].LogStartbit;
         EC_PRINT("    Inputs %p startbit %d\n",
                  context->slavelist[slave].inputs,
                  context->slavelist[slave].Istartbit);
      }
      FMMUc++;
   }
   context->slavelist[slave].FMMUunused = FMMUc;
}

/** 创建输出映射并配置 FMMU
 * @param[in]  context  上下文结构体指针
 * @param[out] pIOmap   指向 IOmap 的指针
 * @param[in]  group    组编号
 * @param[in]  slave    从站编号
 * @param[in,out] LogAddr 逻辑地址指针，用于计算映射地址
 * @param[in,out] BitPos  位位置指针，用于位级映射
 * 
 * 功能：
 * - 查找贡献于输出映射的同步管理器（SM）
 * - 配置 FMMU（现场总线内存管理单元）将逻辑地址映射到物理地址
 * - 处理位级和字节级映射
 * - 配置从站的输出指针指向 IOmap
 */
/* 【第二层桥梁：SM物理寄存器 ↔ EtherCAT逻辑地址】
 * 
 * 本函数为从站的输出PDO配置FMMU（现场总线内存管理单元）映射。
 * FMMU是ESC芯片内的硬件单元，它的工作就是翻译地址：
 *   - EtherCAT帧读写某个逻辑地址
 *   - FMMU查表：这个逻辑地址对应哪个SM缓冲区
 *   - 从SM缓冲区取/存数据
 * 
 * FMMU不理解数据的含义，只做地址翻译和字节搬运。
 * 理解数据含义的是两端的软件（MCU固件和主站程序）。
 * 
 * 配置流程：
 *   1. 搜索SMtype==3（输出PDO）的SM，获取其物理起始地址和长度
 *   2. 填写FMMU寄存器：LogStart=当前逻辑地址, PhysStart=SM物理地址, FMMUtype=2(写)
 *   3. 通过ecx_FPWR将FMMU配置写入ESC寄存器，配置立即生效
 *   4. 计算从站的outputs指针（第三层桥梁：逻辑地址→IOmap）
 *   5. LogAddr递增，为下一个从站分配逻辑地址
 */
static void ecx_config_create_output_mappings(ecx_contextt *context, void *pIOmap,
                                              uint8 group, int16 slave, uint32 *LogAddr, uint8 *BitPos)
{
   int BitCount = 0;
   int FMMUdone = 0;
   uint16 ByteCount = 0;
   uint16 FMMUsize = 0;
   uint8 SMc = 0;
   uint16 EndAddr;
   uint16 SMlength;
   uint16 configadr;
   uint8 FMMUc;

   EC_PRINT("  OUTPUT MAPPING\n");

   FMMUc = context->slavelist[slave].FMMUunused;
   configadr = context->slavelist[slave].configadr;

   /* 搜索贡献于输出映射的SM（SMtype==3表示输出PDO） */
   while ((SMc < EC_MAXSM) && (FMMUdone < ((context->slavelist[slave].Obits + 7) / 8)))
   {
      EC_PRINT("    FMMU %d\n", FMMUc);
      while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 3))
      {
         SMc++;
      }
      EC_PRINT("      SM%d\n", SMc);
      /* FMMU.PhysStart = SM缓冲区的物理起始地址（如0x0C00），
       * 这是第一层桥梁(MCU配置)的结果：MCU已将PDO数据按映射清单排列在此地址 */
      context->slavelist[slave].FMMU[FMMUc].PhysStart =
          context->slavelist[slave].SM[SMc].StartAddr;
      SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
      ByteCount += SMlength;
      BitCount += SMlength * 8;
      EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      while ((BitCount < context->slavelist[slave].Obits) && (SMc < (EC_MAXSM - 1))) /* more SM for output */
      {
         SMc++;
         while ((SMc < (EC_MAXSM - 1)) && (context->slavelist[slave].SMtype[SMc] != 3))
         {
            SMc++;
         }
         /* if addresses from more SM connect use one FMMU otherwise break up in multiple FMMU */
         if (etohs(context->slavelist[slave].SM[SMc].StartAddr) > EndAddr)
         {
            break;
         }
         EC_PRINT("      SM%d\n", SMc);
         SMlength = etohs(context->slavelist[slave].SM[SMc].SMlength);
         ByteCount += SMlength;
         BitCount += SMlength * 8;
         EndAddr = etohs(context->slavelist[slave].SM[SMc].StartAddr) + SMlength;
      }

      /* bit oriented slave */
      if (!context->slavelist[slave].Obytes)
      {
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos += context->slavelist[slave].Obits - 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
         FMMUsize = (uint16)(*LogAddr - etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) + 1);
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos += 1;
         if (*BitPos > 7)
         {
            *LogAddr += 1;
            *BitPos -= 8;
         }
      }
      /* 字节对齐的从站（大多数伺服驱动器属于此类） */
      else
      {
         if (*BitPos)
         {
            *LogAddr += 1;
            *BitPos = 0;
         }
         /* FMMU.LogStart = 当前逻辑地址（按从站遍历顺序递增分配），
          * 如电机1输出从0x00开始，电机2输出从0x07开始 */
         context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
         context->slavelist[slave].FMMU[FMMUc].LogStartbit = *BitPos;
         *BitPos = 7;
         FMMUsize = ByteCount;
         if ((FMMUsize + FMMUdone) > (int)context->slavelist[slave].Obytes)
         {
            FMMUsize = (uint16)(context->slavelist[slave].Obytes - FMMUdone);
         }
         *LogAddr += FMMUsize;
         context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
         context->slavelist[slave].FMMU[FMMUc].LogEndbit = *BitPos;
         *BitPos = 0;
      }
      FMMUdone += FMMUsize;
      if (context->slavelist[slave].FMMU[FMMUc].LogLength)
      {
         context->slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
         /* FMMUtype=2表示写（输出）：主站→从站，帧数据写入SM缓冲区 */
         context->slavelist[slave].FMMU[FMMUc].FMMUtype = 2;
         context->slavelist[slave].FMMU[FMMUc].FMMUactive = 1;
         /* 将FMMU配置写入ESC寄存器，配置立即生效（硬件直接使用，无需额外读取） */
         ecx_FPWR(&context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
                  sizeof(ec_fmmut), &(context->slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);
      }
      if (!context->slavelist[slave].outputs)
      {
         /* 【第三层桥梁：逻辑地址 → IOmap指针】
          * IOmap偏移 = 逻辑地址 - logstartaddr
          * group=0时logstartaddr=0，所以IOmap偏移就等于逻辑地址
          * 如电机1输出：逻辑地址0x00 → &IOmap[0]
          *   电机2输出：逻辑地址0x07 → &IOmap[7]
          * 用户代码通过结构体指针(如LD3M_PDO_Output*)解读IOmap中的字节含义 */
         if (group)
         {
            context->slavelist[slave].outputs =
                (uint8 *)(pIOmap) +
                etohl(context->slavelist[slave].FMMU[FMMUc].LogStart) -
                context->grouplist[group].logstartaddr;
         }
         else
         {
            context->slavelist[slave].outputs =
                (uint8 *)(pIOmap) +
                etohl(context->slavelist[slave].FMMU[FMMUc].LogStart);
         }
         context->slavelist[slave].Ostartbit =
             context->slavelist[slave].FMMU[FMMUc].LogStartbit;
         EC_PRINT("    slave %d Outputs %p startbit %d\n",
                  slave,
                  context->slavelist[slave].outputs,
                  context->slavelist[slave].Ostartbit);
      }
      FMMUc++;
   }
   context->slavelist[slave].FMMUunused = FMMUc;
}

static void ecx_config_create_mbxstatus_mappings(ecx_contextt *context, void *pIOmap,
                                                 uint8 group, int16 slave, uint32 *LogAddr)
{
   uint16 FMMUsize = 1;
   uint16 configadr;
   uint8 FMMUc;
   int position;

   EC_PRINT(" =Slave %d, MBXSTATUS MAPPING\n", slave);

   configadr = context->slavelist[slave].configadr;
   FMMUc = context->slavelist[slave].FMMUunused;
   if (context->slavelist[slave].mbx_l && (FMMUc < EC_MAXFMMU)) /* slave with mailbox */
   {
      context->slavelist[slave].FMMU[FMMUc].LogStart = htoel(*LogAddr);
      context->slavelist[slave].FMMU[FMMUc].LogStartbit = 0;
      *LogAddr += FMMUsize;
      context->slavelist[slave].FMMU[FMMUc].LogLength = htoes(FMMUsize);
      context->slavelist[slave].FMMU[FMMUc].LogEndbit = 7;
      context->slavelist[slave].FMMU[FMMUc].PhysStart = ECT_REG_SM1STAT;
      context->slavelist[slave].FMMU[FMMUc].PhysStartBit = 0;
      context->slavelist[slave].FMMU[FMMUc].FMMUtype = 1;
      context->slavelist[slave].FMMU[FMMUc].FMMUactive = 1;
      /* program FMMU for input */
      ecx_FPWR(&context->port, configadr, ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc),
               sizeof(ec_fmmut), &(context->slavelist[slave].FMMU[FMMUc]), EC_TIMEOUTRET3);

      position = etohl(context->slavelist[slave].FMMU[FMMUc].LogStart);
      context->slavelist[slave].mbxstatus = (uint8 *)(pIOmap) + position;
      context->grouplist[group].mbxstatuslookup[context->grouplist[group].mbxstatuslength] = slave;
      context->grouplist[group].mbxstatuslength++;
      FMMUc++;
      /* account for MBXSTATUS wkc increment */
      if (!context->slavelist[slave].Ibytes)
         context->grouplist[group].inputsWKC++;
   }
   context->slavelist[slave].FMMUunused = FMMUc;
}

/** 主配置映射函数（非重叠模式）
 * 将指定组的从站 PDO 映射到 IOmap，输出和输入按顺序排列
 * 
 * 三层桥梁总览（本函数执行第二层和第三层）：
 *   第一层: CANopen索引 → SM物理寄存器    (MCU固件完成，理解数据含义)
 *          用户通过SDO写对象字典(0x1600/0x1A00)配置PDO映射清单，
 *          MCU据此配置SM2(输出)和SM3(输入)缓冲区
 *   第二层: SM物理寄存器 ↔ 逻辑地址       (FMMU硬件完成，只搬运字节)
 *          本函数为每个从站配置FMMU，通过ecx_FPWR写入ESC寄存器，
 *          FMMU硬件在帧经过时自动完成地址翻译
 *   第三层: 逻辑地址 → IOmap指针          (SOEM软件完成)
 *          IOmap偏移 = 逻辑地址 - logstartaddr，
 *          用户代码通过结构体指针(如LD3M_PDO_Output*)解读IOmap中的字节含义
 * 
 * @param[in]  context 上下文结构体指针
 * @param[out] pIOmap  指向 IOmap 的指针
 * @param[in]  group   要映射的组，0 = 所有组
 * @return IOmap 大小
 * 
 * 功能：
 * - 查找并配置同步管理器
 * - 创建输出映射并配置 FMMU
 * - 创建输入映射并配置 FMMU
 * - 处理分段以适应 EtherCAT 数据包大小限制
 */
static int ecx_main_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
   /* 定义局部变量 */
   uint16 slave, configadr;          // 从站编号和配置地址
   uint8 BitPos;                     // 位位置，用于位级映射
   uint32 LogAddr = 0;               // 当前逻辑地址
   uint32 oLogAddr = 0;              // 上一个逻辑地址，用于计算差值
   uint32 diff;                      // 地址差值
   uint16 currentsegment = 0;        // 当前分段索引
   uint32 segmentsize = 0;          // 当前分段大小
   uint32 segmentmaxsize = (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM); /* 最大分段大小，第一个分段需考虑分布式时钟开销 */

   /* 检查从站数量和组有效性 */
   if ((context->slavecount > 0) && (group < EC_MAXGROUP))
   {
      EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);
      /* 初始化逻辑地址和组参数 */
      LogAddr = context->grouplist[group].logstartaddr;  // 设置起始逻辑地址
      oLogAddr = LogAddr;                                 // 保存起始地址
      BitPos = 0;                                         // 位位置初始化为0
      context->grouplist[group].nsegments = 0;            // 分段数量初始化
      context->grouplist[group].outputsWKC = 0;          // 输出工作计数器初始化
      context->grouplist[group].inputsWKC = 0;            // 输入工作计数器初始化

      /* Find mappings and program syncmanagers */
      ecx_config_find_mappings(context, group);

      /* 遍历所有从站，创建输出映射并配置 FMMU */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;  // 获取从站配置地址

         /* 检查是否属于指定的组（group=0 表示所有组） */
         if (!group || (group == context->slavelist[slave].group))
         {
            /* 如果从站有输出数据，创建输出映射 */
            if (context->slavelist[slave].Obits)
            {
               ecx_config_create_output_mappings(context, pIOmap, group, slave, &LogAddr, &BitPos);

               /* 如果非打包模式，强制字节对齐 */
               if (context->packedMode == FALSE)
                  {
                  /* 如果输出数据小于8位，强制字节对齐 */
                  if (BitPos)
                  {
                     LogAddr++;      // 逻辑地址递增到下一个字节
                     BitPos = 0;     // 位位置重置为0
                  }
               }

               /* 计算地址差值，用于分段处理 */
               diff = LogAddr - oLogAddr;           // 计算当前逻辑地址与上一个地址的差值
               oLogAddr = LogAddr;                  // 更新上一个地址
               
               /* 如果当前分段加上差值超过最大分段大小，创建新分段 */
               if ((segmentsize + diff) > segmentmaxsize && diff <= segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
               {
                  context->grouplist[group].IOsegment[currentsegment++] = segmentsize;  // 保存当前分段大小
                  segmentsize = 0;                                                  // 重置分段大小
                  segmentmaxsize = EC_MAXLRWDATA;  /* 第一个分段后可以忽略 DC 开销 */
               }
               segmentsize += diff;  // 累加当前分段大小
               
               /* 如果分段大小超过最大值，继续分段 */
               while (segmentsize > segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
               {
                  context->grouplist[group].IOsegment[currentsegment++] = segmentmaxsize;  // 保存最大分段大小
                  segmentsize -= segmentmaxsize;                                         // 减去已保存的大小
                  context->grouplist[group].outputsWKC++;                                // 增加输出工作计数器
                  segmentmaxsize = EC_MAXLRWDATA;  /* 第一个分段后可以忽略 DC 开销 */
               }
               /* 如果从站添加了输出数据且存在未完成的分段，增加输出工作计数器 */
               if (segmentsize && diff)
                  context->grouplist[group].outputsWKC++;
            }
         }
      }
      /* 处理输出映射后的位对齐 */
      if (BitPos)  // 如果存在未对齐的位
      {
         LogAddr++;         // 逻辑地址递增到下一个字节
         oLogAddr = LogAddr; // 更新上一个地址
         BitPos = 0;        // 位位置重置为0
         
         /* 检查是否需要创建新分段 */
         if ((segmentsize + 1) > segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
         {
            context->grouplist[group].IOsegment[currentsegment++] = segmentsize;  // 保存当前分段
            segmentsize = 0;                                                  // 重置分段大小
            context->grouplist[group].outputsWKC++;                            // 增加输出工作计数器
            segmentmaxsize = EC_MAXLRWDATA;  /* 第一个分段后可以忽略 DC 开销 */
         }
         segmentsize += 1;  // 累加分段大小
      }
      /* 保存输出映射信息到组结构 */
      context->grouplist[group].outputs = pIOmap;  // 保存输出映射指针
      context->grouplist[group].Obytes = LogAddr - context->grouplist[group].logstartaddr;  // 计算输出字节数
      context->grouplist[group].nsegments = currentsegment + 1;  // 保存分段数量
      context->grouplist[group].Isegment = currentsegment;      // 输入分段起始索引
      context->grouplist[group].Ioffset = (uint16)segmentsize;  // 输入偏移量
      
      /* 如果是组0，同时保存到主站记录 */
      if (!group)
      {
         context->slavelist[0].outputs = pIOmap;  // 保存输出映射指针
         context->slavelist[0].Obytes = LogAddr -
                                        context->grouplist[group].logstartaddr; /* 在主站记录中保存输出字节数 */
      }

      /* 遍历所有从站，创建输入映射并配置 FMMU */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;  // 获取从站配置地址
         
         /* 检查是否属于指定的组（group=0 表示所有组） */
         if (!group || (group == context->slavelist[slave].group))
         {
            /* 如果从站有输入数据，创建输入映射 */
            if (context->slavelist[slave].Ibits)
            {
               ecx_config_create_input_mappings(context, pIOmap, group, slave, &LogAddr, &BitPos);

               /* 如果非打包模式，强制字节对齐 */
               if (context->packedMode == FALSE)
               {
                  /* 如果输入数据小于8位，强制字节对齐 */
                  if (BitPos)
                  {
                     LogAddr++;      // 逻辑地址递增到下一个字节
                     BitPos = 0;     // 位位置重置为0
                  }
               }

               /* 计算地址差值，用于分段处理 */
               diff = LogAddr - oLogAddr;           // 计算当前逻辑地址与上一个地址的差值
               oLogAddr = LogAddr;                  // 更新上一个地址
               
               /* 如果当前分段加上差值超过最大分段大小，创建新分段 */
               if ((segmentsize + diff) > segmentmaxsize && diff <= segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
               {
                  context->grouplist[group].IOsegment[currentsegment++] = segmentsize;  // 保存当前分段大小
                  segmentsize = 0;                                                  // 重置分段大小
                  segmentmaxsize = EC_MAXLRWDATA;  /* 第一个分段后可以忽略 DC 开销 */
               }
               segmentsize += diff;  // 累加当前分段大小
               
               /* 如果分段大小超过最大值，继续分段 */
               while (segmentsize > segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
               {
                  context->grouplist[group].IOsegment[currentsegment++] = segmentmaxsize;  // 保存最大分段大小
                  segmentsize -= segmentmaxsize;                                         // 减去已保存的大小
                  context->grouplist[group].inputsWKC++;                                 // 增加输入工作计数器
                  segmentmaxsize = EC_MAXLRWDATA;  /* 第一个分段后可以忽略 DC 开销 */
               }
               /* 如果从站添加了输入数据且存在未完成的分段，增加输入工作计数器 */
               if (segmentsize && diff)
                  context->grouplist[group].inputsWKC++;
            }
         }
      }
      /* 处理输入映射后的位对齐 */
      if (BitPos)  // 如果存在未对齐的位
      {
         LogAddr++;         // 逻辑地址递增到下一个字节
         oLogAddr = LogAddr; // 更新上一个地址
         BitPos = 0;        // 位位置重置为0
         
         /* 检查是否需要创建新分段 */
         if ((segmentsize + 1) > segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
         {
            context->grouplist[group].IOsegment[currentsegment++] = segmentsize;  // 保存当前分段
            segmentsize = 0;                                                  // 重置分段大小
            context->grouplist[group].inputsWKC++;                              // 增加输入工作计数器
         }
         segmentsize += 1;  // 累加分段大小
      }
      /* 保存输入映射信息到组结构 */
      context->grouplist[group].IOsegment[currentsegment] = segmentsize;  // 保存最后一个分段大小
      context->grouplist[group].nsegments = currentsegment + 1;           // 更新分段数量
      context->grouplist[group].inputs = (uint8 *)(pIOmap) + context->grouplist[group].Obytes;  // 输入映射指针（在输出之后）
      context->grouplist[group].Ibytes = LogAddr -
                                         context->grouplist[group].logstartaddr -
                                         context->grouplist[group].Obytes;  // 计算输入字节数

      /* 遍历所有从站，创建邮箱状态映射并配置 FMMU */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;  // 获取从站配置地址
         
         /* 检查是否属于指定的组（group=0 表示所有组） */
         if (!group || (group == context->slavelist[slave].group))
         {
            ecx_config_create_mbxstatus_mappings(context, pIOmap, group, slave, &LogAddr);  // 创建邮箱状态映射
            diff = LogAddr - oLogAddr;           // 计算地址差值
            oLogAddr = LogAddr;                  // 更新上一个地址
            
            /* 检查是否需要创建新分段（考虑 DC 开销） */
            if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
            {
               context->grouplist[group].IOsegment[currentsegment] = segmentsize;  // 保存当前分段
               if (currentsegment < (EC_MAXIOSEGMENTS - 1))
               {
                  currentsegment++;  // 移动到下一个分段
                  segmentsize = diff;  // 新分段从差值开始
               }
            }
            else
            {
               segmentsize += diff;  // 累加当前分段大小
            }
         }
      }

      /* 保存邮箱状态映射信息到组结构 */
      context->grouplist[group].IOsegment[currentsegment] = segmentsize;  // 保存最后一个分段大小
      context->grouplist[group].nsegments = currentsegment + 1;           // 更新分段数量
      context->grouplist[group].mbxstatus = (uint8 *)(pIOmap) + context->grouplist[group].Obytes + context->grouplist[group].Ibytes;  // 邮箱状态映射指针（在输入之后）
      context->grouplist[group].mbxstatuslength = LogAddr - context->grouplist[group].Obytes - context->grouplist[group].Ibytes;  // 计算邮箱状态长度
      
      /* 如果是组0，同时保存到主站记录 */
      if (!group)
      {
         context->slavelist[0].inputs = (uint8 *)(pIOmap) + context->slavelist[0].Obytes;  // 保存输入映射指针
         context->slavelist[0].Ibytes = LogAddr -
                                        context->grouplist[group].logstartaddr -
                                        context->slavelist[0].Obytes; /* 在主站记录中保存输入字节数 */
         context->slavelist[0].mbxstatus = (uint8 *)(pIOmap) + context->slavelist[0].Obytes + context->slavelist[0].Ibytes;  // 保存邮箱状态映射指针
      }

      /* 执行映射后的操作 */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;  // 获取从站配置地址
         
         /* 检查是否属于指定的组（group=0 表示所有组） */
         if (!group || (group == context->slavelist[slave].group))
         {
            /* 将 EEPROM 控制权交给 PDI（过程数据接口） */
            ecx_eeprom2pdi(context, slave);
            
            /* 如果用户没有禁用自动状态转换 */
            if (context->manualstatechange == 0)
            {
               /* 请求从站进入安全操作状态 */
               ecx_FPWRw(&context->port,
                         configadr,
                         ECT_REG_ALCTL,              // AL 控制寄存器
                         htoes(EC_STATE_SAFE_OP),     // 安全操作状态
                         EC_TIMEOUTRET3);
            }

            /* 保存从站属性到组结构 */
            if (context->slavelist[slave].blockLRW)  // 如果从站只支持 LRD/LWR 协议
            {
               context->grouplist[group].blockLRW++;  // 增加阻塞 LRD/LWR 计数
            }
            context->grouplist[group].Ebuscurrent += context->slavelist[slave].Ebuscurrent;  // 累加 EtherCAT 总线电流
         }
      }

      /* 打印 IOmap 大小并返回 */
      EC_PRINT("IOmapSize %d\n", context->grouplist[group].Obytes +
                                     context->grouplist[group].Ibytes +
                                     context->grouplist[group].mbxstatuslength);

      /* 返回 IOmap 总大小（输出字节数 + 输入字节数 + 邮箱状态长度） */
      return (context->grouplist[group].Obytes +
              context->grouplist[group].Ibytes) +
             context->grouplist[group].mbxstatuslength;
   }

   return 0;
}

/** Map all PDOs in one group of slaves to IOmap with Outputs/Inputs
 * overlapping. NOTE: Must use this for TI ESC when using LRW.
 *
 * @param[in]  context    context struct
 * @param[out] pIOmap     pointer to IOmap
 * @param[in]  group      = group to map, 0 all groups
 * @return IOmap size
 */
static int ecx_config_overlap_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
   uint16 slave, configadr;
   uint8 BitPos;
   uint32 mLogAddr = 0;
   uint32 siLogAddr = 0;
   uint32 soLogAddr = 0;
   uint32 tempLogAddr;
   uint32 diff;
   uint16 currentsegment = 0;
   uint32 segmentsize = 0;
   uint32 segmentmaxsize = (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM);

   if ((context->slavecount > 0) && (group < EC_MAXGROUP))
   {
      EC_PRINT("ec_config_map_group IOmap:%p group:%d\n", pIOmap, group);
      mLogAddr = context->grouplist[group].logstartaddr;
      siLogAddr = mLogAddr;
      soLogAddr = mLogAddr;
      BitPos = 0;
      context->grouplist[group].nsegments = 0;
      context->grouplist[group].outputsWKC = 0;
      context->grouplist[group].inputsWKC = 0;

      /* 查找 PDO 映射并配置同步管理器 */
      ecx_config_find_mappings(context, group);

      /* do IO mapping of slave and program FMMUs */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;
         siLogAddr = soLogAddr = mLogAddr;

         if (!group || (group == context->slavelist[slave].group))
         {
            /* create output mapping */
            if (context->slavelist[slave].Obits)
            {

               ecx_config_create_output_mappings(context, pIOmap, group,
                                                 slave, &soLogAddr, &BitPos);
               if (BitPos)
               {
                  soLogAddr++;
                  BitPos = 0;
               }
            }

            /* create input mapping */
            if (context->slavelist[slave].Ibits)
            {
               ecx_config_create_input_mappings(context, pIOmap, group,
                                                slave, &siLogAddr, &BitPos);
               if (BitPos)
               {
                  siLogAddr++;
                  BitPos = 0;
               }
            }

            tempLogAddr = (siLogAddr > soLogAddr) ? siLogAddr : soLogAddr;
            diff = tempLogAddr - mLogAddr;
            int soLength = soLogAddr - mLogAddr;
            int siLength = siLogAddr - mLogAddr;
            mLogAddr = tempLogAddr;
            if ((segmentsize + diff) > segmentmaxsize && diff <= segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
            {
               context->grouplist[group].IOsegment[currentsegment++] = segmentsize;
               segmentsize = 0;
               segmentmaxsize = EC_MAXLRWDATA; /* can ignore DC overhead after first segment */
            }
            segmentsize += diff;
            while (segmentsize > segmentmaxsize && currentsegment < EC_MAXIOSEGMENTS)
            {
               context->grouplist[group].IOsegment[currentsegment++] = segmentmaxsize;
               segmentsize -= segmentmaxsize;
               context->grouplist[group].inputsWKC += (siLength > 0);
               context->grouplist[group].outputsWKC += (soLength > 0);
               siLength -= segmentmaxsize;
               soLength -= segmentmaxsize;
               segmentmaxsize = EC_MAXLRWDATA; /* can ignore DC overhead after first segment */
            }
            /* if this slave added data and there is a partial segment still outstanding increment the relevant wkc */
            if (segmentsize && diff)
            {
               context->grouplist[group].inputsWKC += (siLength > 0);
               context->grouplist[group].outputsWKC += (soLength > 0);
            }
         }
      }

      /* Update segment info */
      context->grouplist[group].IOsegment[currentsegment] = segmentsize;
      context->grouplist[group].nsegments = currentsegment + 1;
      context->grouplist[group].Isegment = 0;
      context->grouplist[group].Ioffset = 0;

      context->grouplist[group].Obytes = soLogAddr - context->grouplist[group].logstartaddr;
      context->grouplist[group].Ibytes = siLogAddr - context->grouplist[group].logstartaddr;
      context->grouplist[group].outputs = pIOmap;
      context->grouplist[group].inputs = (uint8 *)pIOmap + context->grouplist[group].Obytes;

      context->grouplist[group].mbxstatus = (uint8 *)pIOmap + context->grouplist[group].Obytes + context->grouplist[group].Ibytes;

      /* Move calculated inputs with OBytes offset */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         if (!group || (group == context->slavelist[slave].group))
         {
            if (context->slavelist[slave].Ibits > 0)
            {
               context->slavelist[slave].inputs += context->grouplist[group].Obytes;
            }
         }
      }

      /* Do mbxstatus mapping of slave and program FMMUs */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;
         if (!group || (group == context->slavelist[slave].group))
         {
            ecx_config_create_mbxstatus_mappings(context, pIOmap, group, slave, &tempLogAddr);
            diff = tempLogAddr - mLogAddr;
            mLogAddr = tempLogAddr;
            if ((segmentsize + diff) > (EC_MAXLRWDATA - EC_FIRSTDCDATAGRAM))
            {
               context->grouplist[group].IOsegment[currentsegment] = segmentsize;
               if (currentsegment < (EC_MAXIOSEGMENTS - 1))
               {
                  currentsegment++;
                  segmentsize = diff;
               }
            }
            else
            {
               segmentsize += diff;
            }
            /* Move calculated mbxstatus with OBytes + Ibytes offset */
            context->slavelist[slave].mbxstatus += context->grouplist[group].Obytes;
            context->slavelist[slave].mbxstatus += context->grouplist[group].Ibytes;
         }
      }

      /* Update segment info */
      context->grouplist[group].IOsegment[currentsegment] = segmentsize;
      context->grouplist[group].nsegments = currentsegment + 1;

      if (!group)
      {
         /* store output bytes in master record */
         context->slavelist[0].outputs = pIOmap;
         context->slavelist[0].Obytes = soLogAddr - context->grouplist[group].logstartaddr;
         context->slavelist[0].inputs = (uint8 *)pIOmap + context->slavelist[0].Obytes;
         context->slavelist[0].Ibytes = siLogAddr - context->grouplist[group].logstartaddr;
         context->slavelist[0].mbxstatus = (uint8 *)pIOmap +
                                           context->slavelist[0].Obytes +
                                           context->slavelist[0].Ibytes;
      }

      /* Do post mapping actions */
      for (slave = 1; slave <= context->slavecount; slave++)
      {
         configadr = context->slavelist[slave].configadr;
         if (!group || (group == context->slavelist[slave].group))
         {
            /* set Eeprom control to PDI */
            ecx_eeprom2pdi(context, slave);
            /* User may override automatic state change */
            if (context->manualstatechange == 0)
            {
               /* request safe_op for slave */
               ecx_FPWRw(&context->port,
                         configadr,
                         ECT_REG_ALCTL,
                         htoes(EC_STATE_SAFE_OP),
                         EC_TIMEOUTRET3);
            }

            /* Store slave properties*/
            if (context->slavelist[slave].blockLRW)
            {
               context->grouplist[group].blockLRW++;
            }
            context->grouplist[group].Ebuscurrent += context->slavelist[slave].Ebuscurrent;
         }
      }

      EC_PRINT("IOmapSize %d\n", context->grouplist[group].Obytes +
                                     context->grouplist[group].Ibytes +
                                     context->grouplist[group].mbxstatuslength);

      return (context->grouplist[group].Obytes +
              context->grouplist[group].Ibytes) +
             context->grouplist[group].mbxstatuslength;
   }

   return 0;
}

/**
 * 这个是主API
 * 将一个组的 PDO 映射到 IOmap
 * 功能：
 * - 生成 IOmap 和过程数据的数据报
 * - 尽可能使用默认布局
 * - 如果从站有特定的 SM 配置，则使用该配置
 * - 如果默认布局不可用（blockLRW），则使用 LRD 和 LWR
 * - 如果需要，生成邮箱配置的数据报
 * 
 * 执行流程：
 * 1. 检查是否为重叠模式
 * 2. 如果是重叠模式，调用 ecx_config_overlap_map_group
 * 3. 否则调用 ecx_main_config_map_group
 * 
 * IOmap 位于 pIOmap 给定的地址
 * 返回 IOmap 的总大小
 * 
 * 可以通过多次调用此函数并使用不同的组号来映射多个组
 * 不同组的 IOmap 将被连接在一起
 * 
 * 在重叠模式下，IOmap 的大小调整为输入和输出的最大值，
 * 以允许输入数据在同一缓冲区中替换输出数据。
 * 这仅在启用重叠模式时才可能。
 * 
 * @param[in]  context 上下文结构体指针
 * @param[out] pIOmap  指向 IOmap 的指针
 * @param[in]  group   要映射的组，0 = 所有组
 * @return IOmap 大小
 */
int ecx_config_map_group(ecx_contextt *context, void *pIOmap, uint8 group)
{
   if (context->overlappedMode)
   {
      return ecx_config_overlap_map_group(context, pIOmap, group);
   }
   return ecx_main_config_map_group(context, pIOmap, group);
}


/*************************************** 以下代码是处理错误信息用的 ***********************/


/** 
 * 恢复丢失的从站
 * 功能：
 * - 尝试重新连接丢失的从站
 * - 检查从站是否在正确位置
 * - 为从站重新分配临时地址
 * 
 * 执行流程：
 * 1. 检查是否找到正确的从站
 * 2. 如果找到，直接返回成功
 * 3. 如果未找到且地址为 0，尝试重新分配地址
 * 4. 设置临时节点地址
 * 5. 清除临时地址
 * 
 * @param[in] context 上下文结构体指针
 * @param[in] slave   要恢复的从站编号
 * @param[in] timeout 本地超时时间，例如 EC_TIMEOUTRET3
 * @return >0 表示成功
 */
int ecx_recover_slave(ecx_contextt *context, uint16 slave, int timeout)
{
   int rval;
   int wkc;
   uint16 ADPh, configadr, readadr;

   rval = 0;
   configadr = context->slavelist[slave].configadr;
   ADPh = (uint16)(1 - slave);
   /* check if we found another slave than the requested */
   readadr = 0xfffe;
   wkc = ecx_APRD(&context->port, ADPh, ECT_REG_STADR, sizeof(readadr), &readadr, timeout);
   /* correct slave found, finished */
   if (readadr == configadr)
   {
      return 1;
   }
   /* only try if no config address*/
   if ((wkc > 0) && (readadr == 0))
   {
      /* clear possible slaves at EC_TEMPNODE */
      ecx_FPWRw(&context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0), 0);
      /* set temporary node address of slave */
      if (ecx_APWRw(&context->port, ADPh, ECT_REG_STADR, htoes(EC_TEMPNODE), timeout) <= 0)
      {
         ecx_FPWRw(&context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0), 0);
         return 0; /* slave fails to respond */
      }

      context->slavelist[slave].configadr = EC_TEMPNODE; /* temporary config address */
      ecx_eeprom2master(context, slave);                 /* set Eeprom control to master */

      /* check if slave is the same as configured before */
      if ((ecx_FPRDw(&context->port, EC_TEMPNODE, ECT_REG_ALIAS, timeout) ==
           htoes(context->slavelist[slave].aliasadr)) &&
          (ecx_readeeprom(context, slave, ECT_SII_ID, EC_TIMEOUTEEP) ==
           htoel(context->slavelist[slave].eep_id)) &&
          (ecx_readeeprom(context, slave, ECT_SII_MANUF, EC_TIMEOUTEEP) ==
           htoel(context->slavelist[slave].eep_man)) &&
          (ecx_readeeprom(context, slave, ECT_SII_REV, EC_TIMEOUTEEP) ==
           htoel(context->slavelist[slave].eep_rev)))
      {
         rval = ecx_FPWRw(&context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(configadr), timeout);
         context->slavelist[slave].configadr = configadr;
      }
      else
      {
         /* slave is not the expected one, remove config address*/
         ecx_FPWRw(&context->port, EC_TEMPNODE, ECT_REG_STADR, htoes(0), timeout);
         context->slavelist[slave].configadr = configadr;
      }
   }

   return rval;
}

/** Reconfigure slave.
 *
 * @param[in] context context struct
 * @param[in] slave   slave to reconfigure
 * @param[in] timeout local timeout f.e. EC_TIMEOUTRET3
 * @return Slave state
 */
int ecx_reconfig_slave(ecx_contextt *context, uint16 slave, int timeout)
{
   int state, nSM, FMMUc;
   uint16 configadr;

   configadr = context->slavelist[slave].configadr;
   if (ecx_FPWRw(&context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_INIT), timeout) <= 0)
   {
      return 0;
   }
   state = 0;
   ecx_eeprom2pdi(context, slave); /* set Eeprom control to PDI */
   /* check state change init */
   state = ecx_statecheck(context, slave, EC_STATE_INIT, EC_TIMEOUTSTATE);
   if (state == EC_STATE_INIT)
   {
      /* program all enabled SM */
      for (nSM = 0; nSM < EC_MAXSM; nSM++)
      {
         if (context->slavelist[slave].SM[nSM].StartAddr)
         {
            ecx_FPWR(&context->port, configadr, (uint16)(ECT_REG_SM0 + (nSM * sizeof(ec_smt))),
                     sizeof(ec_smt), &context->slavelist[slave].SM[nSM], timeout);
         }
      }
      /* small delay to allow slave to process SM changes */
      osal_usleep(5000);
      ecx_FPWRw(&context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_PRE_OP), timeout);
      state = ecx_statecheck(context, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE); /* check state change pre-op */
      if (state == EC_STATE_PRE_OP)
      {
         if (context->ENI)
         {
            (void)ecx_mbxENIinitcmds(context, slave, ECT_ESMTRANS_PS);
         }
         /* execute slave configuration hook Pre-Op to Safe-OP */
         if (context->slavelist[slave].PO2SOconfig) /* only if registered */
         {
            context->slavelist[slave].PO2SOconfig(context, slave);
         }
         ecx_FPWRw(&context->port, configadr, ECT_REG_ALCTL, htoes(EC_STATE_SAFE_OP), timeout); /* set safeop status */
         state = ecx_statecheck(context, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);             /* check state change safe-op */
         /* program configured FMMU */
         for (FMMUc = 0; FMMUc < context->slavelist[slave].FMMUunused; FMMUc++)
         {
            ecx_FPWR(&context->port, configadr, (uint16)(ECT_REG_FMMU0 + (sizeof(ec_fmmut) * FMMUc)),
                     sizeof(ec_fmmut), &context->slavelist[slave].FMMU[FMMUc], timeout);
         }
      }
   }

   return state;
}
