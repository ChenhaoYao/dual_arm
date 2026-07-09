/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

/** \file
 * \brief
 * Distributed Clock EtherCAT functions.
 *
 */
#include "soem/soem.h"
#include "oshw.h"
#include "osal.h"

#define PORTM0    0x01
#define PORTM1    0x02
#define PORTM2    0x04
#define PORTM3    0x08

/** 1st sync pulse delay in ns here 100ms */
#define SyncDelay ((int32)100000000)

/**
 * 配置单个从站的 DC SYNC0 周期同步信号
 *
 * 功能：让从站的分布式时钟 (Distributed Clock) 在每个 CyclTime 周期产生一次
 * SYNC0 中断脉冲。SYNC0 是伺服 / IO 实现"硬实时"同步的核心信号——所有从站
 * 在同一时刻触发数据采集 / 输出，实现亚微秒级总线同步。
 *
 * 工作流程：
 *  1. 先关闭周期操作 (DCSYNCACT=0)，避免重配置时产生抖动
 *  2. 读从站本地时间 t1
 *  3. 计算首次触发时间 t = 把 t1 向上对齐到 CyclTime 整数倍 + SyncDelay (100ms 启动余量)
 *     这样所有从站不论何时被配置，第一次 SYNC0 脉冲都会在同一个全局时刻发出
 *  4. 写入 DCSTART0 (起始时间) 和 DCCYCLE0 (周期)
 *  5. 重新使能 DCSYNCACT (bit0=cyclic, bit1=SYNC0)
 *
 * 调用时机：必须在 SAFE_OP 状态下、且已调用 ecx_configdc() 完成校时之后。
 *
 * @param[in]  context     上下文结构指针
 * @param[in]  slave       从站编号 (1..slavecount)
 * @param[in]  act         TRUE=激活 SYNC0，FALSE=关闭 SYNC0
 * @param[in]  CyclTime    SYNC0 周期，单位 ns (常用 1ms=1000000)
 * @param[in]  CyclShift   相位偏移 (ns)，可正可负，用于错开各从站触发点
 */
void ecx_dcsync0(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime, int32 CyclShift)
{
   uint8 h, RA;
   uint16 slaveh;
   int64 t, t1;
   int32 tc;

   slaveh = context->slavelist[slave].configadr;
   RA = 0;

   /* stop cyclic operation, ready for next trigger */
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
   if (act)
   {
      RA = 1 + 2; /* act cyclic operation and sync0, sync1 deactivated */
   }
   h = 0;
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
   t1 = 0;
   (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
   t1 = etohll(t1);

   /* Calculate first trigger time, always a whole multiple of CyclTime rounded up
   plus the shifttime (can be negative)
   This insures best synchronization between slaves, slaves with the same CyclTime
   will sync at the same moment (you can use CyclShift to shift the sync) */
   if (CyclTime > 0)
   {
      t = ((t1 + SyncDelay) / CyclTime) * CyclTime + CyclTime + CyclShift;
   }
   else
   {
      t = t1 + SyncDelay + CyclShift;
      /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
   }
   t = htoell(t);
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
   tc = htoel(CyclTime);
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET);  /* SYNC0 cycle time */
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

   // update ec_slave state
   context->slavelist[slave].DCactive = (uint8)act;
   context->slavelist[slave].DCshift = CyclShift;
   context->slavelist[slave].DCcycle = CyclTime;
}

/**
 * 配置单个从站的 DC SYNC0 + SYNC1 双路周期同步信号
 *
 * 与 ecx_dcsync0 相比多激活一路 SYNC1。SYNC1 通常用于：
 *  - 在 SYNC0 之后再触发一个二级动作 (例如 SYNC0 启动采样、SYNC1 触发输出)
 *  - 或作为 SYNC0 的整数倍周期 (例如 SYNC0=1ms 但 SYNC1=4ms)
 *
 * SYNC1 在硬件上是相对 SYNC0 的"延迟时间"，不是独立绝对时间，所以参数名是
 * CyclTime1 (相对 SYNC0 的偏移 / 周期间隔)。
 *
 * 注意 TrueCyclTime 的计算：因为 SYNC1 可能是 SYNC0 的若干倍周期，对齐起始
 * 时间时需要用 SYNC0 与 SYNC1 的最小公共周期来对齐，避免相位漂移。
 *
 * DCSYNCACT 位含义：bit0=cyclic operation，bit1=SYNC0 enable，bit2=SYNC1 enable，
 * 所以这里激活值是 1+2+4=7。
 *
 * @param[in]  context     上下文结构指针
 * @param[in]  slave       从站编号
 * @param[in]  act         TRUE=激活，FALSE=关闭
 * @param[in]  CyclTime0   SYNC0 周期 (ns)
 * @param[in]  CyclTime1   SYNC1 相对 SYNC0 的延迟 / 周期 (ns)。
 *                         若为 0 则 SYNC1 与 SYNC0 同时触发。
 * @param[in]  CyclShift   全局相位偏移 (ns)，可正可负
 */
void ecx_dcsync01(ecx_contextt *context, uint16 slave, boolean act, uint32 CyclTime0, uint32 CyclTime1, int32 CyclShift)
{
   uint8 h, RA;
   uint16 slaveh;
   int64 t, t1;
   int32 tc;
   uint32 TrueCyclTime;

   /* Sync1 can be used as a multiple of Sync0, use true cycle time */
   TrueCyclTime = ((CyclTime1 / CyclTime0) + 1) * CyclTime0;

   slaveh = context->slavelist[slave].configadr;
   RA = 0;

   /* stop cyclic operation, ready for next trigger */
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET);
   if (act)
   {
      RA = 1 + 2 + 4; /* act cyclic operation and sync0 + sync1 */
   }
   h = 0;
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCCUC, sizeof(h), &h, EC_TIMEOUTRET); /* write access to ethercat */
   t1 = 0;
   (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCSYSTIME, sizeof(t1), &t1, EC_TIMEOUTRET); /* read local time of slave */
   t1 = etohll(t1);

   /* Calculate first trigger time, always a whole multiple of TrueCyclTime rounded up
   plus the shifttime (can be negative)
   This insures best synchronization between slaves, slaves with the same CyclTime
   will sync at the same moment (you can use CyclShift to shift the sync) */
   if (CyclTime0 > 0)
   {
      t = ((t1 + SyncDelay) / TrueCyclTime) * TrueCyclTime + TrueCyclTime + CyclShift;
   }
   else
   {
      t = t1 + SyncDelay + CyclShift;
      /* first trigger at T1 + CyclTime + SyncDelay + CyclShift in ns */
   }
   t = htoell(t);
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSTART0, sizeof(t), &t, EC_TIMEOUTRET); /* SYNC0 start time */
   tc = htoel(CyclTime0);
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCCYCLE0, sizeof(tc), &tc, EC_TIMEOUTRET); /* SYNC0 cycle time */
   tc = htoel(CyclTime1);
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCCYCLE1, sizeof(tc), &tc, EC_TIMEOUTRET);  /* SYNC1 cycle time */
   (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYNCACT, sizeof(RA), &RA, EC_TIMEOUTRET); /* activate cyclic operation */

   // update ec_slave state
   context->slavelist[slave].DCactive = (uint8)act;
   context->slavelist[slave].DCshift = CyclShift;
   context->slavelist[slave].DCcycle = CyclTime0;
}

/**
 * 获取指定从站某个物理端口的 DC 锁存接收时间戳
 *
 * 每个从站 ESC 有 4 个物理端口 (0/1/2/3)，DC 单元会锁存"参考帧到达每个端口"
 * 的本地时间，分别保存到 DCrtA/DCrtB/DCrtC/DCrtD。
 * 这些时间戳是计算传播延迟 (pdelay) 的原始数据。
 *
 * @param context 上下文
 * @param slave   从站编号
 * @param port    端口号 0..3
 * @return        该端口锁存的时间戳 (ns)，端口号越界返回 0
 */
static int32 ecx_porttime(ecx_contextt *context, uint16 slave, uint8 port)
{
   int32 ts;
   switch (port)
   {
   case 0:
      ts = context->slavelist[slave].DCrtA;
      break;
   case 1:
      ts = context->slavelist[slave].DCrtB;
      break;
   case 2:
      ts = context->slavelist[slave].DCrtC;
      break;
   case 3:
      ts = context->slavelist[slave].DCrtD;
      break;
   default:
      ts = 0;
      break;
   }
   return ts;
}

/**
 * 计算从站某端口在 EtherCAT 报文转发顺序中的"上一个激活端口"
 *
 * 背景：EtherCAT ESC 内部固定按 0 → 3 → 1 → 2 → 0 的顺序处理报文 (不是
 * 0→1→2→3)。本函数沿此顺序回退一步，跳过没接线的端口，找到上一个 active
 * 端口。用于在 ecx_configdc 中计算两端口时间戳之差，进而推导线缆传播延迟。
 *
 * 例：port=1，活动端口位图含 PORTM3，则返回 3 (因为 1 的上一个是 3)。
 *
 * @param context 上下文
 * @param slave   从站编号
 * @param port    当前端口号 0..3
 * @return        转发顺序上的上一个激活端口；若没有则原样返回 port
 */
static uint8 ecx_prevport(ecx_contextt *context, uint16 slave, uint8 port)
{
   uint8 pport = port;
   uint8 aport = context->slavelist[slave].activeports;
   switch (port)
   {
   case 0:
      if (aport & PORTM2)
         pport = 2;
      else if (aport & PORTM1)
         pport = 1;
      else if (aport & PORTM3)
         pport = 3;
      break;
   case 1:
      if (aport & PORTM3)
         pport = 3;
      else if (aport & PORTM0)
         pport = 0;
      else if (aport & PORTM2)
         pport = 2;
      break;
   case 2:
      if (aport & PORTM1)
         pport = 1;
      else if (aport & PORTM3)
         pport = 3;
      else if (aport & PORTM0)
         pport = 0;
      break;
   case 3:
      if (aport & PORTM0)
         pport = 0;
      else if (aport & PORTM2)
         pport = 2;
      else if (aport & PORTM1)
         pport = 1;
      break;
   }
   return pport;
}

/**
 * 在父从站的"未消费端口"位图中按固定顺序找一个尚未占用的端口，并占用之
 *
 * 用于 DC 拓扑构建：当一个新的子从站要挂到 parent 上时，需要确定它接到
 * parent 的哪个物理端口。搜索顺序固定为 3 → 1 → 2 → 0 (与 EtherCAT 报文转发
 * 顺序一致)。被分配的端口会从 consumedports 中清除，避免下次再被分配给
 * 其他子节点。
 *
 * @param context 上下文
 * @param parent  父从站编号
 * @return        分配给当前调用者的父端口号 (0..3)
 */
static uint8 ecx_parentport(ecx_contextt *context, uint16 parent)
{
   uint8 parentport = 0;
   uint8 b;
   /* search order is important, here 3 - 1 - 2 - 0 */
   b = context->slavelist[parent].consumedports;
   if (b & PORTM3)
   {
      parentport = 3;
      b &= (uint8)~PORTM3;
   }
   else if (b & PORTM1)
   {
      parentport = 1;
      b &= (uint8)~PORTM1;
   }
   else if (b & PORTM2)
   {
      parentport = 2;
      b &= (uint8)~PORTM2;
   }
   else if (b & PORTM0)
   {
      parentport = 0;
      b &= (uint8)~PORTM0;
   }
   context->slavelist[parent].consumedports = b;
   return parentport;
}

/**
 * 配置 EtherCAT 分布式时钟 (Distributed Clock)：定位所有支持 DC 的从站，
 * 测量每台从站的报文传播延迟，并把所有从站的本地时钟与第一台 DC 从站
 * (参考时钟) 对齐。
 *
 * 这是高精度同步的前提：完成此函数后，所有 DC 从站的本地时间被校到统一
 * 时基 (默认对齐到主机墙钟时间)，之后再用 ecx_dcsync0 / ecx_dcsync01 设置
 * 周期触发，整张总线就能做到亚微秒同步。
 *
 * 主要步骤：
 *  1. 广播 BWR 写 DCTIME0：让所有从站同时锁存"参考帧抵达 portA 的本地时间"，
 *     这是后续传播延迟测量的零点。
 *  2. 取主机当前时间 (转 EtherCAT 纪元 2000-01-01)，做为系统时间偏移基准。
 *  3. 遍历每个从站：
 *     - 读 4 个端口锁存时间 DCrtA..DCrtD，确定 entryport (时间戳最小的端口)
 *     - 写 DCSYSOFFSET：把从站本地时钟的零点搬到主机时间附近
 *     - 沿拓扑追溯第一个支持 DC 的祖先 (parent)，作为时延参考点
 *     - 用 entryport 时间差 - 子节点链路延迟 - 兄弟节点延迟，算出本从站的
 *       pdelay (propagation delay，含父节点累积)
 *     - 写 DCSYSDELAY 通知从站做"提前/延后"补偿
 *  4. 维护 DCnext / DCprevious 链表，便于后续按 DC 顺序遍历从站。
 *
 * 调用时机：必须在 SAFE_OP 状态调用，OP 状态下 DC 已在跑，重新配置可能造成
 * 同步信号抖动。
 *
 * @param[in] context  上下文结构指针
 * @return TRUE=至少有一台 DC 从站；FALSE=总线上没有 DC 能力的从站
 */
boolean ecx_configdc(ecx_contextt *context)
{
   uint16 i, slaveh, parent, child;
   uint16 parenthold = 0;
   uint16 prevDCslave = 0;
   int32 ht, dt1, dt2, dt3;
   int64 hrt;
   uint8 entryport;
   int8 nlist;
   int8 plist[4];
   int32 tlist[4];
   ec_timet mastertime;
   uint64 mastertime64;

   context->slavelist[0].hasdc = FALSE;
   context->grouplist[0].hasdc = FALSE;
   ht = 0;

   ecx_BWR(&context->port, 0, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET); /* latch DCrecvTimeA of all slaves */
   mastertime = osal_current_time();
   mastertime.tv_sec -= 946684800UL; /* EtherCAT uses 2000-01-01 as epoch start instead of 1970-01-01 */
   mastertime64 = ((uint64)mastertime.tv_sec * 1000 * 1000 * 1000) + (uint64)mastertime.tv_nsec;
   for (i = 1; i <= context->slavecount; i++)
   {
      context->slavelist[i].consumedports = context->slavelist[i].activeports;
      if (context->slavelist[i].hasdc)
      {
         if (!context->slavelist[0].hasdc)
         {
            context->slavelist[0].hasdc = TRUE;
            context->slavelist[0].DCnext = i;
            context->slavelist[i].DCprevious = 0;
            context->grouplist[context->slavelist[i].group].hasdc = TRUE;
            context->grouplist[context->slavelist[i].group].DCnext = i;
         }
         else
         {
            context->slavelist[prevDCslave].DCnext = i;
            context->slavelist[i].DCprevious = prevDCslave;
         }
         /* this branch has DC slave so remove parenthold */
         parenthold = 0;
         prevDCslave = i;
         slaveh = context->slavelist[i].configadr;
         (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCTIME0, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtA = etohl(ht);
         /* 64bit latched DCrecvTimeA of each specific slave */
         (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCSOF, sizeof(hrt), &hrt, EC_TIMEOUTRET);
         /* use it as offset in order to set local time around 0 + mastertime */
         hrt = htoell(-etohll(hrt) + mastertime64);
         /* save it in the offset register */
         (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYSOFFSET, sizeof(hrt), &hrt, EC_TIMEOUTRET);
         (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCTIME1, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtB = etohl(ht);
         (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCTIME2, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtC = etohl(ht);
         (void)ecx_FPRD(&context->port, slaveh, ECT_REG_DCTIME3, sizeof(ht), &ht, EC_TIMEOUTRET);
         context->slavelist[i].DCrtD = etohl(ht);

         /* make list of active ports and their time stamps */
         nlist = 0;
         if (context->slavelist[i].activeports & PORTM0)
         {
            plist[nlist] = 0;
            tlist[nlist] = context->slavelist[i].DCrtA;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM3)
         {
            plist[nlist] = 3;
            tlist[nlist] = context->slavelist[i].DCrtD;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM1)
         {
            plist[nlist] = 1;
            tlist[nlist] = context->slavelist[i].DCrtB;
            nlist++;
         }
         if (context->slavelist[i].activeports & PORTM2)
         {
            plist[nlist] = 2;
            tlist[nlist] = context->slavelist[i].DCrtC;
            nlist++;
         }
         /* entryport is port with the lowest timestamp */
         entryport = 0;
         if ((nlist > 1) && (tlist[1] < tlist[entryport]))
         {
            entryport = 1;
         }
         if ((nlist > 2) && (tlist[2] < tlist[entryport]))
         {
            entryport = 2;
         }
         if ((nlist > 3) && (tlist[3] < tlist[entryport]))
         {
            entryport = 3;
         }
         entryport = plist[entryport];
         context->slavelist[i].entryport = entryport;
         /* consume entryport from activeports */
         context->slavelist[i].consumedports &= (uint8) ~(1 << entryport);

         /* finding DC parent of current */
         parent = i;
         do
         {
            child = parent;
            parent = context->slavelist[parent].parent;
         } while (!((parent == 0) || (context->slavelist[parent].hasdc)));
         /* only calculate propagation delay if slave is not the first */
         if (parent > 0)
         {
            /* find port on parent this slave is connected to */
            context->slavelist[i].parentport = ecx_parentport(context, parent);
            if (context->slavelist[parent].topology == 1)
            {
               context->slavelist[i].parentport = context->slavelist[parent].entryport;
            }

            dt1 = 0;
            dt2 = 0;
            /* delta time of (parentport - 1) - parentport */
            /* note: order of ports is 0 - 3 - 1 -2 */
            /* non active ports are skipped */
            dt3 = ecx_porttime(context, parent, context->slavelist[i].parentport) -
                  ecx_porttime(context, parent,
                               ecx_prevport(context, parent, context->slavelist[i].parentport));
            /* current slave has children */
            /* those children's delays need to be subtracted */
            if (context->slavelist[i].topology > 1)
            {
               dt1 = ecx_porttime(context, i,
                                  ecx_prevport(context, i, context->slavelist[i].entryport)) -
                     ecx_porttime(context, i, context->slavelist[i].entryport);
            }
            /* we are only interested in positive difference */
            if (dt1 > dt3) dt1 = -dt1;
            /* current slave is not the first child of parent */
            /* previous child's delays need to be added */
            if ((child - parent) > 1)
            {
               dt2 = ecx_porttime(context, parent,
                                  ecx_prevport(context, parent, context->slavelist[i].parentport)) -
                     ecx_porttime(context, parent, context->slavelist[parent].entryport);
            }
            if (dt2 < 0) dt2 = -dt2;

            /* calculate current slave delay from delta times */
            /* assumption : forward delay equals return delay */
            context->slavelist[i].pdelay = ((dt3 - dt1) / 2) + dt2 +
                                           context->slavelist[parent].pdelay;
            ht = htoel(context->slavelist[i].pdelay);
            /* write propagation delay*/
            (void)ecx_FPWR(&context->port, slaveh, ECT_REG_DCSYSDELAY, sizeof(ht), &ht, EC_TIMEOUTRET);
         }
      }
      else
      {
         context->slavelist[i].DCrtA = 0;
         context->slavelist[i].DCrtB = 0;
         context->slavelist[i].DCrtC = 0;
         context->slavelist[i].DCrtD = 0;
         parent = context->slavelist[i].parent;
         /* if non DC slave found on first position on branch hold root parent */
         if ((parent > 0) && (context->slavelist[parent].topology > 2))
            parenthold = parent;
         /* if branch has no DC slaves consume port on root parent */
         if (parenthold && (context->slavelist[i].topology == 1))
         {
            ecx_parentport(context, parenthold);
            parenthold = 0;
         }
      }
   }

   return context->slavelist[0].hasdc;
}
