/*
 * 本软件采用 GPLv3 和商业许可证双重授权
 * 有关完整的许可证信息，请参阅随软件分发的 LICENSE.md 文件
 *
 * ec_sample —— SOEM 主站最简流程：让 YH052 关节模组在 PP 模式下转 30°
 *
 * 关键流程（按调用先后）：
 *   1. setup_log()           创建 log/ec_sample_YYYYMMDD_HHMMSS.log，stdout 同步落盘
 *   2. ecx_init / config_init     扫描从站
 *   3. PO2SOconfig (PRE-OP)  重建 RxPDO/TxPDO 映射 + 写 vel/acc 参数
 *   4. ecx_config_map_group  生成 IOmap，从站进入 SAFE-OP
 *   5. ecx_configdc          配 DC + 启用 SYNC0
 *   6. outputs[] 预填 CW=0x06 / Mode=PP
 *   7. RT 线程开始周期性 PDO 收发
 *   8. 主站状态切到 OP
 *   9. 主循环按 CiA402 推进：Fault → SOD → RTSO → SO → Op enabled
 *      → PP 触发 (bit4 上升沿) → 等待 Target reached → 结束
 *  10. 退回 SAFE-OP / INIT，关闭日志
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "soem/soem.h"

/* ==========================================================================
 * 用户可调参数
 * ========================================================================== */
#define ENC_BITS         19      /* 电机编码器位数 */
#define GEAR_RATIO       100     /* 减速比 */
#define TARGET_DEG       -80.00      /* 输出端目标角度（°） */
#define MOVE_DURATION_S  10       /* 期望走完时长（s）→ 决定速度/加速度 */
#define MOTION_SLAVE_FIRST 5
#define MAX_MOTION_AXES 1
#define CYCLE_NS         1000000 /* RT 线程 PDO 周期：1 ms */
#define LOG_DIV          100      /* 主循环每 N 周期打印一次（≈200ms） */
#define MAX_CYCLES       20000   /* 主循环上限（≈40s） */
#define PP_TARGET_PRELOAD_CYCLES 20

/* 由上面派生（不再单独宏） */
#define TARGET_COUNTS \
   ((int32_t)(((int64_t)1 << ENC_BITS) * GEAR_RATIO * TARGET_DEG / 360))
#define TARGET_COUNTS_ABS \
   ((uint32_t)((TARGET_COUNTS < 0) ? -(int64_t)TARGET_COUNTS : (int64_t)TARGET_COUNTS))
#define PROFILE_VEL  ((uint32_t)((TARGET_COUNTS_ABS / MOVE_DURATION_S) > 0 ? (TARGET_COUNTS_ABS / MOVE_DURATION_S) : 1))
#define PROFILE_ACC  PROFILE_VEL    /* 1 秒达目标速度 */
#define TARGET_TOLERANCE_COUNTS 500

/* RxPDO (0x1600) 7 字节：CW(2) + TargetPos(4) + Mode(1) */
#define RX_CW         0
#define RX_TARGET     2
#define RX_MODE       6
/* TxPDO (0x1A00) 15 字节：SW(2) + ActualPos(4) + ActualVel(4) + ActualTorq(2) + Err(2) + ModeDisp(1) */
#define TX_SW         0
#define TX_POS        2
#define TX_VEL        6
#define TX_TORQ       10
#define TX_ERR        12
#define TX_MODEDISP   14

#define MODE_PP       1

typedef struct
{
   uint16_t slave;
   int     enabled_logged;
   int     reached_logged;
   int     pp_phase;
   int     tick_header_printed;
   int     motion_seen;
   int     fr_low_cnt;
   int     preload_cnt;
   int32_t base_pos;
   int32_t final_target;
   uint16_t prev_sw;
   uint16_t prev_err;
} axis_state_t;

/* ==========================================================================
 * 全局
 * ========================================================================== */
static ecx_contextt        ctx;
static uint8                IOmap[4096];
static OSAL_THREAD_HANDLE   threadrt;
static volatile int         dorun = 0;     /* RT 线程是否进行 PDO 交换 */
static volatile int         mappingdone = 0;
static int                  expectedWKC = 0;
static int                  wkc = 0;
static int                  cycle = 0;
static int                  motion_axis_count = 0;
static axis_state_t         g_axes[MAX_MOTION_AXES];
static volatile int         run_axes = 0;
static int                  sync_hold_rt = 0;

static void axis_step(axis_state_t *axis, int sync_trigger, int idx, int loop_i);

/* ==========================================================================
 * 日志：stdout 通过 tee 同时写到 log/ec_sample_YYYYMMDD_HHMMSS.log
 * ========================================================================== */
static void setup_log(void)
{
   mkdir("log", 0755);   /* 已存在则忽略 */
   time_t now = time(NULL);
   struct tm tmv;
   localtime_r(&now, &tmv);
   char path[128];
   strftime(path, sizeof(path), "log/ec_sample_%Y%m%d_%H%M%S.log", &tmv);

   char cmd[256];
   snprintf(cmd, sizeof(cmd), "tee %s", path);
   FILE *p = popen(cmd, "w");
   if (!p) { fprintf(stderr, "popen(tee) 失败\n"); return; }
   if (dup2(fileno(p), STDOUT_FILENO) < 0) { fprintf(stderr, "dup2 失败\n"); return; }
   setvbuf(stdout, NULL, _IOLBF, 0);
   printf("[LOG] 输出文件: %s\n", path);
}

/* ==========================================================================
 * SDO 辅助：写 PDO 映射、设 SM 分配
 * ========================================================================== */
static int pdo_map_set(uint16 slave, uint16 idx, const uint32 *entries, uint8 n)
{
   uint8 zero = 0;
   int wk = ecx_SDOwrite(&ctx, slave, idx, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
   if (wk <= 0) return wk;
   for (uint8 i = 0; i < n; i++)
   {
      uint32 v = entries[i];
      wk = ecx_SDOwrite(&ctx, slave, idx, (uint8)(i + 1), FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
      if (wk <= 0) return wk;
   }
   return ecx_SDOwrite(&ctx, slave, idx, 0x00, FALSE, sizeof(n), &n, EC_TIMEOUTRXM);
}

static int sm_assign_set(uint16 slave, uint16 sm_idx, uint16 pdo_idx)
{
   uint8 zero = 0, one = 1;
   int wk;
   wk = ecx_SDOwrite(&ctx, slave, sm_idx, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
   if (wk <= 0) return wk;
   wk = ecx_SDOwrite(&ctx, slave, sm_idx, 0x01, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
   if (wk <= 0) return wk;
   return ecx_SDOwrite(&ctx, slave, sm_idx, 0x00, FALSE, sizeof(one), &one, EC_TIMEOUTRXM);
}

/* PRE-OP 阶段重建 PDO 映射 + 写运动学参数 */
static int my_po2so_config(ecx_contextt *unused, uint16 slave)
{
   (void)unused;
   uint32 rx[] = { 0x60400010, 0x607A0020, 0x60600008 };
   uint32 tx[] = { 0x60410010, 0x60640020, 0x606C0020, 0x60770010, 0x603F0010, 0x60610008 };

   if (pdo_map_set(slave, 0x1600, rx, 3) <= 0) { printf("[PRE-OP] slave%d RxPDO 映射失败\n", slave); return 0; }
   if (pdo_map_set(slave, 0x1A00, tx, 6) <= 0) { printf("[PRE-OP] slave%d TxPDO 映射失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C12, 0x1600) <= 0) { printf("[PRE-OP] slave%d SM2 分配失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C13, 0x1A00) <= 0) { printf("[PRE-OP] slave%d SM3 分配失败\n", slave); return 0; }

   if (slave >= MOTION_SLAVE_FIRST && slave < MOTION_SLAVE_FIRST + MAX_MOTION_AXES)
   {
      uint32 vel = PROFILE_VEL, acc = PROFILE_ACC;
      ecx_SDOwrite(&ctx, slave, 0x6081, 0x00, FALSE, sizeof(vel), &vel, EC_TIMEOUTRXM); // 最大速度
      ecx_SDOwrite(&ctx, slave, 0x6083, 0x00, FALSE, sizeof(acc), &acc, EC_TIMEOUTRXM); // 加速度
      ecx_SDOwrite(&ctx, slave, 0x6084, 0x00, FALSE, sizeof(acc), &acc, EC_TIMEOUTRXM); // 减速度
      printf("[PRE-OP] slave%d PDO 映射 OK，target=%d counts (%.2f°), vel=%u, acc=%u\n",
             slave, TARGET_COUNTS, TARGET_DEG, vel, acc);
   }
   return 1;
}

/* ==========================================================================
 * RT 线程：周期性 PDO 收发
 * ========================================================================== */
static void add_time_ns(ec_timet *ts, int64 add)
{
   ec_timet a;
   a.tv_nsec = add % 1000000000LL;
   a.tv_sec  = (add - a.tv_nsec) / 1000000000LL;
   osal_timespecadd(ts, &a, ts);
}

OSAL_THREAD_FUNC_RT ecatthread(void)
{
   ec_timet ts;
   while (!mappingdone) osal_usleep(1000);
   osal_get_monotonic_time(&ts);
   ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;   /* 对齐到下一毫秒 */
   ecx_send_processdata(&ctx);
   while (1)
   {
      add_time_ns(&ts, CYCLE_NS);
      osal_monotonic_sleep(&ts);
      if (dorun)
      {
         cycle++;
         wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
         ecx_mbxhandler(&ctx, 0, 4);

         if (run_axes)
         {
            int all_ready = 1;
            int sync_trigger = 0;

            for (int a = 0; a < motion_axis_count; a++)
               if (!g_axes[a].enabled_logged || g_axes[a].preload_cnt < PP_TARGET_PRELOAD_CYCLES)
               {
                  all_ready = 0;
                  break;
               }

            if (!all_ready) sync_hold_rt = 0;
            else if (sync_hold_rt < 5) sync_hold_rt++;
            else if (sync_hold_rt == 5)
            {
               sync_trigger = 1;
               sync_hold_rt++;
            }

            for (int a = 0; a < motion_axis_count; a++)
               axis_step(&g_axes[a], sync_trigger, a, cycle);
         }

         ecx_send_processdata(&ctx);
      }
   }
}

/* ==========================================================================
 * 主流程
 * ========================================================================== */
static const char *cia402_state_name(uint16_t sw)
{
   if (sw & 0x0008)              return "Fault";
   if ((sw & 0x004F) == 0x0040)  return "Switch on disabled";
   if ((sw & 0x006F) == 0x0021)  return "Ready to switch on";
   if ((sw & 0x006F) == 0x0023)  return "Switched on";
   if ((sw & 0x006F) == 0x0027)  return "Operation enabled";
   if ((sw & 0x006F) == 0x0007)  return "Quick stop active";
   return "?";
}

/* 阶段分隔线 */
static void log_sep(const char *title)
{
   printf("\n─── %-54s ───\n", title);
}

static void axis_step(axis_state_t *axis, int sync_trigger, int idx, int loop_i)
{
   (void)idx;
   uint8_t *out = ctx.slavelist[axis->slave].outputs;
   uint8_t *in  = ctx.slavelist[axis->slave].inputs;

   uint16_t sw = (uint16_t)in[TX_SW] | ((uint16_t)in[TX_SW + 1] << 8);
   int32_t  pos = (int32_t)((uint32_t)in[TX_POS]
                  | ((uint32_t)in[TX_POS + 1] << 8)
                  | ((uint32_t)in[TX_POS + 2] << 16)
                  | ((uint32_t)in[TX_POS + 3] << 24));
   int32_t  vel = (int32_t)((uint32_t)in[TX_VEL]
                  | ((uint32_t)in[TX_VEL + 1] << 8)
                  | ((uint32_t)in[TX_VEL + 2] << 16)
                  | ((uint32_t)in[TX_VEL + 3] << 24));
   int16_t  torq = (int16_t)((uint16_t)in[TX_TORQ] | ((uint16_t)in[TX_TORQ + 1] << 8));
   uint16_t er = (uint16_t)in[TX_ERR] | ((uint16_t)in[TX_ERR + 1] << 8);
   int8_t   mode_disp = (int8_t)in[TX_MODEDISP];
   uint16_t cw = 0;
   int32_t  tgt = pos;

   if (sw & 0x0008) // 自动处理fault
   {
      if (axis->fr_low_cnt < 10) { cw = 0x0000; axis->fr_low_cnt++; }
      else                       { cw = 0x0080; axis->fr_low_cnt = 0; }
   }
   else if ((sw & 0x004F) == 0x0040) cw = 0x0006;
   else if ((sw & 0x006F) == 0x0021) cw = 0x0007;
   else if ((sw & 0x006F) == 0x0023) cw = 0x000F;
   else if ((sw & 0x006F) == 0x0027)
   {
      int32_t err;

      if (!axis->enabled_logged)
      {
         axis->enabled_logged = 1;
         axis->base_pos = pos;
         axis->final_target = pos + TARGET_COUNTS;
         log_sep("PP MOTION");
         printf("  cycle=%d  Op Enabled, base=%d -> final=%d (%.2f°)\n",
                cycle, axis->base_pos, axis->final_target, TARGET_DEG);
      }

      tgt = axis->final_target;
      err = pos - axis->final_target;

      if (axis->pp_phase == 0)
      {
         cw = 0x000F;
         if (axis->preload_cnt < PP_TARGET_PRELOAD_CYCLES)
         {
            axis->preload_cnt++;
         }
         else if (sync_trigger)
         {
             cw = 0x003F;
             axis->pp_phase = 1;
             printf("  cycle=%d  PP trigger -> %d counts (%.2f°)\n",
                    cycle, tgt, TARGET_DEG);
         }
      }
      else if (axis->pp_phase == 1)
      {
         cw = 0x003F;
         if (!(sw & 0x0400)) axis->motion_seen = 1;
         if (sw & 0x1000)
         {
            axis->pp_phase = 2;
            printf("  cycle=%d  Set-point ack, SW=0x%04X pos=%d err=%d\n",
                   cycle, sw, pos, err);
         }
      }
      else if (axis->pp_phase == 2)
      {
         cw = 0x000F;
         if (!(sw & 0x0400)) axis->motion_seen = 1;
         if (!(sw & 0x1000))
         {
            axis->pp_phase = axis->motion_seen ? 4 : 3;
            printf("  cycle=%d  Set-point ack released, SW=0x%04X pos=%d err=%d\n",
                   cycle, sw, pos, err);
         }
      }
      else if (axis->pp_phase == 3)
      {
         cw = 0x000F;
         if (!(sw & 0x0400))
         {
            axis->motion_seen = 1;
            axis->pp_phase = 4;
            printf("  cycle=%d  Motion active, SW=0x%04X pos=%d err=%d\n",
                   cycle, sw, pos, err);
         }
      }
      else
      {
         cw = 0x000F;
         if ((sw & 0x0400) && llabs((long long)err) <= TARGET_TOLERANCE_COUNTS && !axis->reached_logged)
         {
            axis->reached_logged = 1;
            printf("  cycle=%d  Target Reached, pos=%d (err=%d)\n",
                   cycle, pos, pos - axis->final_target);
         }
      }
   }
   else cw = 0x0006;

   if (sw != axis->prev_sw || er != axis->prev_err)
   {
      printf("  %-6d  SW 0x%04X->0x%04X  %-24s  CW=0x%04X  Err=0x%04X  Mode=%d\n",
             cycle, axis->prev_sw, sw, cia402_state_name(sw), cw, er, mode_disp);
      axis->prev_sw = sw; axis->prev_err = er;
   }

   if ((loop_i % LOG_DIV) == 0)
   {
      if (!axis->tick_header_printed)
      {
         axis->tick_header_printed = 1;
         printf("\n  %-7s %-17s %-12s %-12s %-7s\n",
                "cycle", "pos", "vel", "torq", "phase");
         printf("  %-7s %-17s %-12s %-12s %-7s\n",
                "------", "----------------", "-----------", "-----------", "------");
      }
      printf("  %-7d %-17d %-12d %-12d %-7d\n",
             cycle, pos, vel, torq, axis->pp_phase);
   }

   out[RX_CW    ] = (uint8_t)(cw  & 0xFF);
   out[RX_CW + 1] = (uint8_t)((cw >> 8) & 0xFF);
   out[RX_TARGET    ] = (uint8_t)(tgt        & 0xFF);
   out[RX_TARGET + 1] = (uint8_t)((tgt >>  8) & 0xFF);
   out[RX_TARGET + 2] = (uint8_t)((tgt >> 16) & 0xFF);
   out[RX_TARGET + 3] = (uint8_t)((tgt >> 24) & 0xFF);
   out[RX_MODE      ] = MODE_PP;
}

static void run_motion(void)
{
   memset(g_axes, 0, sizeof(g_axes));
   for (int a = 0; a < motion_axis_count; a++)
   {
      g_axes[a].slave = MOTION_SLAVE_FIRST + a;
      g_axes[a].prev_sw = 0xFFFF;
      g_axes[a].prev_err = 0xFFFF;
   }
   sync_hold_rt = 0;
   run_axes = 1;

   for (int i = 0; i < MAX_CYCLES; i++)
   {
      int all_reached = 1;

      for (int a = 0; a < motion_axis_count; a++)
      {
         if (!g_axes[a].reached_logged) all_reached = 0;
      }

      /* 到位后再多跑 1 秒就退出 */
      if (all_reached)
      {
         osal_usleep(1000000);
         break;
      }

      osal_usleep(1000);
   }

   run_axes = 0;
}

static void ecatbringup(const char *ifname)
{
   printf("[BOOT] EtherCAT init on %s\n", ifname);
   if (!ecx_init(&ctx, ifname))            { printf("[BOOT] ecx_init 失败\n"); return; }
   if (ecx_config_init(&ctx) <= 0)         { printf("[BOOT] 没扫到从站\n");   return; }

   for (int s = 1; s <= ctx.slavecount; s++)
      ctx.slavelist[s].PO2SOconfig = my_po2so_config;
   printf("[BOOT] 找到 %d 个从站，注册 PO2SOconfig 完成\n", ctx.slavecount);
   motion_axis_count = ctx.slavecount - MOTION_SLAVE_FIRST + 1;
   if (motion_axis_count > MAX_MOTION_AXES) motion_axis_count = MAX_MOTION_AXES;
   if (motion_axis_count <= 0)
   {
      printf("[BOOT] 未检测到可控制电机，slavecount=%d\n", ctx.slavecount);
      return;
   }
   printf("[BOOT] 本次将同步控制 %d 个电机 slave%d..slave%d\n",
          motion_axis_count, MOTION_SLAVE_FIRST, MOTION_SLAVE_FIRST + motion_axis_count - 1);

   ec_groupt *grp = &ctx.grouplist[0];
   ecx_config_map_group(&ctx, IOmap, 0);
   expectedWKC = (grp->outputsWKC * 2) + grp->inputsWKC;
   printf("[BOOT] PDO 映射完成，expectedWKC=%d，IO segment[0]=%d\n",
          expectedWKC, grp->IOsegment[0]);

   mappingdone = 1;
   ecx_configdc(&ctx);
   for (int s = 1; s <= ctx.slavecount; s++)
      if (ctx.slavelist[s].hasdc)
      {
         ecx_dcsync0(&ctx, s, TRUE, CYCLE_NS, 0);
         printf("[BOOT] slave%d DC Sync0 enabled, period=%dns\n", s, CYCLE_NS);
      }

   /* 把 CoE 从站接入循环邮箱处理（SDO 仍可使用） */
   for (int s = 1; s <= ctx.slavecount; s++)
      if (ctx.slavelist[s].CoEdetails > 0)
         ecx_slavembxcyclic(&ctx, s);

   /* 预填 outputs[]：避免 RT 线程上线第一帧把 CW=0、Mode=0 写下去 */
   for (int s = MOTION_SLAVE_FIRST; s < MOTION_SLAVE_FIRST + motion_axis_count; s++)
   {
      if (ctx.slavelist[s].outputs)
      {
         uint8_t *o = ctx.slavelist[s].outputs;
         o[RX_CW] = 0x06; o[RX_CW + 1] = 0;
         memset(o + RX_TARGET, 0, 4);
         o[RX_MODE] = MODE_PP;
      }
   }

   dorun = 1;
   osal_usleep(500000);   /* 等 RT 线程稳定 PDO 0.5s */

   ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

   if (ctx.slavelist[0].state != EC_STATE_OPERATIONAL)
   {
      ecx_readstate(&ctx);
      for (int s = 1; s <= ctx.slavecount; s++)
         if (ctx.slavelist[s].state != EC_STATE_OPERATIONAL)
            printf("[BOOT] slave%d 未进入 OP, state=0x%02X AL=0x%04X (%s)\n",
                   s, ctx.slavelist[s].state, ctx.slavelist[s].ALstatuscode,
                   ec_ALstatuscode2string(ctx.slavelist[s].ALstatuscode));
      goto teardown;
   }

   printf("[BOOT] 进入 OP，开始 CiA402 状态机推进\n");
   log_sep("STATE MACHINE");
   run_motion();
   log_sep("TEARDOWN");
   printf("[DONE] 主循环结束\n");

teardown:
   dorun = 0;
   ctx.slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
   ctx.slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(&ctx, 0);
   ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
   printf("[EXIT] 已回到 INIT\n");
}

int main(int argc, char *argv[])
{
   setup_log();
   printf("\n═══════════════════════════════════════════════════════════\n");
   printf("  SOEM ec_sample — PP 模式  |  目标 %.2f°\n", TARGET_DEG);
   printf("═══════════════════════════════════════════════════════════\n");
   log_sep("BOOT");

   if (argc < 2)
   {
      printf("Usage: %s ifname (例如 enp0s31f6)\n\nAvailable adapters:\n", argv[0]);
      ec_adaptert *h = ec_find_adapters(), *a = h;
      while (a) { printf("  - %s (%s)\n", a->name, a->desc); a = a->next; }
      ec_free_adapters(h);
      return 1;
   }

   osal_thread_create_rt(&threadrt, 128000, &ecatthread, NULL);
   ecatbringup(argv[1]);
   return 0;
}
