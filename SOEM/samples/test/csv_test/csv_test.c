/*
 * csv_test — SOEM CSV (Cyclic Synchronous Velocity) 模式电机控制
 *
 * 使用 CiA402 CSV 模式 (0x6060 = 9) 通过周期性速度命令控制电机。
 *
 * CSV 模式特点：
 *   - 主站每周期下发目标速度 (0x60FF)，驱动器内部执行速度环
 *   - 位置环在驱动器内部闭合，主站可读取实际位置用于监控
 *   - RxPDO：CW(2) + TargetVelocity(4) + VelocityOffset(2) + Mode(1)
 *   - TxPDO：SW(2) + ActualPos(4) + ActualVel(4) + ActualTorque(2) + Err(2) + ModeDisp(1)
 *
 * 运动策略：
 *   1. 使能后等待 100ms
 *   2. smoothstep 斜坡把目标速度从 0 升到 TARGET_VEL
 *   3. 保持 HOLD_S 秒
 *   4. smoothstep 斜坡降到 0
 *   5. 等待实际速度回零后退出
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "soem/soem.h"

/* ==========================================================================
 * 用户可调参数
 * ========================================================================== */
#define TARGET_VEL          5242880  /* 目标速度 (counts/s), 19bit编码器100减速比 → ≈0.1 rev/s 输出 */
#define RAMP_S                  2.0  /* 速度斜坡上升/下降时长 (s) */
#define HOLD_S                  3.0  /* 速度保持时长 (s) */
#define CYCLE_NS            1000000  /* RT 线程 PDO 周期：1 ms */
#define MOTION_SLAVE_FIRST       1
#define MAX_MOTION_AXES          1
#define LOG_DIV                 100   /* 主循环每 N 周期打印一次 (≈100ms) */
#define MAX_CYCLES            20000  /* 主循环上限 (≈20s) */
#define VEL_TOLERANCE          1000  /* 速度回零判定容差 (counts/s) */
#define MODE_CSV                 9   /* CiA402 CSV 模式值 */
#define ENABLE_WAIT_MS         100   /* 使能后等待时间 (ms) */

/* 派生量 */
#define RAMP_CYCLES    ((int)((RAMP_S * 1000000000LL) / CYCLE_NS))
#define HOLD_CYCLES    ((int)((HOLD_S * 1000000000LL) / CYCLE_NS))
#define TOTAL_MOTION_CYCLES (2 * RAMP_CYCLES + HOLD_CYCLES)
#define ENABLE_WAIT_CYCLES  (ENABLE_WAIT_MS)

/* RxPDO 0x1600 (11B): CW(2) + TargetVelocity(4) + VelocityOffset(4) + Mode(1) */
#define RX_CW           0
#define RX_TARGET_VEL   2
#define RX_VEL_OFFSET   6
#define RX_MODE         10

/* TxPDO 0x1A00 (15B): SW(2) + ActualPos(4) + ActualVel(4) + ActualTorque(2) + Err(2) + ModeDisp(1) */
#define TX_SW           0
#define TX_POS          2
#define TX_VEL          6
#define TX_TORQ         10
#define TX_ERR          12
#define TX_MODEDISP     14

typedef struct
{
   uint16_t slave;
   int      enabled_logged;
   int      enable_wait_cnt;
   int      sync_hold_cnt;
   int      finished_logged;
   int      csv_started;
   int      tick_header_printed;
   int      sync_cycle;
   int      fault_reset_cnt;
   uint16_t prev_sw;
   uint16_t prev_err;
} axis_state_t;

/* ==========================================================================
 * 全局
 * ========================================================================== */
static ecx_contextt        ctx;
static uint8               IOmap[4096];
static OSAL_THREAD_HANDLE  threadrt;
static volatile int        dorun = 0;
static volatile int        mappingdone = 0;
static int                 expectedWKC = 0;
static int                 wkc = 0;
static int                 cycle = 0;
static int                 motion_axis_count = 0;
static uint16              motion_slaves[MAX_MOTION_AXES];
static axis_state_t        g_axes[MAX_MOTION_AXES];
static volatile int        run_axes = 0;

static void axis_step(axis_state_t *axis);

/* ==========================================================================
 * 日志
 * ========================================================================== */
static void setup_log(void)
{
   mkdir("log", 0755);
   time_t now = time(NULL);
   struct tm tmv;
   localtime_r(&now, &tmv);
   char path[128];
   strftime(path, sizeof(path), "log/csv_test_%Y%m%d_%H%M%S.log", &tmv);

   char cmd[256];
   snprintf(cmd, sizeof(cmd), "tee %s", path);
   FILE *p = popen(cmd, "w");
   if (!p) { fprintf(stderr, "popen(tee) 失败\n"); return; }
   if (dup2(fileno(p), STDOUT_FILENO) < 0) { fprintf(stderr, "dup2 失败\n"); return; }
   setvbuf(stdout, NULL, _IOLBF, 0);
   printf("[LOG] 输出文件: %s\n", path);
}

/* ==========================================================================
 * SDO 辅助
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

static int32_t read_i32_from_pdo(const uint8_t *p)
{
   return (int32_t)((uint32_t)p[0]
                  | ((uint32_t)p[1] << 8)
                  | ((uint32_t)p[2] << 16)
                  | ((uint32_t)p[3] << 24));
}

static void write_u16_to_pdo(uint8_t *p, uint16_t v)
{
   p[0] = (uint8_t)(v & 0xFF);
   p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void write_i32_to_pdo(uint8_t *p, int32_t v)
{
   p[0] = (uint8_t)(v & 0xFF);
   p[1] = (uint8_t)((v >> 8) & 0xFF);
   p[2] = (uint8_t)((v >> 16) & 0xFF);
   p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static int find_motion_axis(uint16 slave)
{
   for (int a = 0; a < motion_axis_count; a++)
      if (motion_slaves[a] == slave) return a;
   return -1;
}

static void discover_motion_axes(void)
{
   motion_axis_count = 0;
   memset(motion_slaves, 0, sizeof(motion_slaves));

   for (int s = MOTION_SLAVE_FIRST; s <= ctx.slavecount && motion_axis_count < MAX_MOTION_AXES; s++)
   {
      ec_slavet *sl = &ctx.slavelist[s];

      if (strcmp(sl->name, "DC Servo Drive") != 0)
      {
         printf("[DISCOVER] slave%d %s 非电机，跳过\n", s, sl->name);
         continue;
      }

      motion_slaves[motion_axis_count] = (uint16)s;
      printf("[DISCOVER] axis%d -> slave%d %s\n", motion_axis_count, s, sl->name);
      motion_axis_count++;
   }
}

/* PRE-OP 阶段：设置 CSV 模式 + 重建 PDO 映射 */
static int my_po2so_config(ecx_contextt *unused, uint16 slave)
{
   (void)unused;
   int axis = find_motion_axis(slave);

   if (axis < 0)
   {
      printf("[PRE-OP] slave%d 非运动轴，跳过 CSV/PDO 配置\n", slave);
      return 1;
   }

   /* RxPDO: CW(2) + TargetVelocity(4) + VelocityOffset(4) + Mode(1) */
   uint32 rx[] = { 0x60400010, 0x60FF0020, 0x60B10020, 0x60600008 };
   /* TxPDO: SW(2) + ActualPos(4) + ActualVel(4) + ActualTorque(2) + Err(2) + ModeDisp(1) */
   uint32 tx[] = { 0x60410010, 0x60640020, 0x606C0020, 0x60770010, 0x603F0010, 0x60610008 };

   if (pdo_map_set(slave, 0x1600, rx, 4) <= 0) { printf("[PRE-OP] slave%d RxPDO 映射失败\n", slave); return 0; }
   if (pdo_map_set(slave, 0x1A00, tx, 6) <= 0) { printf("[PRE-OP] slave%d TxPDO 映射失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C12, 0x1600) <= 0) { printf("[PRE-OP] slave%d SM2 分配失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C13, 0x1A00) <= 0) { printf("[PRE-OP] slave%d SM3 分配失败\n", slave); return 0; }

   int8_t mode = MODE_CSV;
   int wk = ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, FALSE, sizeof(mode), &mode, EC_TIMEOUTRXM);
   if (wk <= 0) { printf("[PRE-OP] slave%d 写 6060h CSV 失败\n", slave); return 0; }

   printf("[PRE-OP] axis%d slave%d PDO 映射 OK, CSV target=%d cnt/s, ramp=%.1fs, hold=%.1fs\n",
          axis, slave, TARGET_VEL, RAMP_S, HOLD_S);
   return 1;
}

/* ==========================================================================
 * RT 线程
 * ========================================================================== */
OSAL_THREAD_FUNC_RT ecatthread(void)
{
   ec_timet ts, step;

   while (!mappingdone) osal_usleep(1000);

   osal_get_monotonic_time(&ts);
   ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;
   step.tv_sec  = 0;
   step.tv_nsec = CYCLE_NS;

   ecx_send_processdata(&ctx);

   for (;;)
   {
      osal_timespecadd(&ts, &step, &ts);
      osal_monotonic_sleep(&ts);
      if (!dorun) continue;

      cycle++;
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      ecx_mbxhandler(&ctx, 0, 4);

      if (run_axes)
      {
         for (int a = 0; a < motion_axis_count; a++)
            axis_step(&g_axes[a]);
      }

      ecx_send_processdata(&ctx);
   }
}

/* ==========================================================================
 * CiA402 状态机 + CSV 速度斜坡
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

static double smoothstep(double t)
{
   if (t <= 0.0) return 0.0;
   if (t >= 1.0) return 1.0;
   return t * t * (3.0 - 2.0 * t);
}

static void log_sep(const char *title)
{
   printf("\n─── %-54s ───\n", title);
}

static void axis_step(axis_state_t *axis)
{
   ec_slavet *sl = &ctx.slavelist[axis->slave];
   if (!sl->outputs || !sl->inputs) return;
   uint8_t *out = sl->outputs;
   uint8_t *in  = sl->inputs;

   uint16_t sw        = (uint16_t)in[TX_SW] | ((uint16_t)in[TX_SW + 1] << 8);
   int32_t  pos       = read_i32_from_pdo(in + TX_POS);
   int32_t  vel       = read_i32_from_pdo(in + TX_VEL);
   int16_t  torq      = (int16_t)((uint16_t)in[TX_TORQ] | ((uint16_t)in[TX_TORQ + 1] << 8));
   uint16_t er        = (uint16_t)in[TX_ERR] | ((uint16_t)in[TX_ERR + 1] << 8);
   int8_t   mode_disp = (int8_t)in[TX_MODEDISP];

   uint16_t cw  = 0;
   int32_t  tgt_vel = 0;

   if (sw & 0x0008)
   {
      if (axis->fault_reset_cnt < 10) { cw = 0x0000; axis->fault_reset_cnt++; }
      else                            { cw = 0x0080; axis->fault_reset_cnt = 0; }
   }
   else if ((sw & 0x004F) == 0x0040) cw = 0x0006;
   else if ((sw & 0x006F) == 0x0021) cw = 0x0007;
   else if ((sw & 0x006F) == 0x0023) cw = 0x000F;
   else if ((sw & 0x006F) == 0x0027)
   {
      cw = 0x000F;

      if (!axis->enabled_logged)
      {
         axis->enabled_logged = 1;
         log_sep("CSV VELOCITY RAMP");
         printf("  cycle=%d  Op Enabled, 等待 %dms 后发速度指令\n",
                cycle, ENABLE_WAIT_MS);
      }

      if (axis->enable_wait_cnt < ENABLE_WAIT_CYCLES)
      {
         axis->enable_wait_cnt++;
         tgt_vel = 0;
      }
      else if (axis->sync_hold_cnt < 5)
      {
         axis->sync_hold_cnt++;
         tgt_vel = 0;
      }
      else if (!axis->csv_started)
      {
         axis->csv_started = 1;
         axis->sync_cycle  = cycle;
         tgt_vel = 0;
         printf("  cycle=%d  启动斜坡 → %d cnt/s\n", cycle, TARGET_VEL);
      }
      else
      {
         int step = cycle - axis->sync_cycle;

         if (step < RAMP_CYCLES)
         {
            double p = (double)step / (double)RAMP_CYCLES;
            tgt_vel = (int32_t)(TARGET_VEL * smoothstep(p));
         }
         else if (step < RAMP_CYCLES + HOLD_CYCLES)
         {
            tgt_vel = (int32_t)TARGET_VEL;
         }
         else if (step < TOTAL_MOTION_CYCLES)
         {
            double p = (double)(step - RAMP_CYCLES - HOLD_CYCLES) / (double)RAMP_CYCLES;
            tgt_vel = (int32_t)(TARGET_VEL * (1.0 - smoothstep(p)));
         }
         else
         {
            tgt_vel = 0;

            if (!axis->finished_logged)
            {
               axis->finished_logged = 1;
               printf("  cycle=%d  斜坡完成, pos=%d vel=%d torq=%d\n",
                      cycle, pos, vel, torq);
            }
         }
      }
   }
   else
   {
      cw = 0x0006;
   }

   if (sw != axis->prev_sw || er != axis->prev_err)
   {
      printf("  %-6d  SW 0x%04X->0x%04X  %-24s  CW=0x%04X  Err=0x%04X  Mode=%d\n",
             cycle, axis->prev_sw, sw, cia402_state_name(sw), cw, er, mode_disp);
      axis->prev_sw  = sw;
      axis->prev_err = er;
   }

   if ((cycle % LOG_DIV) == 0)
   {
      if (!axis->tick_header_printed)
      {
         axis->tick_header_printed = 1;
         printf("\n  %-7s %-12s %-12s %-17s %-12s %-7s\n",
                "cycle", "tgt_vel", "act_vel", "pos", "torq", "mode");
         printf("  %-7s %-12s %-12s %-17s %-12s %-7s\n",
                "------", "-----------", "-----------", "----------------", "-----------", "------");
      }
      printf("  %-7d %-12d %-12d %-17d %-12d %-7d\n",
             cycle, tgt_vel, vel, pos, torq, mode_disp);
   }

   write_u16_to_pdo(out + RX_CW, cw);
   write_i32_to_pdo(out + RX_TARGET_VEL, tgt_vel);
   write_i32_to_pdo(out + RX_VEL_OFFSET, 0);
   out[RX_MODE] = MODE_CSV;
}

/* ==========================================================================
 * 主流程
 * ========================================================================== */
static void run_motion(void)
{
   memset(g_axes, 0, sizeof(g_axes));
   for (int a = 0; a < motion_axis_count; a++)
   {
      g_axes[a].slave    = motion_slaves[a];
      g_axes[a].prev_sw  = 0xFFFF;
      g_axes[a].prev_err = 0xFFFF;
   }
   run_axes = 1;

   for (int i = 0; i < MAX_CYCLES; i++)
   {
      int all_finished = 1;
      for (int a = 0; a < motion_axis_count; a++)
      {
         if (!g_axes[a].finished_logged) all_finished = 0;
      }
      if (all_finished)
      {
         osal_usleep(1000000);
         break;
      }
      osal_usleep(10000);
   }

   run_axes = 0;
}

static void ecatbringup(const char *ifname)
{
   printf("[BOOT] EtherCAT init on %s\n", ifname);
   if (!ecx_init(&ctx, ifname))            { printf("[BOOT] ecx_init 失败\n"); return; }
   if (ecx_config_init(&ctx) <= 0)         { printf("[BOOT] 没扫到从站\n");   return; }

   discover_motion_axes();
   if (motion_axis_count <= 0)
   {
      printf("[BOOT] 未检测到可控制电机，slavecount=%d\n", ctx.slavecount);
      return;
   }
   for (int s = 1; s <= ctx.slavecount; s++)
      ctx.slavelist[s].PO2SOconfig = my_po2so_config;
   printf("[BOOT] 找到 %d 个从站，注册 PO2SOconfig 完成\n", ctx.slavecount);
   printf("[BOOT] 本次将控制 %d 个 CSV 电机\n", motion_axis_count);

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

   for (int s = 1; s <= ctx.slavecount; s++)
      if (ctx.slavelist[s].CoEdetails > 0)
         ecx_slavembxcyclic(&ctx, s);

   /* OP 前预填：CW=0x0006, 速度=0, 模式=CSV */
   dorun = 1;
   for (int i = 0; i < 500; i++)
   {
      for (int a = 0; a < motion_axis_count; a++)
      {
         uint16 s = motion_slaves[a];
         if (!ctx.slavelist[s].outputs) continue;
         uint8_t *o = ctx.slavelist[s].outputs;
         o[RX_CW] = 0x06; o[RX_CW + 1] = 0;
         write_i32_to_pdo(o + RX_TARGET_VEL, 0);
         write_i32_to_pdo(o + RX_VEL_OFFSET, 0);
         o[RX_MODE] = MODE_CSV;
      }
      osal_usleep(CYCLE_NS / 1000);
   }

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

   printf("[BOOT] 进入 OP，开始 CiA402 CSV 状态机推进\n");
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
   printf("  SOEM CSV Test — 目标 %d cnt/s  |  ramp=%.1fs  hold=%.1fs\n",
          TARGET_VEL, RAMP_S, HOLD_S);
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
