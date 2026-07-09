/*
 * 调试过程沉淀下来的几条核心经验（详见 docs/csp_test_debug_notes.md）：
 *   1. CSP 必须开 DC Sync0 + 写 0x60C2 插补周期；只设 0x6060=8 状态机会进 OP
 *      但电机不动。
 *   2. 部分驱动要求 RxPDO 至少含 60B1/60B2，每周期写 0 即可。
 *   3. 使能前每个 PDO 周期都要把 0x607A = 0x6064（持续跟随，不是一次性预填），
 *      否则使能瞬间会被驱动当作 "目标 ≠ 实际" 钉住或抽动。
 *   4. 轨迹生成（写 outputs[Target]）必须放在 RT 线程，与 receive/send 同序执行。
 *      放在主线程靠 osal_usleep 模拟 1ms 周期，会因调度抖动 + 撕裂写让 CSP 驱动
 *      看到锯齿状速度命令，表现为电机反复抽动。
 *
 * 整体流程：
 *   ecx_init -> config_init -> PO2SOconfig(PRE-OP, 重建 PDO + 写 60C2)
 *   -> config_map_group -> configdc -> ecx_dcsync0(TRUE,1ms)
 *   -> 500 周期持续跟随 (607A=6064)
 *   -> 切 OP -> run_motion 等待 RT 线程把轨迹跑完 -> 回 INIT。
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
 * 可调参数
 * ========================================================================== */
#define ENC_BITS         19      /* 电机编码器位数 */
#define GEAR_RATIO       100     /* 减速比 */
#define TARGET_DEG       20       /* 输出端目标角度（°）；0 表示原地不动，便于干跑通信链路 */
#define MOVE_DURATION_S  4       /* 期望走完时长（s） */
#define CYCLE_NS         1000000 /* RT 周期 = DC Sync0 = 0x60C2 = 1 ms，三者必须一致 */
#define MOTION_SLAVE_FIRST 1
#define MAX_MOTION_AXES    1
#define LOG_DIV            50  /* 每 N 个 RT 周期打一次 TICK 行 */
#define MAX_CYCLES         40000 /* run_motion 整体超时（10ms 轮询单位） */
#define MAX_TARGET_LEAD_COUNTS  20000  /* 命令变化率钳位：|tgt[k]-tgt[k-1]| 上限 (counts/cycle) */
#define TARGET_TOLERANCE_COUNTS 1000
#define MODE_CSP      8

/* 派生量 */
#define TARGET_COUNTS \
   ((int32_t)(((int64_t)1 << ENC_BITS) * GEAR_RATIO * TARGET_DEG / 360))

#define CSP_STEPS               ((MOVE_DURATION_S * 1000000000LL) / CYCLE_NS)


/* RxPDO 0x1600（13B）: CW(2)+607A(4)+60B1(4)+60B2(2)+6060(1)
 * 60B1/60B2 写 0 即可，但映射不能省 —— 部分驱动缺它会拒绝采纳 607A。 */
#define RX_CW       0
#define RX_TARGET   2
#define RX_VELOFF   6
#define RX_TORQOFF  10
#define RX_MODE     12

/* TxPDO 0x1A00（21B）: SW(2)+6064(4)+606C(4)+6077(2)+60F4(4)+603F(2)+6061(1) */
#define TX_SW       0
#define TX_POS      2
#define TX_VEL      6
#define TX_TORQ     10
#define TX_ERR      16
#define TX_MODEDISP 18

// 每个轴的运行flag集合成一个结构体
typedef struct
{
   uint16_t slave;
   int      enabled_logged;
   int      reached_logged;
   int      csp_started;       /* 0=尚未启动 CSP 插补; 1=已启动 */
   int      sync_cycle;        /* csp_started=1 时记录的 RT cycle 起点 */
   int      tick_header_printed;
   int      fault_reset_cnt;   /* Fault 状态下做 0x00->0x80 的脉冲计数 */
   int32_t  base_pos;          /* Op enabled 瞬间的实际位置，作轨迹起点（锁死后不再更新） */
   int32_t  final_target;      /* base_pos + TARGET_COUNTS（锁死末端，故障复位也不重新规划） */
   int32_t  last_tgt;          /* 上一周期实际写出去的 607A，用于变化率钳位 */
   int      last_tgt_inited;   /* 0=last_tgt 还没用 pos 对齐过 */
   uint16_t prev_sw;           /* 仅供 [CHG] 行打印用 */
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
static uint16               motion_slaves[MAX_MOTION_AXES];
static axis_state_t         g_axes[MAX_MOTION_AXES];
static volatile int         run_axes = 0;
static int                  sync_hold_rt = 0;

/* 全局轨迹时钟暂停：任一轴离开 Op enabled 时整组冻结，恢复时累加暂停的周期数。
 * 多轴情况下避免某一轴故障 50ms 让其它轴继续走、末端笛卡尔轨迹断点。 */
static int                  g_pause_cycles = 0;
static int                  g_pause_start  = -1;

/* 单轴 CSP 周期处理：推进 CiA402 状态机并更新 PDO 输出。 */
static void axis_step(axis_state_t *axis, int sync_trigger);

/* ==========================================================================
 * 日志：stdout 通过 tee 同时写到 log/csp_test_YYYYMMDD_HHMMSS.log
 * ========================================================================== */
/* 初始化日志重定向，把 stdout 同时输出到终端和日志文件。 */
static void setup_log(void)
{
   mkdir("log", 0755);
   time_t now = time(NULL);
   struct tm tmv;
   localtime_r(&now, &tmv);
   char path[128];
   strftime(path, sizeof(path), "log/csp_test_%Y%m%d_%H%M%S.log", &tmv);

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
/* 重写指定 PDO 映射对象，先清零子索引 0 再逐项写入映射条目。 */
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

/* 配置指定 Sync Manager 的 PDO 分配表，使其只关联一个 PDO。 */
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

/* 从小端格式的 PDO 字节区读取 32 位有符号整数。 */
static int32_t read_i32_from_pdo(const uint8_t *p)
{
   return (int32_t)((uint32_t)p[0]
                  | ((uint32_t)p[1] << 8)
                  | ((uint32_t)p[2] << 16)
                  | ((uint32_t)p[3] << 24));
}

/* 将 32 位有符号整数按小端格式写入 PDO 字节区。 */
static void write_i32_to_pdo(uint8_t *p, int32_t v)
{  // 真的写入的时候字节高地位需要反转
   p[0] = (uint8_t)(v & 0xFF); // 相当于提取最后两位，即提取第7、8位
   p[1] = (uint8_t)((v >> 8) & 0xFF); // 往后移两位再提取，相当于提取第5、6位
   p[2] = (uint8_t)((v >> 16) & 0xFF); // 往后移四位再提取，相当于提取第3、4位
   p[3] = (uint8_t)((v >> 24) & 0xFF); // 往后移六位再提取，相当于提取第1、2位
}

/* 将 16 位有符号整数按小端格式写入 PDO 字节区。 */
static void write_i16_to_pdo(uint8_t *p, int16_t v)
{
   p[0] = (uint8_t)(v & 0xFF);
   p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* 将 16 位无符号整数按小端格式写入 PDO 字节区。 */
static void write_u16_to_pdo(uint8_t *p, uint16_t v)
{
   p[0] = (uint8_t)(v & 0xFF);
   p[1] = (uint8_t)((v >> 8) & 0xFF);
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

/* CSP 必需的 SDO 设置：mode + 插补周期。
 * 失败经验：只设 0x6060=8 不写 0x60C2 时，驱动默认插补周期可能
 *   与我们 1ms 不匹配，表现为位置加梯形跳进 → 电机动作猛列。 */
/* 在 PRE-OP 阶段写入 CSP 模式和 1ms 插补周期参数。 */
static int write_csp_setup(uint16 slave)
{
   int8_t  mode     = MODE_CSP;
   uint8_t ip_time  = (uint8_t)(CYCLE_NS / 1000000);   /* 1 */
   int8_t  ip_index = -3;                              /* 10^-3 s */
   int wk;

   wk = ecx_SDOwrite(&ctx, slave, 0x6060, 0x00, FALSE, sizeof(mode),     &mode,     EC_TIMEOUTRXM);
   if (wk <= 0) { printf("[PRE-OP] slave%d 写 6060h CSP 失败\n", slave); return 0; }
   // 60C2是插补周期参数，0x01是插补周期时间，0x02是插补周期指数（因为用的是科学计数法）
   wk = ecx_SDOwrite(&ctx, slave, 0x60C2, 0x01, FALSE, sizeof(ip_time),  &ip_time,  EC_TIMEOUTRXM);
   if (wk <= 0) { printf("[PRE-OP] slave%d 写 60C2:01 失败\n", slave); return 0; }
   wk = ecx_SDOwrite(&ctx, slave, 0x60C2, 0x02, FALSE, sizeof(ip_index), &ip_index, EC_TIMEOUTRXM);
   if (wk <= 0) { printf("[PRE-OP] slave%d 写 60C2:02 失败\n", slave); return 0; }

   printf("[PRE-OP] slave%d CSP setup OK: 6060=8, 60C2=%ums\n", slave, ip_time);
   return 1;
}

/* PRE-OP 阶段 PO2SOconfig：重建 PDO 映射 + 写 CSP 必需参数。
 *
 * RxPDO 里 60B1/60B2 是“必须映射但可以一直写 0”。
 *   — 失败经验：只映 CW+607A+6060 时，部分驱动 SM2 实际会拒绝采纳 607A。 */
/* SOEM 状态转换回调：从 PRE-OP 到 SAFE-OP 前完成 PDO 和 CSP 参数配置。 */
static int my_po2so_config(ecx_contextt *unused, uint16 slave)
{
   (void)unused;
   uint32 rx[] = { 0x60400010, 0x607A0020, 0x60B10020, 0x60B20010, 0x60600008 };
   uint32 tx[] = { 0x60410010, 0x60640020, 0x606C0020, 0x60770010, 0x60F40020, 0x603F0010, 0x60610008 };
   int axis = find_motion_axis(slave);

   if (axis < 0)
   {
      printf("[PRE-OP] slave%d 非运动轴，跳过 CSP/PDO 配置\n", slave);
      return 1;
   }

   if (pdo_map_set(slave, 0x1600, rx, 5) <= 0) { printf("[PRE-OP] slave%d RxPDO 映射失败\n", slave); return 0; }
   if (pdo_map_set(slave, 0x1A00, tx, 7) <= 0) { printf("[PRE-OP] slave%d TxPDO 映射失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C12, 0x1600) <= 0) { printf("[PRE-OP] slave%d SM2 分配失败\n", slave); return 0; }
   if (sm_assign_set(slave, 0x1C13, 0x1A00) <= 0) { printf("[PRE-OP] slave%d SM3 分配失败\n", slave); return 0; }
   if (!write_csp_setup(slave)) return 0;

   printf("[PRE-OP] axis%d slave%d PDO 映射 OK，CSP target=%d counts (%d°)\n",
          axis, slave, TARGET_COUNTS, TARGET_DEG);
   return 1;
}

/* ==========================================================================
 * RT 线程：周期性 PDO 收发 + 轨迹推进
 *
 * 关键经验：axis_step 必须在这里调用（receive 之后、send 之前）。
 * 只要拿到主线程去跑，就会出现以下坑过的调试现象：
 *   - osal_usleep(1ms) 实际 1.0–3.0ms抖动→ 某些 PDO 帧重发上一次目标（Δ=0），
 *     有些帧一次推 2 步 → 驱动看到锯齿状速度命令、电机反复抽动。
 *   - 主线程逐字节写 4B Target 与 RT 发包有撕裂窗口，偏远目标会让电机猛跳。
 * ========================================================================== */
/* 实时线程入口：按固定周期收发 PDO，并在发送前更新各轴目标。 */
OSAL_THREAD_FUNC_RT ecatthread(void)
{
   ec_timet ts, step;

   while (!mappingdone) osal_usleep(1000);

   // TODO 这里时间对齐是什么意思
   osal_get_monotonic_time(&ts);
   ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;   /* 对齐到下一毫秒 */
   step.tv_sec  = 0;
   step.tv_nsec = CYCLE_NS;

   ecx_send_processdata(&ctx);

   for (;;) // 无限循环
   {
      osal_timespecadd(&ts, &step, &ts);
      osal_monotonic_sleep(&ts);
      if (!dorun) continue; // 立即结束本次循环

      cycle++;
      wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
      ecx_mbxhandler(&ctx, 0, 4);

      if (run_axes)
      {
         /* === 全局轨迹时钟暂停判定 ===
          * 任一轴 SW != Operation enabled (0x0027) 视为暂停：所有轴 step 同步冻结，
          * 恢复后把这段时长累加到 g_pause_cycles，让 axis_step 里
          *   step = cycle - sync_cycle - g_pause_cycles
          * 整体往后推迟相同时长，避免单轴故障导致末端笛卡尔轨迹断点。 */
         int all_op = 1;
         for (int a = 0; a < motion_axis_count; a++)
         {
            uint8_t *in_ = ctx.slavelist[motion_slaves[a]].inputs;
            if (!in_) { all_op = 0; break; }
            uint16_t s_sw = (uint16_t)in_[TX_SW] | ((uint16_t)in_[TX_SW + 1] << 8);
            if ((s_sw & 0x006F) != 0x0027) { all_op = 0; break; }
         }
         if (!all_op)
         {
            if (g_pause_start < 0) g_pause_start = cycle;
         }
         else if (g_pause_start >= 0)
         {
            g_pause_cycles += (cycle - g_pause_start);
            printf("  [PAUSE] cycle=%d  paused %d cycles\n", cycle, g_pause_cycles);
            g_pause_start = -1;
         }

         /* sync_trigger：所有轴都 Op enabled 后隔 5 个周期再启动 CSP 插补，
          * 让 CW=0x000F 在驱动那侧稳定生效后再下发新目标。 */
         int all_enabled = 1;
         for (int a = 0; a < motion_axis_count; a++)
            if (!g_axes[a].enabled_logged) { all_enabled = 0; break; }

         int sync_trigger = 0;
         if (all_enabled && all_op)
         {
            if (sync_hold_rt < 5)        sync_hold_rt++;
            else if (sync_hold_rt == 5)  { sync_trigger = 1; sync_hold_rt++; }
         }

         for (int a = 0; a < motion_axis_count; a++)
            axis_step(&g_axes[a], sync_trigger); // 调用插值算法
      }

      ecx_send_processdata(&ctx);
   }
}

/* ==========================================================================
 * 主流程
 * ========================================================================== */
/* 根据 CiA402 状态字返回便于日志打印的状态名称。 */
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

/* axis_step：在 RT 线程每个 PDO 周期调用一次。
 *   - 根据 SW 推进 CiA402 状态机；Op enabled 下按 RT cycle 生成轨迹。
 *   - 轨迹进度用 (cycle - sync_cycle) / CSP_STEPS，避免用调用次数计数被
 *     主线程抖动带偏；smoothstep 让加减速连续。
 *   - tgt 用 ±MAX_TARGET_LEAD_COUNTS 钳住：万一驱动跟不上（机械卡死/限位）
 *     不会让后续轨迹越跑越远。 */
/* 阶段分隔线 */
static void log_sep(const char *title)
{
   printf("\n─── %-54s ───\n", title);
}

/* 单轴周期控制核心：读取反馈、计算控制字和目标位置并写回 RxPDO。 */
static void axis_step(axis_state_t *axis, int sync_trigger)
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

   /* 第一次调用：把 last_tgt 与当前 pos 对齐，避免后续变化率钳位首周期误伤 */
   if (!axis->last_tgt_inited)
   {
      axis->last_tgt        = pos;
      axis->last_tgt_inited = 1;
   }

   uint16_t cw  = 0;
   /* 关键：默认保持上一周期命令值，不要默认 = pos。
    * 旧实现 tgt=pos 会在 Fault→Op enabled 切换的那 1 个周期出现命令阶跃
    * (last cycle Fault: tgt≈pos_drift；this cycle 立刻 base_pos+delta)，
    * 编码器复位后 pos 跳几千 counts，命令也跟着跳，电机以最大速度冲。
    * 现在把 tgt 默认为 last_tgt，配合下面的变化率钳位形成双重保护。 */
   int32_t  tgt = axis->last_tgt;

   /* CiA402 状态机推进（保留故障自动复位语义；base_pos/final_target 锁死） */
   if (sw & 0x0008) // 状态字的BIT3=1说明报错了
   {
      /* Fault：10 个周期 CW=0，随后 1 个周期把控制字的BIT7置1,FAULT RESET */
      if (axis->fault_reset_cnt < 10) { cw = 0x0000; axis->fault_reset_cnt++; }
      else                            { cw = 0x0080; axis->fault_reset_cnt = 0; }
      /* 故障期间 tgt 保持 last_tgt：编码器跳变也不污染命令值 */
   }
   else if ((sw & 0x004F) == 0x0040) cw = 0x0006;   /* Switch on disabled  -> Shutdown */
   else if ((sw & 0x006F) == 0x0021) cw = 0x0007;   /* Ready to switch on  -> Switch on */
   else if ((sw & 0x006F) == 0x0023) cw = 0x000F;   /* Switched on         -> Enable op */
   else if ((sw & 0x006F) == 0x0027)                /* Operation enabled */
   {
      cw = 0x000F;

      if (!axis->enabled_logged)
      {
         /* 仅第一次进 Op enabled 锁定 base_pos / final_target；
          * 故障复位再回到此分支不会重写，符合"锁死末端"原则。 */
         axis->enabled_logged = 1;
         axis->base_pos       = pos;
         axis->final_target   = pos + TARGET_COUNTS;
         axis->last_tgt       = pos;
         tgt                  = pos;
         log_sep("CSP MOTION");
         printf("  cycle=%d  Op Enabled, base=%d -> final=%d (%d°)\n",
                cycle, axis->base_pos, axis->final_target, TARGET_DEG);
      }

      if (!axis->csp_started)
      {
         /* sync_trigger 未到前：跟随 pos，让驱动看到 follow error = 0 */
         tgt = pos;
         if (sync_trigger)
         {
            axis->csp_started = 1;
            axis->sync_cycle  = cycle;
            tgt               = axis->base_pos;
            printf("  cycle=%d  CSP start -> %d counts (%d°)\n",
                   cycle, axis->final_target, TARGET_DEG);
         }
      }
      else
      {
         /* smoothstep 插补；step 减 g_pause_cycles 让故障期间冻结轨迹时钟，
          * 避免复位后 step 飞过 CSP_STEPS 导致 tgt 直接跳到 final_target。 */
         int     step  = cycle - axis->sync_cycle - g_pause_cycles;
         if (step < 0) step = 0;
         double  p     = (step >= CSP_STEPS) ? 1.0 : (double)step / (double)CSP_STEPS;
         double  s     = p * p * (3.0 - 2.0 * p); // S是对P的平滑
         int64_t delta = (int64_t)((double)TARGET_COUNTS * s);
         tgt = axis->base_pos + (int32_t)delta;

         int err = pos - axis->final_target;
         if (step > CSP_STEPS && !axis->reached_logged &&
             llabs((long long)err) <= TARGET_TOLERANCE_COUNTS)
         {
             axis->reached_logged = 1;
             printf("  cycle=%d  Target Reached, pos=%d (err=%d)\n",
                    cycle, pos, err);
         }
      }
   }
   else
   {
      cw = 0x0006;
   }

   /* === 命令变化率钳位：|tgt - last_tgt| ≤ MAX_TARGET_LEAD_COUNTS ===
    * 替代旧的 |tgt - pos| 钳位。后者在 pos 不动时持续把 tgt 推到 pos±N，
    * 一旦驱动恢复响应就以 N counts/cycle 的速度狂追；本质上是把 lead 转化成速度命令。
    * 现在钳位的是相邻两周期命令位移：无论 pos 怎么跳，CSP 命令本身始终平滑，
    * 在 1ms 周期里最多 MAX_TARGET_LEAD_COUNTS counts，对应可控的最大速度。 */
   if (tgt > axis->last_tgt + MAX_TARGET_LEAD_COUNTS)
      tgt = axis->last_tgt + MAX_TARGET_LEAD_COUNTS;
   if (tgt < axis->last_tgt - MAX_TARGET_LEAD_COUNTS)
      tgt = axis->last_tgt - MAX_TARGET_LEAD_COUNTS;

   /* 诊断打印 */
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
         printf("\n  %-7s %-17s %-17s %-12s %-12s\n",
                "cycle", "tgt", "pos", "vel", "torq");
         printf("  %-7s %-17s %-17s %-12s %-12s\n",
                "------", "----------------", "----------------", "-----------", "-----------");
      }
      printf("  %-7d %-17d %-17d %-12d %-12d\n",
             cycle, tgt, pos, vel, torq);
   }

   /* 写到本地output缓冲区，ecx_send_processdata 顺序执行，无撕裂 */
   write_u16_to_pdo(out + RX_CW, cw);
   write_i32_to_pdo(out + RX_TARGET,  tgt); // out + RX_TARGET指向 outputs 缓冲区第 2 个字节的位置
   write_i32_to_pdo(out + RX_VELOFF,  0);
   write_i16_to_pdo(out + RX_TORQOFF, 0);
   out[RX_MODE]   = MODE_CSP;

   /* 更新 last_tgt，作为下一周期变化率钳位的基准 */
   axis->last_tgt = tgt;
}

/* 启动所有轴运动，并等待 RT 线程报告到位或超时。 */
static void run_motion(void)
{
   memset(g_axes, 0, sizeof(g_axes));
   for (int a = 0; a < motion_axis_count; a++)
   {
      g_axes[a].slave    = motion_slaves[a];
      g_axes[a].prev_sw  = 0xFFFF;
      g_axes[a].prev_err = 0xFFFF;
      /* 把 last_tgt 用当前 PDO 的 pos 对齐：bringup 已跑过 500 周期，inputs 必然有效。
       * 这样 axis_step 第一次进入时不会误以为命令值默认 0，被钳位到 0±MAX_TARGET_LEAD_COUNTS。 */
      uint8_t *in = ctx.slavelist[motion_slaves[a]].inputs;
      if (in)
      {
         g_axes[a].last_tgt        = read_i32_from_pdo(in + TX_POS);
         g_axes[a].last_tgt_inited = 1;
      }
   }
   sync_hold_rt   = 0;
   g_pause_cycles = 0;
   g_pause_start  = -1;
   run_axes = 1; // 从这里开始ecatthread正式调用axis_step，用一个FLAG与另一个线程关联

   /* 主线程仅作"等待到位"观察者：所有真正的 PDO 输出都由 RT 线程产生 */
   for (int i = 0; i < MAX_CYCLES; i++) // 循环等待所有电机到位
   {
      int all_reached = 1;
      for (int a = 0; a < motion_axis_count; a++)
      {
         if (!g_axes[a].reached_logged) all_reached = 0;
      } // 检查是否全部电机都已经到位
      if (all_reached)
      {
         /* 到位后再多跑 1 秒让 RT 把最终目标稳态发出去 */
         osal_usleep(1000000);
         break;
      }
      osal_usleep(10000);   /* 10ms 轮询足够，无需贴 RT 周期 */
   }

   run_axes = 0;
}

/* 完成 EtherCAT 主站初始化、从站配置、进 OP、执行运动和退出清理。 */
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
   printf("[BOOT] 本次将控制 %d 个 CSP 电机\n", motion_axis_count);

   // TODO 两个机械臂分组
   ec_groupt *grp = &ctx.grouplist[0];
   ecx_config_map_group(&ctx, IOmap, 0);
   expectedWKC = (grp->outputsWKC * 2) + grp->inputsWKC;
   printf("[BOOT] PDO 映射完成，expectedWKC=%d，IO segment[0]=%d\n",
          expectedWKC, grp->IOsegment[0]);

   mappingdone = 1;
   ecx_configdc(&ctx);

   /* 早期没开 Sync0（参数 FALSE,0,0），状态机能进 OP、ModeDisp=8
    * 但电机一动不动 —— 大量 CSP 驱动会忽略 607A。开启 Sync0 后才正常采纳。 */
   for (int s = 1; s <= ctx.slavecount; s++)
      if (ctx.slavelist[s].hasdc)
      {
         ecx_dcsync0(&ctx, s, TRUE, CYCLE_NS, 0);
         printf("[BOOT] slave%d DC Sync0 enabled, period=%dns\n", s, CYCLE_NS);
      }

   /* 让 CoE 从站接入循环邮箱处理 */
   for (int s = 1; s <= ctx.slavecount; s++)
      if (ctx.slavelist[s].CoEdetails > 0)
         ecx_slavembxcyclic(&ctx, s);

   /* === CSP 入 OP 前的关键步骤：使能前持续 607A=6064 跟随 ===
    * 早先只在这里一次性预填 outputs，使能瞬间驱动认为
    *   "目标位置 ≠ 实际位置" → 直接抽动或被钉住。
    * 修复：dorun=1 后跑 500 个 1ms 周期，每次都把当前 6064 写回 607A，
    *   让驱动在使能那一刻看到 follow error = 0。 */
   dorun = 1;
   for (int i = 0; i < 500; i++)
   {
      for (int a = 0; a < motion_axis_count; a++)
      {
         uint16 s = motion_slaves[a];
         if (!ctx.slavelist[s].inputs || !ctx.slavelist[s].outputs) continue;
         int32_t pos = read_i32_from_pdo(ctx.slavelist[s].inputs + TX_POS);
         uint8_t *o  = ctx.slavelist[s].outputs;
         o[RX_CW] = 0x06; o[RX_CW + 1] = 0;
         write_i32_to_pdo(o + RX_TARGET,  pos); // TODO 变量名不统一的需要统一
         write_i32_to_pdo(o + RX_VELOFF,  0);
         write_i16_to_pdo(o + RX_TORQOFF, 0);
         o[RX_MODE] = MODE_CSP;
         if (i == 499)
            printf("[BOOT] slave%d OP前最终 607A=%d (= 6064)\n", s, pos);
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
      goto teardown; // 进状态机失败，直接跳到函数的最后
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
   printf("  SOEM csp_test — CSP 模式  |  目标 %d°\n", TARGET_DEG);
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
