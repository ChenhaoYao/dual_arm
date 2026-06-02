#include "dual_arm_soem_bridge/soem_pp_master.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "soem/soem.h"

namespace dual_arm_soem_bridge
{

// ==========================================================================
// 常量(迁移自 ec_sample.c)
// ==========================================================================
namespace
{
constexpr int64_t CYCLE_NS = 1000000;            // RT 线程 PDO 周期：1ms
constexpr int PP_TARGET_PRELOAD_CYCLES = 20;     // 触发前预载周期数
constexpr uint8_t MODE_PP = 1;                   // 0x6060 模式：Profile Position

// RxPDO (0x1600) 7 字节：CW(2) + TargetPos(4) + Mode(1)
constexpr int RX_CW = 0;
constexpr int RX_TARGET = 2;
constexpr int RX_MODE = 6;
// TxPDO (0x1A00) 15 字节：SW(2)+ActualPos(4)+ActualVel(4)+ActualTorq(2)+Err(2)+ModeDisp(1)
constexpr int TX_SW = 0;
constexpr int TX_POS = 2;
constexpr int TX_VEL = 6;
constexpr int TX_TORQ = 10;
constexpr int TX_ERR = 12;

constexpr double TWO_PI = 6.283185307179586;

// SOEM 上下文与 IO 映射：单实例主站，沿用 ec_sample 的文件级静态变量。
ecx_contextt g_ctx;
uint8 g_iomap[4096];
// 供静态 PO2SOconfig 回调访问当前主站实例(读取每轴 profile 参数)。
SoemPpMaster * g_self = nullptr;

// 写 PDO 映射(0x1600/0x1A00)。
int pdo_map_set(uint16 slave, uint16 idx, const uint32 * entries, uint8 n)
{
  uint8 zero = 0;
  int wk = ecx_SDOwrite(&g_ctx, slave, idx, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
  if (wk <= 0) return wk;
  for (uint8 i = 0; i < n; i++) {
    uint32 v = entries[i];
    wk = ecx_SDOwrite(&g_ctx, slave, idx, (uint8)(i + 1), FALSE, sizeof(v), &v, EC_TIMEOUTRXM);
    if (wk <= 0) return wk;
  }
  return ecx_SDOwrite(&g_ctx, slave, idx, 0x00, FALSE, sizeof(n), &n, EC_TIMEOUTRXM);
}

// 设 SyncManager PDO 分配(0x1C12/0x1C13)。
int sm_assign_set(uint16 slave, uint16 sm_idx, uint16 pdo_idx)
{
  uint8 zero = 0, one = 1;
  int wk = ecx_SDOwrite(&g_ctx, slave, sm_idx, 0x00, FALSE, sizeof(zero), &zero, EC_TIMEOUTRXM);
  if (wk <= 0) return wk;
  wk = ecx_SDOwrite(&g_ctx, slave, sm_idx, 0x01, FALSE, sizeof(pdo_idx), &pdo_idx, EC_TIMEOUTRXM);
  if (wk <= 0) return wk;
  return ecx_SDOwrite(&g_ctx, slave, sm_idx, 0x00, FALSE, sizeof(one), &one, EC_TIMEOUTRXM);
}

const char * cia402_state_name(uint16_t sw)
{
  if (sw & 0x0008) return "Fault";
  if ((sw & 0x004F) == 0x0040) return "Switch on disabled";
  if ((sw & 0x006F) == 0x0021) return "Ready to switch on";
  if ((sw & 0x006F) == 0x0023) return "Switched on";
  if ((sw & 0x006F) == 0x0027) return "Operation enabled";
  if ((sw & 0x006F) == 0x0007) return "Quick stop active";
  return "?";
}

// PRE-OP 阶段重建 PDO 映射 + 写每轴运动参数(profile vel/acc)。
int po2so_config(ecx_contextt * ctx, uint16 slave)
{
  (void)ctx;
  uint32 rx[] = {0x60400010, 0x607A0020, 0x60600008};
  uint32 tx[] = {0x60410010, 0x60640020, 0x606C0020, 0x60770010, 0x603F0010, 0x60610008};

  if (pdo_map_set(slave, 0x1600, rx, 3) <= 0) {
    printf("[PRE-OP] slave%d RxPDO 映射失败\n", slave);
    return 0;
  }
  if (pdo_map_set(slave, 0x1A00, tx, 6) <= 0) {
    printf("[PRE-OP] slave%d TxPDO 映射失败\n", slave);
    return 0;
  }
  if (sm_assign_set(slave, 0x1C12, 0x1600) <= 0) {
    printf("[PRE-OP] slave%d SM2 分配失败\n", slave);
    return 0;
  }
  if (sm_assign_set(slave, 0x1C13, 0x1A00) <= 0) {
    printf("[PRE-OP] slave%d SM3 分配失败\n", slave);
    return 0;
  }

  // 找到该 slave 对应轴的 profile 参数并下发。
  if (g_self != nullptr) {
    for (const auto & ax : g_self->axes_for_config()) {
      if (ax.slave == slave && ax.profile_vel > 0) {
        uint32 vel = ax.profile_vel;
        uint32 acc = ax.profile_acc > 0 ? ax.profile_acc : ax.profile_vel;
        ecx_SDOwrite(&g_ctx, slave, 0x6081, 0x00, FALSE, sizeof(vel), &vel, EC_TIMEOUTRXM);
        ecx_SDOwrite(&g_ctx, slave, 0x6083, 0x00, FALSE, sizeof(acc), &acc, EC_TIMEOUTRXM);
        ecx_SDOwrite(&g_ctx, slave, 0x6084, 0x00, FALSE, sizeof(acc), &acc, EC_TIMEOUTRXM);
        printf("[PRE-OP] slave%d profile vel=%u acc=%u\n", slave, vel, acc);
      }
    }
  }
  printf("[PRE-OP] slave%d PDO 映射 OK\n", slave);
  return 1;
}

void add_time_ns(ec_timet * ts, int64_t add)
{
  ec_timet a;
  a.tv_nsec = add % 1000000000LL;
  a.tv_sec = (add - a.tv_nsec) / 1000000000LL;
  osal_timespecadd(ts, &a, ts);
}
}  // namespace

// ==========================================================================
// SoemPpMaster
// ==========================================================================
SoemPpMaster::SoemPpMaster() = default;

SoemPpMaster::~SoemPpMaster()
{
  // 析构兜底停止，避免进程退出后 EtherCAT 仍处于 OP。
  stop();
}

const std::vector<AxisConfig> & SoemPpMaster::axes_for_config() const
{
  return axis_configs_;
}

bool SoemPpMaster::configure(const std::string & ifname, const std::vector<AxisConfig> & axes)
{
  ifname_ = ifname;
  axis_configs_ = axes;
  // 网卡名为空仍允许 dry-run；真实启动在 start() 内再校验。
  configured_ = true;
  return configured_;
}

bool SoemPpMaster::configure(const std::string & ifname)
{
  return configure(ifname, axis_configs_);
}

int32_t SoemPpMaster::rad_to_counts(const AxisConfig & cfg, double rad) const
{
  double scale = (double)((int64_t)1 << cfg.enc_bits) * cfg.gear_ratio / TWO_PI;
  int64_t c = (int64_t)std::llround(rad * scale) * cfg.direction + cfg.zero_offset_counts;
  if (c < cfg.min_counts) c = cfg.min_counts;
  if (c > cfg.max_counts) c = cfg.max_counts;
  return (int32_t)c;
}

double SoemPpMaster::counts_to_rad(const AxisConfig & cfg, int32_t counts) const
{
  double scale = (double)((int64_t)1 << cfg.enc_bits) * cfg.gear_ratio / TWO_PI;
  return ((double)(counts - cfg.zero_offset_counts) / scale) * cfg.direction;
}

double SoemPpMaster::counts_to_rad_vel(const AxisConfig & cfg, int32_t counts) const
{
  // 速度换算不含零点偏置。
  double scale = (double)((int64_t)1 << cfg.enc_bits) * cfg.gear_ratio / TWO_PI;
  return ((double)counts / scale) * cfg.direction;
}

bool SoemPpMaster::start()
{
  if (!configured_) return false;
  if (running_.load()) return true;
  if (ifname_.empty()) {
    printf("[SOEM] ifname 为空，无法启动真实 EtherCAT\n");
    return false;
  }
  if (axis_configs_.empty()) {
    printf("[SOEM] 未配置任何轴，拒绝启动\n");
    return false;
  }

  // 构造运行期轴状态。
  axes_.clear();
  for (const auto & cfg : axis_configs_) {
    auto rt = std::make_unique<AxisRuntime>();
    rt->cfg = cfg;
    axes_.push_back(std::move(rt));
  }

  g_self = this;
  dorun_.store(false);
  mapping_done_.store(false);
  rt_should_exit_.store(false);
  cycle_ = 0;

  // 先起 RT 线程(它会等待 mapping_done)，再做 bringup(迁移自 ec_sample 顺序)。
  rt_thread_ = std::thread([this] { rt_loop(); });

  if (!ecat_bringup()) {
    printf("[SOEM] bringup 失败，回退\n");
    rt_should_exit_.store(true);
    dorun_.store(false);
    if (rt_thread_.joinable()) rt_thread_.join();
    g_self = nullptr;
    return false;
  }

  running_.store(true);
  return true;
}

void SoemPpMaster::stop()
{
  if (!running_.load() && !rt_thread_.joinable()) {
    return;
  }
  dorun_.store(false);
  if (running_.load()) {
    ecat_teardown();
  }
  rt_should_exit_.store(true);
  if (rt_thread_.joinable()) {
    rt_thread_.join();
  }
  running_.store(false);
  g_self = nullptr;
}

bool SoemPpMaster::ecat_bringup()
{
  printf("[BOOT] EtherCAT init on %s\n", ifname_.c_str());
  if (!ecx_init(&g_ctx, ifname_.c_str())) {
    printf("[BOOT] ecx_init 失败\n");
    return false;
  }
  if (ecx_config_init(&g_ctx) <= 0) {
    printf("[BOOT] 没扫到从站\n");
    return false;
  }
  printf("[BOOT] 找到 %d 个从站\n", g_ctx.slavecount);

  for (int s = 1; s <= g_ctx.slavecount; s++) {
    g_ctx.slavelist[s].PO2SOconfig = po2so_config;
  }

  ec_groupt * grp = &g_ctx.grouplist[0];
  ecx_config_map_group(&g_ctx, g_iomap, 0);
  expected_wkc_ = (grp->outputsWKC * 2) + grp->inputsWKC;
  printf("[BOOT] PDO 映射完成，expectedWKC=%d\n", expected_wkc_);

  mapping_done_.store(true);
  ecx_configdc(&g_ctx);
  for (int s = 1; s <= g_ctx.slavecount; s++) {
    if (g_ctx.slavelist[s].hasdc) {
      ecx_dcsync0(&g_ctx, s, TRUE, CYCLE_NS, 0);
    }
  }
  for (int s = 1; s <= g_ctx.slavecount; s++) {
    if (g_ctx.slavelist[s].CoEdetails > 0) {
      ecx_slavembxcyclic(&g_ctx, s);
    }
  }

  // 预填 outputs，避免 RT 线程首帧写下 CW=0/Mode=0。
  for (auto & ax : axes_) {
    uint16_t s = ax->cfg.slave;
    if (s >= 1 && s <= g_ctx.slavecount && g_ctx.slavelist[s].outputs) {
      uint8_t * o = g_ctx.slavelist[s].outputs;
      o[RX_CW] = 0x06;
      o[RX_CW + 1] = 0;
      std::memset(o + RX_TARGET, 0, 4);
      o[RX_MODE] = MODE_PP;
    }
  }

  dorun_.store(true);
  osal_usleep(500000);  // 等 RT 线程稳定 PDO 0.5s

  g_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
  ecx_writestate(&g_ctx, 0);
  ecx_statecheck(&g_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

  if (g_ctx.slavelist[0].state != EC_STATE_OPERATIONAL) {
    ecx_readstate(&g_ctx);
    for (int s = 1; s <= g_ctx.slavecount; s++) {
      if (g_ctx.slavelist[s].state != EC_STATE_OPERATIONAL) {
        printf("[BOOT] slave%d 未进入 OP, state=0x%02X AL=0x%04X (%s)\n",
          s, g_ctx.slavelist[s].state, g_ctx.slavelist[s].ALstatuscode,
          ec_ALstatuscode2string(g_ctx.slavelist[s].ALstatuscode));
      }
    }
    return false;
  }
  printf("[BOOT] 进入 OP，开始 CiA402 PP 跟随\n");
  return true;
}

void SoemPpMaster::ecat_teardown()
{
  dorun_.store(false);
  g_ctx.slavelist[0].state = EC_STATE_SAFE_OP;
  ecx_writestate(&g_ctx, 0);
  ecx_statecheck(&g_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
  g_ctx.slavelist[0].state = EC_STATE_INIT;
  ecx_writestate(&g_ctx, 0);
  ecx_statecheck(&g_ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE);
  printf("[EXIT] 已回到 INIT\n");
}

void SoemPpMaster::rt_loop()
{
  ec_timet ts;
  while (!mapping_done_.load() && !rt_should_exit_.load()) {
    osal_usleep(1000);
  }
  if (rt_should_exit_.load()) return;

  osal_get_monotonic_time(&ts);
  ts.tv_nsec = ((ts.tv_nsec / 1000000) + 1) * 1000000;  // 对齐到下一毫秒
  ecx_send_processdata(&g_ctx);

  while (!rt_should_exit_.load()) {
    add_time_ns(&ts, CYCLE_NS);
    osal_monotonic_sleep(&ts);
    if (!dorun_.load()) continue;

    cycle_++;
    wkc_ = ecx_receive_processdata(&g_ctx, EC_TIMEOUTRET);
    ecx_mbxhandler(&g_ctx, 0, 4);

    for (auto & ax : axes_) {
      axis_step(*ax);
    }

    ecx_send_processdata(&g_ctx);
  }
}

void SoemPpMaster::axis_step(AxisRuntime & ax)
{
  uint16_t s = ax.cfg.slave;
  if (s < 1 || s > g_ctx.slavecount) return;
  uint8_t * out = g_ctx.slavelist[s].outputs;
  uint8_t * in = g_ctx.slavelist[s].inputs;
  if (!out || !in) return;

  uint16_t sw = (uint16_t)in[TX_SW] | ((uint16_t)in[TX_SW + 1] << 8);
  int32_t pos = (int32_t)((uint32_t)in[TX_POS] | ((uint32_t)in[TX_POS + 1] << 8) |
    ((uint32_t)in[TX_POS + 2] << 16) | ((uint32_t)in[TX_POS + 3] << 24));
  int32_t vel = (int32_t)((uint32_t)in[TX_VEL] | ((uint32_t)in[TX_VEL + 1] << 8) |
    ((uint32_t)in[TX_VEL + 2] << 16) | ((uint32_t)in[TX_VEL + 3] << 24));
  int16_t torq = (int16_t)((uint16_t)in[TX_TORQ] | ((uint16_t)in[TX_TORQ + 1] << 8));
  uint16_t er = (uint16_t)in[TX_ERR] | ((uint16_t)in[TX_ERR + 1] << 8);

  // 反馈快照(供 ROS 发布)。
  ax.fb_pos.store(pos);
  ax.fb_vel.store(vel);
  ax.fb_torq.store(torq);
  ax.fb_sw.store(sw);
  ax.fb_err.store(er);

  uint16_t cw = 0;
  int32_t tgt = pos;  // 默认保持当前位置

  if (sw & 0x0008) {  // Fault：自动复位
    if (ax.fr_low_cnt < 10) {
      cw = 0x0000;
      ax.fr_low_cnt++;
    } else {
      cw = 0x0080;
      ax.fr_low_cnt = 0;
    }
  } else if ((sw & 0x004F) == 0x0040) {
    cw = 0x0006;
  } else if ((sw & 0x006F) == 0x0021) {
    cw = 0x0007;
  } else if ((sw & 0x006F) == 0x0023) {
    cw = 0x000F;
  } else if ((sw & 0x006F) == 0x0027) {
    // Operation enabled：执行 PP 目标跟随。
    if (!ax.enabled_logged) {
      ax.enabled_logged = true;
      ax.latched_target = pos;
      printf("[AXIS%d] Op Enabled, pos=%d\n", s, pos);
    }

    if (!ax.has_target.load()) {
      // 还没收到目标，保持当前位置不动。
      cw = 0x000F;
      tgt = pos;
    } else {
      int32_t desired = ax.target_counts.load();
      tgt = desired;

      if (ax.trig_phase == 0) {
        // 预载/空闲：目标变化时进入触发流程。
        cw = 0x000F;
        if (desired != ax.latched_target || !ax.first_target_latched) {
          if (ax.preload_cnt < PP_TARGET_PRELOAD_CYCLES) {
            ax.preload_cnt++;
          } else {
            ax.trig_phase = 1;  // 预载完成，准备拉 bit4
          }
        } else {
          ax.preload_cnt = 0;
        }
      } else if (ax.trig_phase == 1) {
        // 拉高 bit4(new set-point)+bit5(立即更新)：0x003F。
        cw = 0x003F;
        if (sw & 0x1000) {  // set-point acknowledged
          ax.trig_phase = 2;
        }
      } else {
        // 释放 bit4，等驱动清 ack，本次握手完成并锁存目标。
        cw = 0x000F;
        if (!(sw & 0x1000)) {
          ax.latched_target = desired;
          ax.first_target_latched = true;
          ax.trig_phase = 0;
          ax.preload_cnt = 0;
        }
      }
    }
  } else {
    cw = 0x0006;
  }

  // 状态变化打印(限频：仅在 SW/Err 改变时)。
  if (sw != ax.prev_sw || er != ax.prev_err) {
    printf("[CHG][AXIS%d] cycle=%lu SW=0x%04X (%s) Err=0x%04X CW=0x%04X pos=%d\n",
      s, (unsigned long)cycle_, sw, cia402_state_name(sw), er, cw, pos);
    ax.prev_sw = sw;
    ax.prev_err = er;
  }

  out[RX_CW] = (uint8_t)(cw & 0xFF);
  out[RX_CW + 1] = (uint8_t)((cw >> 8) & 0xFF);
  out[RX_TARGET] = (uint8_t)(tgt & 0xFF);
  out[RX_TARGET + 1] = (uint8_t)((tgt >> 8) & 0xFF);
  out[RX_TARGET + 2] = (uint8_t)((tgt >> 16) & 0xFF);
  out[RX_TARGET + 3] = (uint8_t)((tgt >> 24) & 0xFF);
  out[RX_MODE] = MODE_PP;
}

bool SoemPpMaster::submit_waypoints(const std::vector<PpWaypoint> & waypoints)
{
  if (!running_.load() || waypoints.empty()) {
    return false;
  }
  // 取最后一个 waypoint 作为目标(低频采样持续刷新，无需逐点等待)。
  const PpWaypoint & wp = waypoints.back();
  int matched = 0;
  for (size_t i = 0; i < wp.joint_names.size() && i < wp.positions.size(); i++) {
    for (auto & ax : axes_) {
      if (ax->cfg.joint_name == wp.joint_names[i]) {
        ax->target_counts.store(rad_to_counts(ax->cfg, wp.positions[i]));
        ax->has_target.store(true);
        matched++;
        break;
      }
    }
  }
  return matched > 0;
}

bool SoemPpMaster::enabled() const
{
  return running_.load();
}

std::vector<AxisFeedback> SoemPpMaster::feedback() const
{
  std::vector<AxisFeedback> out;
  out.reserve(axes_.size());
  for (const auto & ax : axes_) {
    AxisFeedback fb;
    fb.joint_name = ax->cfg.joint_name;
    fb.position_rad = counts_to_rad(ax->cfg, ax->fb_pos.load());
    fb.velocity_rad_s = counts_to_rad_vel(ax->cfg, ax->fb_vel.load());
    fb.torque = ax->fb_torq.load();
    fb.status_word = ax->fb_sw.load();
    fb.error_code = ax->fb_err.load();
    out.push_back(std::move(fb));
  }
  return out;
}

std::size_t SoemPpMaster::soem_context_size() const
{
  return sizeof(ecx_contextt);
}

}  // namespace dual_arm_soem_bridge
