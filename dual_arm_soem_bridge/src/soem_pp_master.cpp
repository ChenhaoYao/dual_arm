#include "dual_arm_soem_bridge/soem_pp_master.hpp"

#include "soem/soem.h"

namespace dual_arm_soem_bridge
{

SoemPpMaster::SoemPpMaster() = default;

SoemPpMaster::~SoemPpMaster()
{
  // 析构时兜底停止，后续真实硬件接入后可避免进程退出仍保持运行。
  stop();
}

bool SoemPpMaster::configure(const std::string & ifname)
{
  // 当前骨架只保存网卡名，后续会在这里准备 SOEM 上下文。
  ifname_ = ifname;
  configured_ = !ifname_.empty();
  return configured_;
}

bool SoemPpMaster::start()
{
  // 没有配置网卡名时不允许进入运行状态。
  if (!configured_) {
    return false;
  }
  // 当前还未打开 EtherCAT，后续会替换成 ecx_init/config_init 等流程。
  running_ = true;
  return running_;
}

void SoemPpMaster::stop()
{
  // 当前只维护状态标志，后续会加入 SAFE-OP/INIT 降级和 socket 关闭。
  running_ = false;
}

bool SoemPpMaster::submit_waypoints(const std::vector<PpWaypoint> & waypoints)
{
  // 当前只验证桥接状态，后续会写入线程安全 waypoint 队列。
  return running_ && !waypoints.empty();
}

bool SoemPpMaster::enabled() const
{
  return running_;
}

std::size_t SoemPpMaster::soem_context_size() const
{
  // 引用 SOEM 类型，确认编译时已经正确包含 SOEM。
  return sizeof(ecx_contextt);
}

}  // namespace dual_arm_soem_bridge
