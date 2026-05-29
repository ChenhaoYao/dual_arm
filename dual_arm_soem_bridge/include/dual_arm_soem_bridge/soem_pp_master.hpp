#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace dual_arm_soem_bridge
{

// 单个 PP waypoint，positions 使用 ROS 标准单位 rad。
struct PpWaypoint
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  double time_from_start{0.0};
};

// SOEM 主站封装类，后续把 ec_sample.c 的 PP 控制逻辑迁移到这里。
class SoemPpMaster
{
public:
  SoemPpMaster();
  ~SoemPpMaster();

  // 配置 EtherCAT 网卡名。
  bool configure(const std::string & ifname);
  // 启动/停止 SOEM 主站。
  bool start();
  void stop();
  // 接收 ROS 侧转换后的 waypoint 队列。
  bool submit_waypoints(const std::vector<PpWaypoint> & waypoints);
  bool enabled() const;
  // 用于验证 SOEM 头文件和链接是否可用。
  std::size_t soem_context_size() const;

private:
  std::string ifname_;
  bool configured_{false};
  bool running_{false};
};

}  // namespace dual_arm_soem_bridge
