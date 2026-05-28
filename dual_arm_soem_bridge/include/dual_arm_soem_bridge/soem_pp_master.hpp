#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace dual_arm_soem_bridge
{

struct PpWaypoint
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  double time_from_start{0.0};
};

class SoemPpMaster
{
public:
  SoemPpMaster();
  ~SoemPpMaster();

  bool configure(const std::string & ifname);
  bool start();
  void stop();
  bool submit_waypoints(const std::vector<PpWaypoint> & waypoints);
  bool enabled() const;
  std::size_t soem_context_size() const;

private:
  std::string ifname_;
  bool configured_{false};
  bool running_{false};
};

}  // namespace dual_arm_soem_bridge
