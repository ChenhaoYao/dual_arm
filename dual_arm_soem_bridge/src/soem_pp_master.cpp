#include "dual_arm_soem_bridge/soem_pp_master.hpp"

#include "soem/soem.h"

namespace dual_arm_soem_bridge
{

SoemPpMaster::SoemPpMaster() = default;

SoemPpMaster::~SoemPpMaster()
{
  stop();
}

bool SoemPpMaster::configure(const std::string & ifname)
{
  ifname_ = ifname;
  configured_ = !ifname_.empty();
  return configured_;
}

bool SoemPpMaster::start()
{
  if (!configured_) {
    return false;
  }
  running_ = true;
  return running_;
}

void SoemPpMaster::stop()
{
  running_ = false;
}

bool SoemPpMaster::submit_waypoints(const std::vector<PpWaypoint> & waypoints)
{
  return running_ && !waypoints.empty();
}

bool SoemPpMaster::enabled() const
{
  return running_;
}

std::size_t SoemPpMaster::soem_context_size() const
{
  return sizeof(ecx_contextt);
}

}  // namespace dual_arm_soem_bridge
