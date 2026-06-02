#include <memory>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "dual_arm_soem_bridge/soem_pp_master.hpp"

namespace dual_arm_soem_bridge
{

class SoemBridgeNode : public rclcpp::Node
{
public:
  SoemBridgeNode()
  : Node("soem_bridge_node"), master_(std::make_unique<SoemPpMaster>()) // 初始化列表，与master_ = std::make_unique<SoemPpMaster>();效果相同
  {
    // 参数由 launch/yaml 注入；默认 dry_run=true，避免误连真实硬件。
    ifname_ = declare_parameter<std::string>("ifname", "");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    waypoint_topic_ = declare_parameter<std::string>("waypoint_topic", "/dual_arm/pp_waypoints");
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names());

    // 订阅低频 PP waypoint，消息类型为 JointTrajectory。
    waypoint_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      waypoint_topic_, rclcpp::QoS(10),
      [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { on_waypoints(msg); });

    // 真实机械臂反馈单独发布，不覆盖仿真的 /joint_states。
    real_joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/real_joint_states", rclcpp::QoS(10));

    // 显式使能服务，真实硬件不随节点启动自动运动。
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handle_enable(request, response);
      });

    // 停止服务，后续会扩展为清空队列和安全停机。
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_stop(request, response);
      });

    // 故障复位服务，后续映射到 CiA402 fault reset。
    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_fault",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_clear_fault(request, response);
      });

    RCLCPP_INFO(
      get_logger(), "SOEM bridge ready: dry_run=%s ifname='%s' topic='%s' soem_context_size=%zu",
      dry_run_ ? "true" : "false", ifname_.c_str(), waypoint_topic_.c_str(),
      master_->soem_context_size());
  }

private:
  static std::vector<std::string> default_joint_names()
  {
    // 默认顺序：左臂 7 轴 + 右臂 7 轴。
    return {
      "laxis1_joint", "laxis2_joint", "laxis3_joint", "laxis4_joint", "laxis5_joint",
      "laxis6_joint", "laxis7_joint", "raxis1_joint", "raxis2_joint", "raxis3_joint",
      "raxis4_joint", "raxis5_joint", "raxis6_joint", "raxis7_joint"};
  }

  void on_waypoints(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    // 将 ROS JointTrajectory 转成内部 PP waypoint 结构。
    std::vector<PpWaypoint> waypoints;
    waypoints.reserve(msg->points.size());

    for (const auto & point : msg->points) {
      PpWaypoint waypoint;
      waypoint.joint_names = msg->joint_names;
      waypoint.positions = point.positions;
      waypoint.time_from_start = rclcpp::Duration(point.time_from_start).seconds();
      waypoints.push_back(std::move(waypoint));
    }

    if (dry_run_) {
      RCLCPP_INFO(
        get_logger(), "dry-run received trajectory: joints=%zu points=%zu",
        msg->joint_names.size(), msg->points.size());
      return;
    }

    // 非 dry-run 时必须先调用 enable 服务。
    if (!master_->enabled()) {
      RCLCPP_WARN(get_logger(), "trajectory ignored because SOEM master is not enabled");
      return;
    }

    if (!master_->submit_waypoints(waypoints)) {
      RCLCPP_WARN(get_logger(), "failed to submit SOEM PP waypoints");
    }
  }

  void handle_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // 请求 false 表示关闭桥接。
    if (!request->data) {
      master_->stop();
      response->success = true;
      response->message = "SOEM bridge disabled";
      return;
    }

    // dry-run 模式下不打开网卡，只验证 ROS 流程。
    if (dry_run_) {
      response->success = true;
      response->message = "dry-run enabled without opening EtherCAT";
      return;
    }

    // 真实模式必须提供网卡名。
    if (!master_->configure(ifname_)) {
      response->success = false;
      response->message = "ifname parameter is empty";
      return;
    }

    response->success = master_->start();
    response->message = response->success ? "SOEM bridge enabled" : "failed to enable SOEM bridge";
  }

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    // 当前骨架只停止主站标志。
    master_->stop();
    response->success = true;
    response->message = "SOEM bridge stopped";
  }

  void handle_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    // 真实故障复位还未实现，非 dry-run 下明确返回失败。
    response->success = dry_run_;
    response->message = dry_run_ ? "dry-run clear_fault accepted" : "clear_fault is not implemented yet";
  }

  std::unique_ptr<SoemPpMaster> master_; // 指向SoemPpMaster类型的智能指针
  std::string ifname_;
  std::string waypoint_topic_;
  std::vector<std::string> joint_names_; // 一个vector动态数组，元素类型是字符串
  bool dry_run_{true};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
};

}  // namespace dual_arm_soem_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dual_arm_soem_bridge::SoemBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
