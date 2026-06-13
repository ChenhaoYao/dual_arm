#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace dual_arm_soem_bridge
{

// 单个 CSV waypoint，positions/velocities 使用 ROS 标准单位 rad/rad/s，顺序与 joint_names 对应。
struct CsvWaypoint
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;      // 目标位置(rad)，用于位置环反馈
  std::vector<double> velocities;     // 目标速度(rad/s)，直接下发给驱动器
  double time_from_start{0.0};
};

// 单轴标定参数：把 ROS 关节角(rad)换算成电机端编码器 counts。
// 公式：counts = direction * round(rad/(2pi) * 2^enc_bits * gear_ratio) + zero_offset_counts
struct AxisConfig
{
  std::string joint_name;            // ROS 关节名，用于和 waypoint 对齐
  uint16_t slave{0};                 // EtherCAT 从站编号(1-based)
  int enc_bits{19};                  // 编码器位数
  double gear_ratio{100.0};          // 减速比
  int direction{1};                  // +1 / -1，关节正方向与电机方向关系
  int32_t zero_offset_counts{0};     // 零点偏置(counts)
  int32_t min_counts{-2147483647};   // 软限位下限(counts)
  int32_t max_counts{2147483647};    // 软限位上限(counts)
};

// 单轴运行期状态：CiA402 状态机 + CSV 速度跟随所需的内部变量。
struct AxisRuntime
{
  AxisConfig cfg;
  std::atomic<int32_t> target_vel_counts{0};   // 最新速度命令(由 submit_waypoints 写入)
  std::atomic<bool> has_target{false};          // 是否已收到过有效目标
  // 以下仅在 RT 线程内访问，无需原子。
  bool enabled_logged{false};
  bool op_enabled{false};       // 是否已进入 Operation enabled
  int fr_low_cnt{0};       // fault reset 低电平计数
  int enable_wait_cnt{0};  // 使能后等待计数
  uint16_t prev_sw{0xFFFF};
  uint16_t prev_err{0xFFFF};
  // 反馈快照(原子，供 ROS 线程读取发布)。
  std::atomic<int32_t> fb_pos{0};
  std::atomic<int32_t> fb_vel{0};
  std::atomic<int16_t> fb_torq{0};
  std::atomic<uint16_t> fb_sw{0};
  std::atomic<uint16_t> fb_err{0};
};

// 真实电机反馈快照，供 ROS 节点发布。
struct AxisFeedback
{
  std::string joint_name;
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  int16_t torque{0};
  uint16_t status_word{0};
  uint16_t error_code{0};
};

// SOEM 主站封装类，CiA402 CSV (Cyclic Synchronous Velocity) 模式实现。
// 周期性下发速度指令，驱动器内部执行速度环。
class SoemCsvMaster
{
public:
  SoemCsvMaster();
  ~SoemCsvMaster();

  // 配置网卡名与各轴标定参数(start 之前调用)。
  bool configure(const std::string & ifname, const std::vector<AxisConfig> & axes);
  // 仅配置网卡名(兼容旧接口，使用默认空轴表)。
  bool configure(const std::string & ifname);
  // 启动/停止 SOEM 主站(打开 EtherCAT、起 RT 线程 / 降级关闭)。
  bool start();
  void stop();
  // 接收 ROS 侧 waypoint，转 counts/s 后更新每轴 pending target。
  bool submit_waypoints(const std::vector<CsvWaypoint> & waypoints);
  bool enabled() const;
  // 读取各轴真实反馈(rad)，供 ROS 发布。
  std::vector<AxisFeedback> feedback() const;
  // 设置逐个使能间隔时间(ms)。
  void set_enable_delay_ms(int delay_ms);
  // 把单个关节角(rad)换算为该轴 counts，并应用方向/零点/软限位。
  int32_t rad_to_counts(const AxisConfig & cfg, double rad) const;
  double counts_to_rad(const AxisConfig & cfg, int32_t counts) const;
  // 把速度(counts/s)换算为 rad/s(不含零点偏置)。
  double counts_to_rad_vel(const AxisConfig & cfg, int32_t counts) const;
  // 把速度(rad/s)换算为 counts/s。
  int32_t vel_to_counts(const AxisConfig & cfg, double vel_rad_s) const;
  // 用于验证 SOEM 头文件和链接是否可用。
  std::size_t soem_context_size() const;

private:
  bool ecat_bringup();    // EtherCAT 初始化
  void ecat_teardown();   // 降级到 SAFE-OP/INIT
  void rt_loop();         // RT 线程
  void axis_step(AxisRuntime & ax, int axis_idx);  // 单轴 CSV 状态机

  std::string ifname_;
  std::vector<AxisConfig> axis_configs_;
  bool configured_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> dorun_{false};        // RT 线程是否进行 PDO 交换
  std::atomic<bool> mapping_done_{false};
  std::atomic<bool> rt_should_exit_{false};

  std::vector<std::unique_ptr<AxisRuntime>> axes_;
  std::thread rt_thread_;
  int expected_wkc_{0};
  int wkc_{0};
  uint64_t cycle_{0};
  
  // 逐个使能控制
  int current_enabling_axis_{0};  // 当前正在使能的轴索引
  bool all_enabled_{false};       // 所有轴是否都已使能
  int enable_delay_ms_{100};      // 逐个使能间隔时间(ms)
};

}  // namespace dual_arm_soem_bridge
