#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <std_msgs/UInt16.h>

#include <indexer_controller/indexer_command_interface.h>

namespace indexer_controller {
class IndexerCommandController : public controller_interface::Controller<hardware_interface::IndexerCommandInterface> {
public:
  IndexerCommandController() = default;
  ~IndexerCommandController() { cmd_sub_.shutdown(); }

  bool init(hardware_interface::IndexerCommandInterface* hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  hardware_interface::IndexerCommandHandle indexer_;
  realtime_tools::RealtimeBuffer<float> cmd_buffer_;

  std::vector<float> raw_pos_;
  uint16_t start_index_;

  ros::Subscriber cmd_sub_;
  void commandCallback(const std_msgs::UInt16ConstPtr& msg);
};

} //namespace indexer_controller