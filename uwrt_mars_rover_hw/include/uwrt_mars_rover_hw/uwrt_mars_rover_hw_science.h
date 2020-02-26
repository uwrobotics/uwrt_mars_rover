#pragma once

#include <indexer_controller/indexer_state_interface.h>
#include <indexer_controller/indexer_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace uwrt_mars_rover_hw {
// A generic template for interfacing with the science firmware (real or simulated)
class UWRTRoverHWScience : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWScience() : UWRTRoverHWScience("uwrt_mars_rover_hw_science") {}

  // Overrides
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  inline std::string getName() const {
    return name_;
  }

 protected:
  explicit UWRTRoverHWScience(const std::string& name) : name_(std::move(name)) {}

  // Short name for this class
  const std::string name_;

  // Indexer joint names
  std::vector<std::string> indexer_names_;

  // Indexer state interfaces
  hardware_interface::IndexerStateInterface indexer_state_interface_;

  // Indexer command interfaces
  hardware_interface::IndexerCommandInterface indexer_cmd_interface_;

  // Joint states and commands
  std::map<std::string, hardware_interface::IndexerStateHandle::IndexerState> indexer_states_;
  std::map<std::string, float> indexer_cmds_;
};
}  // namespace uwrt_mars_rover_hw
