#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <indexer_controller/indexer_command_interface.h>
#include <indexer_controller/indexer_state_interface.h>
#include <ros/ros.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWScience : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWScience() : UWRTRoverHWScience("UWRTRoverHWScience") {}

  struct ScienceJointState {
    double pos;
    double vel;
    double eff;
  };

  struct ScienceJointCommand {
    enum class Type { NONE, POS };
    Type type;
    double data;
  };

  struct ScienceIndexerState {
    float raw_pos;
  };

  struct ScienceIndexerCommand {
    enum class Type { NONE, POS };
    Type type;
    float pos;
  };

  // Overrides
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  inline std::string getName() const {
    return name_;
  }

 protected:
  explicit UWRTRoverHWScience(std::string name) : name_(std::move(name)) {}

  // Short name for this class
  const std::string name_;

  // Joint & Indexer names
  std::vector<std::string> joint_names_;
  std::vector<std::string> indexer_names_;

  // State interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::IndexerStateInterface indexer_state_interface_;

  // Command interfaces
  hardware_interface::PositionJointInterface joint_pos_interface_;
  hardware_interface::IndexerCommandInterface indexer_cmd_interface_;

  // States and commands
  std::map<std::string, ScienceJointState> joint_states_;
  std::map<std::string, ScienceJointCommand> joint_cmds_;

  std::map<std::string, ScienceIndexerState> indexer_states_;
  std::map<std::string, ScienceIndexerCommand> indexer_cmds_;
};
}  // namespace uwrt_mars_rover_hw
