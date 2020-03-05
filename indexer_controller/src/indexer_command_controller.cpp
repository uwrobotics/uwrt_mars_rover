#include <indexer_controller/indexer_command_controller.h>
#include <pluginlib/class_list_macros.h>

namespace indexer_controller {

bool IndexerCommandController::init(hardware_interface::IndexerCommandInterface* hw, ros::NodeHandle& nh) {
  // Get joint
  std::string joint_name;
  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("Did not specify joint for IndexerCommandController");
    return false;
  }
  indexer_ = hw->getHandle(joint_name);

  // Get list of raw positions
  if (!nh.getParam("raw_pos", raw_pos_)) {
    ROS_ERROR_STREAM("Did not specify raw positions for indices " << joint_name);
    return false;
  }

  // Get starting index
  int start_index;
  if (!nh.getParam("start_index_", start_index)) {
    ROS_WARN_STREAM("Did not specify a start index for " << joint_name << ", assuming 0");
    start_index = 0;
  }
  start_index_ = start_index;

  // Set up subscriber
  cmd_sub_ = nh.subscribe<std_msgs::UInt16>("command", 1, &IndexerCommandController::commandCallback, this);

  return true;
}

void IndexerCommandController::starting(const ros::Time& /*time*/) {
  cmd_buffer_.writeFromNonRT(start_index_);
}

void IndexerCommandController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  indexer_.setCommand(*cmd_buffer_.readFromRT());
}

void IndexerCommandController::commandCallback(const std_msgs::UInt16ConstPtr& msg) {
  auto new_index = (uint16_t)msg->data;
  if (new_index >= raw_pos_.size()) {
    ROS_ERROR_STREAM("Index " << new_index << " is out of bounds for " << indexer_.getName()
                              << ", not updating command");
    return;
  }

  cmd_buffer_.writeFromNonRT(raw_pos_[msg->data]);
}

}  // namespace indexer_controller

PLUGINLIB_EXPORT_CLASS(indexer_controller::IndexerCommandController, controller_interface::ControllerBase)