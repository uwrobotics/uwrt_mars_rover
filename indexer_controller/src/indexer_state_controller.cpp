#include <indexer_controller/indexer_state_controller.h>
#include <algorithm>

#include <pluginlib/class_list_macros.h>

namespace indexer_controller {

bool IndexerStateController::init(hardware_interface::IndexerStateInterface* hw, ros::NodeHandle& root_nh,
                                  ros::NodeHandle& controller_nh) {
  // Get names of the indexers
  const std::vector<std::string>& indexer_names = hw->getNames();

  // Get publish period
  if (!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Could not get find publish rate for IndexerStateController");
    return false;
  }

  // Setup publishers
  for (const auto& indexer_name : indexer_names) {
    indexer_states_.push_back(hw->getHandle(indexer_name));
    realtime_pubs_.push_back(std::make_shared<RtPublisher>(root_nh, indexer_name, 4));

    // get list of raw positions from other controllers
    std::vector<float> raw_pos_list;
    std::string param_name = indexer_name + "_controller/raw_pos";
    if (!root_nh.getParam(param_name, raw_pos_list)) {
      ROS_ERROR_STREAM("Could not find list of raw positions for "
                       << indexer_name << ". Please make sure it follows the proper naming conventions " << param_name);
      return false;
    } else if (raw_pos_list.size() <= 1) {
      ROS_ERROR_STREAM("Indexer must have at least positions");
      return false;
    }
    raw_pos_lists_.push_back(raw_pos_list);
  }

  // Set size of last publish times
  last_pub_times_.resize(indexer_names.size());

  return true;
}

void IndexerStateController::starting(const ros::Time& time) {
  for (auto pub_time = last_pub_times_.begin(); pub_time < last_pub_times_.end(); ++pub_time) {
    *pub_time = time;
  }
}

void IndexerStateController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  for (unsigned i = 0; i < realtime_pubs_.size(); i++) {
    // Only pulished at desired rate
    if (publish_rate_ > 0.0 && last_pub_times_[i] + ros::Duration(1 / publish_rate_) < time) {
      // calculate high and low index
      int low_index = -1;
      int high_index = 0;
      float raw_pos = indexer_states_[i].getRawPos();
      for (auto idx_pos : raw_pos_lists_[i]) {
        if (raw_pos < idx_pos) {
          break;
        }
        low_index++;
        high_index++;
      }

      float raw_del = raw_pos - raw_pos_lists_[i][std::max(low_index, (int)0)];
      float index_del = raw_pos_lists_[i][std::min(std::max(high_index, 1), (int)(raw_pos_lists_[i].size() - 1))] -
                        raw_pos_lists_[i][std::min(std::max(low_index, 0), (int)(raw_pos_lists_[i].size() - 2))];

      // Try to publish
      if (realtime_pubs_[i]->trylock()) {
        last_pub_times_[i] = last_pub_times_[i] + ros::Duration(1 / publish_rate_);

        realtime_pubs_[i]->msg_.header.stamp = time;
        realtime_pubs_[i]->msg_.raw_pos = raw_pos;
        realtime_pubs_[i]->msg_.high_index = high_index;
        realtime_pubs_[i]->msg_.low_index = low_index;
        realtime_pubs_[i]->msg_.percent_indexed = (raw_del / index_del) * 100.0;

        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

}  // namespace indexer_controller

PLUGINLIB_EXPORT_CLASS(indexer_controller::IndexerStateController, controller_interface::ControllerBase)