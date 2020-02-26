#include <indexer_controller/indexer_state_controller.h>

#include <pluginlib/class_list_macros.h>

namespace indexer_controller {

bool IndexerStateController::init(hardware_interface::IndexerStateInterface* hw, 
                                    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  
  // Get names of the indexers
  const std::vector<std::string>& indexer_names = hw->getNames();

  // Get publish period
  if(!controller_nh.getParam("publish_rate", publish_rate_)) {
    ROS_ERROR("Could not get find publish rate for IndexerStateController");
    return false;
  }

  // Setup publishers
  for(const auto& indexer_name : indexer_names) {
    indexer_states_.push_back(hw->getHandle(indexer_name));
    realtime_pubs_.push_back(std::make_shared<RtPublisher>(root_nh, indexer_name, 4));
  }

  // Set size of last publish times
  last_pub_times_.resize(indexer_names.size());
  
  return true;
}

void IndexerStateController::starting(const ros::Time& time) {
  for (auto pub_time = last_pub_times_.begin(); pub_time < last_pub_times_.end(); ++pub_time)
    *pub_time = time;
}

void IndexerStateController::update(const ros::Time& time, const ros::Duration& period) {
  for (unsigned i = 0; i < realtime_pubs_.size(); i++) {
    // Only pulished at desired rate
    if(publish_rate_ > 0.0 && last_pub_times_[i] + ros::Duration(1/publish_rate_) < time){
      // Try to publish
      if(realtime_pubs_[i]->trylock()) {
        last_pub_times_[i] = last_pub_times_[i] +  ros::Duration(1/publish_rate_);

        realtime_pubs_[i]->msg_.header.stamp = time;
        realtime_pubs_[i]->msg_.cur_index = indexer_states_[i].getCurIndex();
        realtime_pubs_[i]->msg_.raw_pos = indexer_states_[i].getRawPos();
        realtime_pubs_[i]->msg_.to_index = indexer_states_[i].getToIndex();
        realtime_pubs_[i]->msg_.from_index = indexer_states_[i].getFromIndex();

        realtime_pubs_[i]->unlockAndPublish();
      }
    }
  }
}

} // namespace indexer_controller

PLUGINLIB_EXPORT_CLASS(indexer_controller::IndexerStateController, controller_interface::ControllerBase)