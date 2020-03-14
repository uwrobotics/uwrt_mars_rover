#pragma once

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>

#include <indexer_controller/IndexerData.h>
#include <indexer_controller/indexer_state_interface.h>

namespace indexer_controller {

class IndexerStateController : public controller_interface::Controller<hardware_interface::IndexerStateInterface> {
 public:
  IndexerStateController() = default;

  bool init(hardware_interface::IndexerStateInterface* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  using RtPublisher = realtime_tools::RealtimePublisher<indexer_controller::IndexerData>;

  std::vector<hardware_interface::IndexerStateHandle> indexer_states_;
  std::vector<std::shared_ptr<RtPublisher>> realtime_pubs_;
  std::vector<ros::Time> last_pub_times_;
  std::vector<std::vector<float>> raw_pos_lists_;
  double publish_rate_{};
};

}  // namespace indexer_controller
