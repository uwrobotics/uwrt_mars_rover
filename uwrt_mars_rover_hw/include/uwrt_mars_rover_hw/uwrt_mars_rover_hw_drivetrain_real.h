#pragma once

#include <uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWDrivetrainReal : public UWRTRoverHWDrivetrain {
 public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
};

}  // namespace uwrt_mars_rover_hw
