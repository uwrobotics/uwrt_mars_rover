#pragma once

#include <uwrt_mars_rover_hw_science.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWScienceReal : public UWRTRoverHWScienceReal {
 public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override; 
};

} // namespace uwrt_mars_rover_hw