#pragma once

#include <uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWScienceReal : public UWRTRoverHWScience {
 public:
  explicit UWRTRoverHWScienceReal() : UWRTRoverHWScience("UWRTRoverHWScienceReal"){};

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
};

}  // namespace uwrt_mars_rover_hw