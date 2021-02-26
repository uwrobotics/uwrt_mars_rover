#pragma once

#include <sys/types.h>
#include <uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWScienceReal : public UWRTRoverHWScience {
 public:
  explicit UWRTRoverHWScienceReal() : UWRTRoverHWScience("UWRTRoverHWScienceReal"){};
  explicit ~UWRTRoverHWScienceReal();

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  u_int8_t roboteq_canopen_id_{0};
  bool loadScienceFromParamServer(ros::NodeHandle& robot_hw_nh);

  struct __attribute__((__packed)) MotionReport {
    float position{0}, velocity{0};
  };
};

}  // namespace uwrt_mars_rover_hw
