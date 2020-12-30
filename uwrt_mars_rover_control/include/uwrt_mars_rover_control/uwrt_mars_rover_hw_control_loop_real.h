#pragma once

#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw_real.h"
#include "uwrt_mars_rover_hw_control_loop.h"

namespace uwrt_mars_rover_control {
class MarsRoverHWControlLoopReal : public MarsRoverHWControlLoop {
 public:
  explicit MarsRoverHWControlLoopReal(const ros::NodeHandle& nh);

  void runForeverBlocking();
};
}  // namespace uwrt_mars_rover_control
