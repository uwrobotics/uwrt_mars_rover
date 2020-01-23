#pragma once

#include "uwrt_mars_rover_hw_control_loop.h"
#include "uwrt_mars_rover_hw_drivetrain_real.h"

namespace uwrt_mars_rover_hw {
class MarsRoverHWControlLoopReal : public MarsRoverHWControlLoop {
 public:
  explicit MarsRoverHWControlLoopReal(const ros::NodeHandle& nh);

  void runForeverBlocking();
};
}  // namespace uwrt_mars_rover_hw
