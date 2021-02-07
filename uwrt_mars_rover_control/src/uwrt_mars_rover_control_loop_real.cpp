#include <uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop_real.h>
namespace uwrt_mars_rover_control
{

      MarsRoverHWControlLoopReal::MarsRoverHWControlLoopReal(const ros::NodeHandle& nh)
      MarsRoverHWControlLoop("uwrt_mars_rover_control_loop_real", nh) 
{
  if (!this->init()) 
  {
    ROS_ERROR_NAMED(name_, "Failed to initialize RobotHW");
    throw std::runtime_error("Failed to initialize RobotHW");
  }
}

void MarsRoverHWControlLoopReal::runForeverBlocking() 
{
  ros::Rate rate(control_freq_);

  while (ros::ok()) 
  {
    this->update();
    rate.sleep();
  }
}
}  // namespace uwrt_mars_rover_control
