/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_hw/arm_hw.h"

#include <ros/ros.h>

#include <linux/can.h>
#include <net/if.h>
#include <string>

namespace uwrt
{
namespace arm
{
class ArmHWReal : public ArmHW
{
public:
  enum FeedbackType {GENERAL, POSVEL};
  explicit ArmHWReal(const std::string& urdf_str);

  ArmHWReal(const std::string& name, const std::string& urdf_str);

  virtual bool init(ros::NodeHandle& nh,
                    ros::NodeHandle& arm_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

private:
  // CAN variables/parameters
  std::string can_device_;
  int can_socket_handle_;
  struct sockaddr_can can_address_;
  struct ifreq can_ifr_;
  const bool use_any_can_device_;

  // Utility functions
  std::string canFrameToString(const struct can_frame& frame);
  void writeCanFrame(const struct can_frame& frame);
  template <class T>
  T convertCanData(const uint8_t* data,
                   uint8_t data_len,
                   bool swap_endianness = false);
};  // class ArmHWReal
}  // namespace arm
}  // namespace uwrt
