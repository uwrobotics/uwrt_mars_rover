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
/**
 * Hardware Interface for the real Arm
 */
class ArmHWReal : public ArmHW
{
public:
  /**
   * \brief Enum to distinguish between the type of feedback received over the CAN interface
   */
  enum FeedbackType {GENERAL, POSVEL};
  /**
   * \brief Constructor
   * 
   * \param urdf_str URDF of the arm in string format
   */
  explicit ArmHWReal(const std::string& urdf_str);

  /**
   * \brief: Constructor
   * 
   * \param name Arbitrary name for an instance of this class
   * \param urdf_str URDF of the arm in string format
   */
  ArmHWReal(const std::string& name, const std::string& urdf_str);

  /**
   * \brief Initializes the hardware interface by registering all joints and interfaces
   * 
   * \param nh Nodehandle for the root of the arm namespace
   * \param arm_hw_nh Nodehandle in the namespace of the arm hardware interface
   */ 
  virtual bool init(ros::NodeHandle& nh,
                    ros::NodeHandle& arm_hw_nh);
  /**
   * \brief Reads data from the arm
   * 
   * \param time The current time
   * \param period The time passes since last call to read
   */
  virtual void read(const ros::Time& time, const ros::Duration& period);
  /**
   * \brief Writes data to the arm
   * 
   * \param time The current time
   * \param period The time passes since last call to write
   */
  virtual void write(const ros::Time& time, const ros::Duration& period);
  /**
   * \brief Starts/Stops controllers based on controller manager service calls
   * 
   * \param start_list Controllers to start
   * \param stop_list Controllers to stop
   */
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

private:
  // CAN variables/parameters
  std::string can_device_;  ///< The CAN device file descriptor
  int can_socket_handle_;
  struct sockaddr_can can_address_;
  struct ifreq can_ifr_;
  const bool use_any_can_device_;  ///< Read from all can interfaces on the pc

  // Utility functions
  /**
   * \brief Converts CAN frame to a string
   * 
   * \param frame The CAN frame
   */
  std::string canFrameToString(const struct can_frame& frame);
  /**
   * \brief Writes CAN frame to the CAN interface(s)
   * 
   * \param frame The CAN frame to write
   */
  void writeCanFrame(const struct can_frame& frame);
  /**
   * Converts data from its byte represetation to the specified templated type
   * 
   * \tparam T Output format of the byte representation of the data
   * \param data Pointer to the first byte of the data
   * \param data_len The number of bytes constituting the data
   * \param swap_endianess Swap the endianness of the input data
   */
  template <class T>
  T convertCanData(const uint8_t* data,
                   uint8_t data_len,
                   bool swap_endianness = false);
};  // class ArmHWReal
}  // namespace arm
}  // namespace uwrt
