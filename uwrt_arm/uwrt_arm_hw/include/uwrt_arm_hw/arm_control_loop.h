/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_hw/arm_hw.h"

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include <string>

namespace uwrt
{
namespace arm
{
/**
 * Base class for Arm control loops that perform reads, writes and updates of controllers
 */
class ArmControlLoop
{
public:
  /**
   * \brief Constructor
   * 
   * \param name Name for the ArmControlLoop object
   * \param nh ROS Nodehandle for root of the arm namespace 
   */
  ArmControlLoop(std::string name,
                 const ros::NodeHandle& nh = ros::NodeHandle());
  /**
   * \brief Destructor
   */
  virtual ~ArmControlLoop() = default;

  /**
   * \brief Initializer for the class  
   */
  virtual bool init();

  /**
   * \brief Performs reads/writes and updates controllers
   * 
   * Reads and writes to the hardware are performed at each call to this function. The controller manager
   * updates the state of the controllers when specified
   * 
   * \param time_now The current ROS time
   * \param update_controllers Whether to update the controller states (e.g. start, stop, switch, etc)
   */
  void update(const ros::Time& time_now, bool update_controllers = true);

protected:
  // Short name of this class
  const std::string name_;  ///< Name of the control loop object

  // Handle at root of robot's namespace
  ros::NodeHandle nh_;  ///< Nodehandle

  // ArmHW instance
  std::unique_ptr<ArmHW> arm_hw_;  ///< Pointer to the  hardware interface for the Arm */

  // Controller Manager
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;  ///< Manages all active and inactive controllers  */

  // Timing
  ros::Time last_update_time_;  ///< The last time the controller manager updated the state of the controllers
  ros::Time last_rw_time_;  ///< The last time data was read from or written to the hardware interface
  double    controller_watchdog_timeout_;  ///< Watchdog timer timeout for the active controller(s)
  double    control_freq_;  ///< Frequency of controller updates
};  // class ArmControlLoop
}  // namespace arm
}  // namespace uwrt
