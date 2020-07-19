/**
Copyright (c) 2019 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#pragma once

#include "uwrt_arm_hw/voltage_joint_interface.h"

#include <forward_command_controller/forward_command_controller.h>

namespace voltage_controllers
{
/**
 * \brief Joint Voltage Controller
 *
 * This class passes the commanded voltage/pwm to the joint
 *
 * \section ROS interface
 *
 * \param type Must be "JointVoltageController".
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64) : The joint voltage/pwm to apply in range [-1, 1]
 */
typedef forward_command_controller::ForwardCommandController<hardware_interface::VoltageJointInterface>
        JointVoltageController;

}