/**
Copyright (c) 2020 Somesh Daga <s2daga@uwaterloo.ca>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "uwrt_arm_hw/arm_hw_real.h"
#include "uwrt_arm_hw/can_config.h"

#include <ros/ros.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace uwrt
{
namespace arm
{
ArmHWReal::ArmHWReal(const std::string& urdf_str)
  : ArmHWReal("arm_hw_real", urdf_str)
{}

ArmHWReal::ArmHWReal(const std::string& name, const std::string& urdf_str)
  : ArmHW(name, urdf_str),
    use_any_can_device_(false)
{}

bool ArmHWReal::init(ros::NodeHandle& nh,
                     ros::NodeHandle& arm_hw_nh)
{
  if (!ArmHW::init(nh, arm_hw_nh))
    return false;

  // === Initialize CAN Communications ===
  // Get the CAN Device(s) to use for arm control/feedback
  if (!use_any_can_device_)
  {
    if (!arm_hw_nh_.getParam("can_dev", can_device_))
    {
      ROS_FATAL_NAMED("arm_hw_real",
                      "[ArmHWReal] ROS Parameter 'can_dev' not found! "
                      "Failed to initialize the Arm Interface!");
      return false;
    }
  }
  // TODO(someshdaga): Consider allowing communications over all
  //                   existing CAN devices on the PC (e.g. can0 and can1)
  else
  {
    ROS_FATAL_NAMED("arm_hw_real",
                    "[ArmHWReal] Usage of 'any' CAN devices is unsupported. "
                    "Failed to initialize the Arm Interface!");
    return false;
  }

  // Initialize SocketCAN and required filters
  // TODO(someshdaga): Consider usage of CAN_BCM instead of CAN_RAW
  //                   for advanced CAN features (e.g. throttling, cycling etc)
  can_socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  snprintf(can_ifr_.ifr_name, can_device_.length() + 1,
           "%s", can_device_.c_str());
  ioctl(can_socket_handle_, SIOCGIFINDEX, &can_ifr_);
  can_address_.can_family = AF_CAN;
  can_address_.can_ifindex = can_ifr_.ifr_ifindex;
  // Set filter(s) for CAN socket
  struct can_filter filter[8];
  // Filter based on the CAN IDs associated with PC Arm Feedback
  //    - Match Condition: <received_can_id> & mask == can_id & mask
  filter[0].can_id = can_id::Get::ARM_ERROR;
  filter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[1].can_id = can_id::Get::TURNTABLE_POSITION;
  filter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[2].can_id = can_id::Get::SHOULDER_POSITION;
  filter[2].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[3].can_id = can_id::Get::ELBOW_POSITION;
  filter[3].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[4].can_id = can_id::Get::WRIST_PITCH_POSITION;
  filter[4].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[5].can_id = can_id::Get::WRIST_ROLL_POSITION;
  filter[5].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[6].can_id = can_id::Get::CLAW_POSITION;
  filter[6].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  filter[7].can_id = can_id::Get::FORCE_SENSOR_VALUE;
  filter[7].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);

  setsockopt(can_socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER,
             &filter, sizeof(filter));
  // Lastly, bind the CAN socket to the CAN address
  if (bind(can_socket_handle_, (struct sockaddr *)&can_address_, sizeof(can_address_)) < 0)
  {
    ROS_ERROR("Failed to bind to socket");
  }

  ROS_INFO_NAMED("arm_hw_real",
                 "[ArmHWReal] Successfully initialized the Real Arm Interface!");
  return true;
}

template <class T>
T ArmHWReal::convertCanData(const uint8_t* data,
                            uint8_t data_len,
                            bool swap_endianness)
{
  uint64_t out_data = 0;

  if (swap_endianness) 
    for (int i = 0; i < data_len; i++)
      out_data |= (static_cast<uint64_t>(data[i]) << (8 * (data_len - 1 - i)));
  else
    for (int i = 0; i < data_len; i++)
      out_data |= (static_cast<uint64_t>(data[i]) << (8 * i));
  
  return *(reinterpret_cast<T*>(&out_data));
}

void ArmHWReal::read(const ros::Time& time,
                     const ros::Duration& duration)
{
  // TODO(someshdaga): Implement
  // Read all messages from the socket buffer
  // struct can_frame frame;
  // int n_bytes;
  // int joint_idx;
  // do
  // {
  //   n_bytes = recv(can_socket_handle_, &frame, sizeof(struct can_frame), 0);

  //   if (n_bytes == sizeof(struct can_frame))
  //   {
  //     switch (frame.can_id)
  //     {
  //       case can_id::Get::ARM_ERROR:
  //         // Do something with this
  //         break;
  //       case can_id::Get::TURNTABLE_POSITION:
  //         // Get joint index
  //         joint_idx = joint_index_map_["arm_base_turntable_joint"];
  //         joint_position_[joint_idx] =
  //           convertCanData<double>(frame.data,
  //                                  sizeof(frame.data),
  //                                  true);
  //         ROS_WARN("[ArmHWReal] Turntable Position: %f",
  //                  joint_position_[joint_idx]);
  //         break;
  //       case can_id::Get::SHOULDER_POSITION:
  //         break;
  //       case can_id::Get::ELBOW_POSITION:
  //         break;
  //       case can_id::Get::WRIST_PITCH_POSITION:
  //         break;
  //       case can_id::Get::WRIST_ROLL_POSITION:
  //         break;
  //       default:
  //         ROS_ERROR_THROTTLE(1.0,
  //                            "[ArmHWReal][read] Unexpected CAN ID: %d",
  //                            frame.can_id);
  //     }
  //   }
  // }
  // while (n_bytes == sizeof(struct can_frame));
}

void ArmHWReal::write(const ros::Time& time,
                      const ros::Duration& duration)
{
  for (std::size_t i = 0; i < num_joints_; i++)
  {
    struct can_frame frame {};
    if (joint_names_[i] == "arm_base_turntable_joint")
      frame.can_id = can_id::Set::TURNTABLE_POSITION;
    else if (joint_names_[i] == "arm_shoulder_joint")
      frame.can_id = can_id::Set::SHOULDER_POSITION;
    else if (joint_names_[i] == "arm_elbow_joint")
      frame.can_id = can_id::Set::ELBOW_POSITION;
    else if (joint_names_[i] == "arm_wrist_pitch_joint")
      frame.can_id = can_id::Set::WRIST_PITCH_POSITION;
    else if (joint_names_[i] == "arm_wrist_roll_joint")
      frame.can_id = can_id::Set::WRIST_ROLL_POSITION;
    else
      ROS_ERROR_STREAM("Invalid joint: " << joint_names_[i]);
    frame.can_dlc = 8;

    uint64_t* data_ptr;
    switch (joint_control_method_[i])
    {
      case ControlMethod::POSITION:
        break;
      case ControlMethod::VELOCITY:
        break;
      case ControlMethod::EFFORT:
        break;
      case ControlMethod::VOLTAGE:
        *data_ptr = convertCanData<uint64_t>(frame.data, sizeof(frame.data), true);
        // data_ptr = reinterpret_cast<uint64_t*>(frame.data);
        // *data_ptr = *(reinterpret_cast<uint64_t*>(&joint_voltage_command_[i]));
        break;
      case ControlMethod::NONE:
        break;
      default:
        break;
    }

    // Switch Endianness of data
    // *data_ptr = ((*data_ptr & 0x00000000FFFFFFFFull) << 32) | ((*data_ptr & 0xFFFFFFFF00000000ull) >> 32);
    // *data_ptr = ((*data_ptr & 0x0000FFFF0000FFFFull) << 16) | ((*data_ptr & 0xFFFF0000FFFF0000ull) >> 16);
    // *data_ptr = ((*data_ptr & 0x00FF00FF00FF00FFull) << 8)  | ((*data_ptr & 0xFF00FF00FF00FF00ull) >> 8);

    writeCanFrame(frame);
  }
}

void ArmHWReal::writeCanFrame(const struct can_frame& frame)
{
  int bytes_written =
    send(can_socket_handle_, &frame, sizeof(struct can_frame), 0);

  if (bytes_written != sizeof(struct can_frame))
  {
    ROS_ERROR_STREAM("Failed to send CAN data for " <<
                     "\nID: " << frame.can_id <<
                     "\nDLC: " << static_cast<unsigned>(frame.can_dlc) <<
                     "\ndata[0]: " << static_cast<unsigned>(frame.data[0]) <<
                     "\ndata[1]: " << static_cast<unsigned>(frame.data[1]) <<
                     "\ndata[2]: " << static_cast<unsigned>(frame.data[2]) <<
                     "\ndata[3]: " << static_cast<unsigned>(frame.data[3]) <<
                     "\ndata[4]: " << static_cast<unsigned>(frame.data[4]) <<
                     "\ndata[5]: " << static_cast<unsigned>(frame.data[5]) <<
                     "\ndata[6]: " << static_cast<unsigned>(frame.data[6]) <<
                     "\ndata[7]: " << static_cast<unsigned>(frame.data[7]) <<
                     "\nBytes Written: " << bytes_written);
  }
}

void ArmHWReal::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                         const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  ArmHW::doSwitch(start_list, stop_list);

  // Switch the real hardware to the appropriate control modes
  struct can_frame frame {};
  frame.can_dlc = CAN_FRAME_SIZE_BYTES_;
  for (const auto& controller : start_list)
  {
    for (const auto& claimed : controller.claimed_resources)
    {
      if (claimed.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        frame.data[0] = 0;
      }
      else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface")
      {
        frame.data[0] = 1;
      }
      else if (claimed.hardware_interface == "hardware_interface::VoltageJointInterface")
      {
        frame.data[0] = 2;
      }
      else if (claimed.hardware_interface == "hardware_interface::JointStateInterface")
      {
        // Do nothing
      }
      else
      {
        frame.data[0] = 0;
        ROS_ERROR_NAMED("arm_hw_real",
                        "[ArmHWReal] Invalid hardware interface '%s' specified",
                        claimed.hardware_interface.c_str());
      }

      frame.can_id = can_id::Set::TURNTABLE_CONTROL_MODE;
      writeCanFrame(frame);
      frame.can_id = can_id::Set::SHOULDER_CONTROL_MODE;
      writeCanFrame(frame);
      frame.can_id = can_id::Set::ELBOW_CONTROL_MODE;
      writeCanFrame(frame);
      frame.can_id = can_id::Set::WRIST_CONTROL_MODE;
      writeCanFrame(frame);
      frame.can_id = can_id::Set::CLAW_CONTROL_MODE;
      writeCanFrame(frame);
    }
  }
}

}  // namespace arm
}  // namespace uwrt
