#pragma once

#include <hardware_interface/joint_state_interface.h>

namespace uwrt_hardware_interface {

/// \ref JointCommandInterface for commanding voltage-based joints.
class VoltageJointInterface : public hardware_interface::JointCommandInterface {};

}  // namespace uwrt_hardware_interface
