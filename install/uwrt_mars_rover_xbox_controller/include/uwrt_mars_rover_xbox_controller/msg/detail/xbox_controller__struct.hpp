// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from uwrt_mars_rover_xbox_controller:msg/XboxController.idl
// generated code does not contain a copyright notice

#ifndef UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_HPP_
#define UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__uwrt_mars_rover_xbox_controller__msg__XboxController __attribute__((deprecated))
#else
# define DEPRECATED__uwrt_mars_rover_xbox_controller__msg__XboxController __declspec(deprecated)
#endif

namespace uwrt_mars_rover_xbox_controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct XboxController_
{
  using Type = XboxController_<ContainerAllocator>;

  explicit XboxController_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivetrain_joy_x = 0.0f;
      this->drivetrain_joy_y = 0.0f;
      this->gimble_joy_x = 0.0f;
      this->gimble_joy_y = 0.0f;
      this->lt = 0.0f;
      this->rt = 0.0f;
    }
  }

  explicit XboxController_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drivetrain_joy_x = 0.0f;
      this->drivetrain_joy_y = 0.0f;
      this->gimble_joy_x = 0.0f;
      this->gimble_joy_y = 0.0f;
      this->lt = 0.0f;
      this->rt = 0.0f;
    }
  }

  // field types and members
  using _drivetrain_joy_x_type =
    float;
  _drivetrain_joy_x_type drivetrain_joy_x;
  using _drivetrain_joy_y_type =
    float;
  _drivetrain_joy_y_type drivetrain_joy_y;
  using _gimble_joy_x_type =
    float;
  _gimble_joy_x_type gimble_joy_x;
  using _gimble_joy_y_type =
    float;
  _gimble_joy_y_type gimble_joy_y;
  using _lt_type =
    float;
  _lt_type lt;
  using _rt_type =
    float;
  _rt_type rt;

  // setters for named parameter idiom
  Type & set__drivetrain_joy_x(
    const float & _arg)
  {
    this->drivetrain_joy_x = _arg;
    return *this;
  }
  Type & set__drivetrain_joy_y(
    const float & _arg)
  {
    this->drivetrain_joy_y = _arg;
    return *this;
  }
  Type & set__gimble_joy_x(
    const float & _arg)
  {
    this->gimble_joy_x = _arg;
    return *this;
  }
  Type & set__gimble_joy_y(
    const float & _arg)
  {
    this->gimble_joy_y = _arg;
    return *this;
  }
  Type & set__lt(
    const float & _arg)
  {
    this->lt = _arg;
    return *this;
  }
  Type & set__rt(
    const float & _arg)
  {
    this->rt = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> *;
  using ConstRawPtr =
    const uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__uwrt_mars_rover_xbox_controller__msg__XboxController
    std::shared_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__uwrt_mars_rover_xbox_controller__msg__XboxController
    std::shared_ptr<uwrt_mars_rover_xbox_controller::msg::XboxController_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const XboxController_ & other) const
  {
    if (this->drivetrain_joy_x != other.drivetrain_joy_x) {
      return false;
    }
    if (this->drivetrain_joy_y != other.drivetrain_joy_y) {
      return false;
    }
    if (this->gimble_joy_x != other.gimble_joy_x) {
      return false;
    }
    if (this->gimble_joy_y != other.gimble_joy_y) {
      return false;
    }
    if (this->lt != other.lt) {
      return false;
    }
    if (this->rt != other.rt) {
      return false;
    }
    return true;
  }
  bool operator!=(const XboxController_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct XboxController_

// alias to use template instance with default allocator
using XboxController =
  uwrt_mars_rover_xbox_controller::msg::XboxController_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace uwrt_mars_rover_xbox_controller

#endif  // UWRT_MARS_ROVER_XBOX_CONTROLLER__MSG__DETAIL__XBOX_CONTROLLER__STRUCT_HPP_
