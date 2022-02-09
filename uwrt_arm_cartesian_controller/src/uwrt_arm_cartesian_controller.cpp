#include "uwrt_arm_cartesian_controller/uwrt_arm_cartesian_controller.hpp"

#include <array>
#include <utility>

#include "controller_interface/helpers.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace uwrt_arm_cartesian_controller
{
UWRTCartesianController::UWRTCartesianController()
: m_cart_command_sub(nullptr), m_ik_solver(nullptr), m_rt_buffer(nullptr)
{
}

CallbackReturn UWRTCartesianController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>());
    auto_declare<std::string>("end_effector_velocity_topic", "");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("root_name", "");
    auto_declare<std::string>("tip_name", "");
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UWRTCartesianController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!parse_params()) {
    return CallbackReturn::ERROR;
  }

  // Generate tree from urdf
  if (!kdl_parser::treeFromFile(m_robot_description, m_kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from robot description");
    return CallbackReturn::ERROR;
  }

  // Populate the KDL chain
  const bool chain_generated_successfully =
    m_kdl_tree.getChain(m_root_name, m_tip_name, m_kdl_chain);
  std::stringstream tree_info;
  tree_info << "Tree has " << m_kdl_tree.getNrOfJoints() << " joints" << std::endl;
  tree_info << "The joint names are: " << std::endl;
  for (auto it = m_kdl_chain.segments.begin(); it != m_kdl_chain.segments.end(); ++it) {
    tree_info << it->getJoint().getName() << std::endl;
  }
  tree_info << "Tree has " << m_kdl_tree.getNrOfSegments() << " segments" << std::endl;
  tree_info << "The segments are:" << std::endl;
  KDL::SegmentMap segment_map = m_kdl_tree.getSegments();
  for (auto it = segment_map.begin(); it != segment_map.end(); it++) {
    tree_info << it->first << std::endl;
  }

  if (!chain_generated_successfully) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      ("Failed to get KDL chain from tree: " + m_root_name + " --> " + m_tip_name).c_str());
    RCLCPP_ERROR(get_node()->get_logger(), tree_info.str().c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    ("Successfully got KDL chain from tree: " + m_root_name + " --> " + m_tip_name).c_str());
  RCLCPP_INFO(get_node()->get_logger(), tree_info.str().c_str());

  m_ik_solver.reset(new KDL::ChainIkSolverVel_pinv(m_kdl_chain));

  m_cart_command_sub = get_node()->create_subscription<TwistStamped>(
    m_cart_command_topic, rclcpp::SystemDefaultsQoS(), [this](const TwistStamped::SharedPtr msg) {
      m_rt_buffer.writeFromNonRT(msg);
    });  // younes todo any chance msg could be null?

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UWRTCartesianController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : m_joint_names) {
    command_interfaces_config.names.push_back(joint + "/" + COMMAND_INTERFACE);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
UWRTCartesianController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : m_joint_names) {
    state_interfaces_config.names.push_back(joint + "/" + STATE_INTERFACE);
  }

  return state_interfaces_config;
}

CallbackReturn UWRTCartesianController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  /* younes todo do i need this std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
        if (!controller_interface::get_ordered_interfaces(command_interfaces_, m_joint_names, COMMAND_INTERFACE, ordered_interfaces) ||
            command_interfaces_.size() != ordered_interfaces.size())
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Expected %zu velocity command interfaces, got %zu", m_joint_names.size(),
                ordered_interfaces.size());
            return CallbackReturn::ERROR;
        }*/

  if (
    command_interfaces_.size() != m_joint_names.size() ||
    state_interfaces_.size() != m_joint_names.size()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Expected %zu velocity command interfaces, got %zu. Expected %zu position state interfaces, "
      "got %zu.",
      m_joint_names.size(), command_interfaces_.size(), m_joint_names.size(),
      state_interfaces_.size());
    return CallbackReturn::ERROR;
  }

  // reset command if a command came through callback when controller was inactive
  m_rt_buffer.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UWRTCartesianController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  m_rt_buffer.reset();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type UWRTCartesianController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto send_zero_joint_velocities = [this]() {
    for (auto & command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
  };
  const auto end_effector_cart_vel = m_rt_buffer.readFromRT();
  // no command available or expired command
  if (
    !end_effector_cart_vel || !*end_effector_cart_vel ||
    (time - (*end_effector_cart_vel)->header.stamp) > rclcpp::Duration(5, 0)) {
    // younes todo make the duration a param
    send_zero_joint_velocities();
  } else {
    KDL::JntArray joint_positions(state_interfaces_.size());
    for (uint8_t i = 0; i < state_interfaces_.size(); ++i) {
      joint_positions(i) = state_interfaces_.at(i).get_value();
    }

    KDL::Twist end_effector_velocity = KDL::Twist::Zero();
    end_effector_velocity(0) = (*end_effector_cart_vel)->twist.linear.x;
    end_effector_velocity(1) = (*end_effector_cart_vel)->twist.linear.y;
    end_effector_velocity(2) = (*end_effector_cart_vel)->twist.linear.z;
    end_effector_velocity(3) = (*end_effector_cart_vel)->twist.angular.x;
    end_effector_velocity(4) = (*end_effector_cart_vel)->twist.angular.y;
    end_effector_velocity(5) = (*end_effector_cart_vel)->twist.angular.z;

    KDL::JntArray joint_velocities(command_interfaces_.size());

    if (m_ik_solver->CartToJnt(joint_positions, end_effector_velocity, joint_velocities) < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Unable to compute joint velocities for given end-effector velocity");
      send_zero_joint_velocities();
      return controller_interface::return_type::ERROR;
    }

    for (uint8_t i = 0; i < command_interfaces_.size(); ++i) {
      command_interfaces_.at(i).set_value(joint_velocities(i));
    }
  }
  return controller_interface::return_type::OK;
}

bool UWRTCartesianController::parse_params()
{
  m_joint_names = node_->get_parameter("joints").as_string_array();

  if (m_joint_names.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return false;
  }

  // Loop over string params
  const std::unordered_map<std::string *, const std::string> string_params = {
    {&m_cart_command_topic, "end_effector_velocity_topic"},
    {&m_robot_description, "robot_description"},
    {&m_root_name, "root_name"},
    {&m_tip_name, "tip_name"}};
  for (auto & string_param : string_params) {
    *(string_param.first) = node_->get_parameter(string_param.second).as_string();
    if (string_param.first->empty()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), (string_param.second + " parameter was empty").c_str());
      return false;
    }
  }

  return true;
}

}  // namespace uwrt_arm_cartesian_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  uwrt_arm_cartesian_controller::UWRTCartesianController, controller_interface::ControllerInterface)