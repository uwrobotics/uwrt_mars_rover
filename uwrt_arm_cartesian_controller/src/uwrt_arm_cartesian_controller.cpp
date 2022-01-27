#include "uwrt_arm_cartesian_controller/uwrt_arm_cartesian_controller.hpp"
#include "controller_interface/helpers.hpp"

#include "kdl_parser/kdl_parser.hpp"

#include <utility>
#include <array>

namespace uwrt_arm_cartesian_controller
{
    UWRTCartesianController::UWRTCartesianController() : m_cart_command_sub(nullptr), m_ik_solver(nullptr) {}

    CallbackReturn UWRTCartesianController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joint_names", std::vector<std::string>());
            auto_declare<std::string>("interface_name", "");
            auto_declare<std::string>("end_effector_velocity_topic", "");
            auto_declare<std::string>("robot_description", "");
            auto_declare<std::string>("root_name", "");
            auto_declare<std::string>("tip_name", "");
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UWRTCartesianController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        if (!parse_params())
        {
            return CallbackReturn::ERROR;
        }

        // Generate tree from urdf
        if (!kdl_parser::treeFromParam(m_robot_description, m_kdl_tree))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from robot description");
            return CallbackReturn::ERROR;
        }

        // Populate the KDL chain
        const bool chain_generated_successfully = m_kdl_tree.getChain(m_root_name, m_tip_name, m_kdl_chain);
        std::stringstream tree_info = "\tTree has " << m_kdl_tree.getNrOfJoints() << " joints \n\tThe joint names are:";
        for (auto it = m_kdl_chain.segments.begin(); it != m_kdl_chain.segments.end(); ++it)
        {
            tree_info << "\n\t\t" << it->getJoint().getName());
        }
        tree_info << "\n\tTree has " << m_kdl_tree.getNrOfSegments() << " segments"
                  << "\n\tThe segments are:";
        KDL::SegmentMap segment_map = m_kdl_tree.getSegments();
        for (auto it = segment_map.begin(); it != segment_map.end(); it++)
        {
            tree_info << "\n\t\t" << it->first);
        }

        if (!chain_generated_successfully)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain from tree: \n\t" << m_root_name << " --> " << m_tip_name);
            RCLCPP_ERROR(get_node()->get_logger(), tree_info.str());
            return CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "Successfully got KDL chain from tree: \n\t" << m_root_name << " --> " << m_tip_name);
        RCLCPP_INFO(get_node()->get_logger(), tree_info.str());

        m_ik_solver.reset(new KDL::ChainIkSolverVel_pinv(m_kdl_chain));

        m_cart_command_sub = get_node()->create_subscription<TwistStamped>(
            m_cart_command_topic, rclcpp::SystemDefaultsQoS(),
            [this](const TwistStamped::SharedPtr msg)
            { m_last_twist = *msg; }); // younes todo any chance msg could be null?

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    UWRTCartesianController::command_interface_configuration() const
    {
        // younes todo better understand this. what is command_interfaces_?
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto &joint : m_joint_names)
        {
            command_interfaces_config.names.push_back(joint + "/" + m_interface_name);
        }

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration
    UWRTCartesianController::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    CallbackReturn UWRTCartesianController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        //  check if we have all resources defined in the "points" parameter
        //  also verify that we *only* have the resources defined in the "points" parameter
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
        if (
            !controller_interface::get_ordered_interfaces(
                command_interfaces_, m_joint_names, m_interface_name, ordered_interfaces) ||
            command_interfaces_.size() != ordered_interfaces.size())
        {
            RCLCPP_ERROR(
                node_->get_logger(), "Expected %zu position command interfaces, got %zu", m_joint_names.size(),
                ordered_interfaces.size());
            return CallbackReturn::ERROR;
        }

        // reset command if a command came through callback when controller was inactive
        m_last_twist = get_null_twist();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn UWRTCartesianController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        for (auto &command_interface : command_interfaces_)
        {
            command_interface.set_value(0.0);
        }

        m_last_twist = get_null_twist();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type UWRTCartesianController::update(const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
    {
        // expired command
        if (time - m_last_twist.header.stamp > rclcpp::Duration(5, 0)) // younes todo make this a param
        {
            KDL::SetToZero(m_cart_vel); // younes todo this var doesnt exist and its a geometry msg not a kdl type
            for (uint8_t i = 0; i < command_interfaces_.size(); ++i)
            {
                command_interface.set_value(0.0);
            }
        }
        else
        {
            KDL::JntArray joint_positions(command_interfaces_.size());

            KDL::Twist end_effector_velocity = KDL::Twist::Zero();
            end_effector_velocity(0) = m_last_twist.twist.linear.x;
            end_effector_velocity(1) = m_last_twist.twist.linear.y;
            end_effector_velocity(2) = m_last_twist.twist.linear.z;
            end_effector_velocity(3) = m_last_twist.twist.angular.x;
            end_effector_velocity(4) = m_last_twist.twist.angular.y;
            end_effector_velocity(5) = m_last_twist.twist.angular.z;

            KDL::JntArray joint_velocities(command_interfaces_.size();

            for (uint8_t i = 0; i < command_interfaces_.size(); ++i)
            {
                joint_positions(i) = command_interafces_.at(i).getPosition();
            }

            if (m_ik_solver->CartToJnt(joint_positions, end_effector_velocity, joint_velocities) < 0)
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Unable to compute joint velocities for given end-effector velocity");
                KDL::SetToZero(m_cart_vel); // younes todo var doesnt exist anymroe
                for (uint8_t i = 0; i < command_interfaces_.size(); ++i)
                {
                    command_interface.set_value(0.0);
                }
                return controller_interface::return_type::ERROR;
            }
            
            for (uint8_t i = 0; i < command_interfaces_.size(); ++i)
            {
                command_interfaces_.at(i).setCommand(joint_velocities(i)); // younes todo use set_values
            }
        }

        return controller_interface::return_type::OK;
    }

    bool UWRTCartesianController::parse_params()
    {
        m_joint_names = node_->get_parameter("joints").as_string_array();

        if (m_joint_names.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
            return false;
        }

        // Loop over string params
        const std::array<std::pair<std::reference_wrapper<std::string>, std::string>, 5> string_params = {{m_interface_name, "interface_name"},
                                                                                                          {m_cart_command_topic, "end_effector_velocity_topic"},
                                                                                                          {m_robot_description, "robot_description"},
                                                                                                          {m_root_name, "root_name"},
                                                                                                          {m_tip_name, "tip_name"}};
        for (auto &string_param : string_params)
        {
            if (string_param.first.empty())
            {
                string_param.first = node_->get_parameter(string_param.second).as_string();
                if (string_param.first.empty())
                {
                    RCLCPP_ERROR(get_node()->get_logger(), string_param.second + " parameter was empty");
                    return false;
                }
            }
        }

        return true;
    }

    TwistStamped UWRTCartesianController::get_null_twist() const
    {
        TwistStamped ret;
        ret.twist.linear.x = 0;
        ret.twist.linear.y = 0;
        ret.twist.linear.z = 0;
        ret.twist.angular.x = 0;
        ret.twist.angular.y = 0;
        ret.twist.angular.z = 0;
        ret.header.stamp = rclcpp::Time::now();
        // younes todo whats the name of the arm's base frame ret.header.frame_id = "foo";
        return ret;
    }

} // namespace uwrt_arm_cartesian_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    uwrt_arm_cartesian_controller::UWRTCartesianController, controller_interface::ControllerInterface)