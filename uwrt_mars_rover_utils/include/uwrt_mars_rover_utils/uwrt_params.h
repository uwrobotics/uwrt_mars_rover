#pragma once
#include <ros/ros.h>

#include <string>

namespace uwrt_mars_rover_utils {

/** getParam - Retrieves a ROS parameter and provides a default value if there's a failure
 *
 * @tparam T The type of value to load from parameter server. Must be a type supported by XMLRPC.
 * @param nh The nodehandle to use to do the parameter lookup
 * @param param_name The name of the parameter to lookup
 * @param default_value The default value to return if lookup fails
 * @param logger_name The logger name to use with rosconsole log statements
 * @return parameter from ROS parameter server, or default_value if parameter lookup fails
 */
template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& logger_name, const std::string& param_name,
           const T& default_value) {
  T retrieved_param;
  bool param_found = nh.param<T>(param_name, retrieved_param, default_value);

  const std::string absolute_parameter_name = nh.resolveName(param_name);

  if (param_found) {
    // getParam will usually be called in the init/constructor of nodes. Its useful to know what params are being used.
    ROS_INFO_STREAM_NAMED(
        logger_name, "Loaded \"" << retrieved_param << "\" from " << absolute_parameter_name << " on parameter server");
  } else {
    ROS_WARN_STREAM_NAMED(logger_name,
                          absolute_parameter_name
                              << " could not be found and loaded from parameter server. Using default value of \""
                              << default_value << "\"");
  }

  return retrieved_param;
}

/** getLoggerName - Determines a rosconsole logger name from a nodehandle. Uses the last part of a nodehandle's
 * namespace.
 *
 * @example
 * \code{.cpp}
 * ros::NodeHandle nh("/a/b/c");
 * std::string logger_name = uwrt_mars_rover_utils::getLoggerName(nh);
 * // logger_name is equal to "c"
 * \endcode
 *
 * @param nh nodehandle to derive a logger name from
 * @return derived logger name
 */
std::string getLoggerName(ros::NodeHandle& nh);

/** getLoggerName - Determines a rosconsole logger name by using the executing node's name. This is the name passed to
 * ros::init
 *
 * @return derived logger name
 */
std::string getLoggerName();

}  // namespace uwrt_mars_rover_utils
