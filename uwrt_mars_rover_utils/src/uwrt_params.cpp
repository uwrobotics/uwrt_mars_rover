#include <uwrt_mars_rover_utils/uwrt_params.h>

namespace uwrt_mars_rover_utils {
std::string getLoggerName(ros::NodeHandle& nh) {
  const std::string& nh_namespace{nh.getNamespace()};
  std::size_t start_of_name_index = nh_namespace.rfind('/') + 1;

  std::string logger_name = nh_namespace.substr(start_of_name_index);
  if (logger_name.empty()) {
    static unsigned number_of_unnamed_loggers = 0;
    std::string fallback_logger_name{"UNNAMED_LOGGER_" + std::to_string(number_of_unnamed_loggers)};
    ROS_ERROR_STREAM_NAMED("uwrt_params",
                           "Failed to construct a valid logger name from NodeHandle. Using fallback of \""
                               << fallback_logger_name << "\"");

    logger_name = fallback_logger_name;
    number_of_unnamed_loggers++;
  }

  return logger_name;
}
}  // namespace uwrt_mars_rover_utils
