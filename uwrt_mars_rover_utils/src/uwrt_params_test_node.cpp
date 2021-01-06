#include <ros/console.h>
#include <ros/ros.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

static const std::string NODE_NAME{"uwrt_params_test_node"};
static const std::string CUSTOM_LOGGER_NAME{"custom_logger"};

static const std::string CUSTOM_NAMESPACE{"custom_namespace"};

static const std::string PARAM_KEY{"test_param"};
static const std::string PARAM_VALUE{"This is a test string param value!"};
static const std::string DEFAULT_PARAM_VALUE{"This is the DEFAULT string param value!"};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle nh2(nh, CUSTOM_NAMESPACE);

  // Set loggers to print debug statements
  bool success = ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + '.' + NODE_NAME,
                                                ros::console::levels::Debug);
  assert(success);
  success = ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + '.' + CUSTOM_LOGGER_NAME,
                                           ros::console::levels::Debug);
  assert(success);
  ros::console::notifyLoggerLevelsChanged();

  // Set param in nh namespace
  nh.setParam(PARAM_KEY, PARAM_VALUE);

  // Test successful param retrieval
  std::string retrieved_param =
      uwrt_mars_rover_utils::getParam<std::string>(nh, PARAM_KEY, DEFAULT_PARAM_VALUE, NODE_NAME);
  assert(PARAM_VALUE == retrieved_param);

  // Test unsuccessful param retrieval. This should return default because PARAM_KEY was not set in nh2's namespace
  retrieved_param =
      uwrt_mars_rover_utils::getParam<std::string>(nh2, PARAM_KEY, DEFAULT_PARAM_VALUE, CUSTOM_LOGGER_NAME);
  assert(DEFAULT_PARAM_VALUE == retrieved_param);

  return EXIT_SUCCESS;
}
