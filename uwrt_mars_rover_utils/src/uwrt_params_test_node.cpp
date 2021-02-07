#include <ros/console.h>
#include <ros/ros.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

static const std::string NODE_NAME{"uwrt_params_test_node"};
static const std::string CUSTOM_LOGGER_NAME{"custom_logger"};

static const std::string CUSTOM_NAMESPACE{"custom_namespace"};

static const std::string PARAM_KEY{"test_param"};
static const std::string PARAM_VALUE{"This is a test string param value!"};
static const std::string DEFAULT_PARAM_VALUE{"This is the DEFAULT string param value!"};

void getParamTest();
void getLoggerNameTest();

void getParamTest() 
{
  ros::NodeHandle nh;
  ros::NodeHandle nh2(nh, CUSTOM_NAMESPACE);

  // Set param in nh namespace
  nh.setParam(PARAM_KEY, PARAM_VALUE);

  // Test successful param retrieval
  std::string retrieved_param =
      uwrt_mars_rover_utils::getParam<std::string>(nh, NODE_NAME, PARAM_KEY, DEFAULT_PARAM_VALUE);
  assert(PARAM_VALUE == retrieved_param);

  // Test unsuccessful param retrieval w/ custom logger name
  // This should return default because PARAM_KEY was not set in nh2's namespace
  retrieved_param =
      uwrt_mars_rover_utils::getParam<std::string>(nh2, CUSTOM_LOGGER_NAME, PARAM_KEY, DEFAULT_PARAM_VALUE);
  assert(DEFAULT_PARAM_VALUE == retrieved_param);
}

void getLoggerNameTest() 
{
  ros::NodeHandle nh;
  ros::NodeHandle nh2(nh, CUSTOM_NAMESPACE);

  // NodeHandles with a global root namespace will return an "UNNAMED_LOGGER"
  std::string logger_name{uwrt_mars_rover_utils::getLoggerName(nh)};
  assert("UNNAMED_LOGGER_0" == logger_name);

  // All other NodeHandles will return the last part of the namespace as the logger
  logger_name = uwrt_mars_rover_utils::getLoggerName(nh2);
  assert(CUSTOM_NAMESPACE == logger_name);

  // Subsequent calls to getLoggerName with global root namespaces will increment the number after "UNNAMED_LOGGER".
  logger_name = uwrt_mars_rover_utils::getLoggerName(nh);
  assert("UNNAMED_LOGGER_1" == logger_name);

  logger_name = uwrt_mars_rover_utils::getLoggerName(nh);
  assert("UNNAMED_LOGGER_2" == logger_name);

  // Should use node executable name when called without a nodehandle
  logger_name = uwrt_mars_rover_utils::getLoggerName();
  assert(NODE_NAME == logger_name);
}

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, NODE_NAME);

  getParamTest();
  getLoggerNameTest();

  return EXIT_SUCCESS;
}
