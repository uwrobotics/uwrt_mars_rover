#include "uwrt_mars_rover_pid_api.h"
#define LOG_FILTER "pidApi"
#ifdef PID_API_TESTING
#define CAN_INTERFACE "vcan0"
#else
#define CAN_INTERFACE "can0"
#endif

pidApi::pidApi(const ros::NodeHandle &nh, uint8_t loop_rate) : _nh(nh), _loop_rate(loop_rate) {
    CANMsg = uwrt_utils::UWRTCANWrapper("pid_api_can", CAN_INTERFACE, true);
    std::vector<int> dontmergethis;
    // fill this vector with arm board ID?
    CANMsg.init(dontmergethis);
}

std::string pidApi::stringifyParam(uint8_t param) {
  switch(param) {
    case(FW_CONSTANTS::ARM::PID::P): return 'P';
    case(FW_CONSTANTS::ARM::PID::I): return'I';
    case(FW_CONSTANTS::ARM::PID::D): return 'D';
    case(FW_CONSTANTS::ARM::PID::DEADZONE): return "DEADZONE";
    case(FW_CONSTANTS::ARM::PID::BIAS): return "BIAS";
    default: return "ERROR";
  }
}

std::string pidApi::stringifyVelPos(bool vel_pos) {
  return vel_pos ? "velocity" : "position";
}

std::string pidApi::stringifyActuatorID(uint8_t actuatorID) {
  switch(actuatorID) {
    case(FW_CONSTANTS::ARM::ACTUATOR::TURNTABLE): return "TURNTABLE";
    case(FW_CONSTANTS::ARM::ACTUATOR::SHOULDER): return"SHOULDER";
    case(FW_CONSTANTS::ARM::ACTUATOR::ELBOW): return "ELBOW";
    case(FW_CONSTANTS::ARM::ACTUATOR::WRISTLEFT): return "WRISTLEFT";
    case(FW_CONSTANTS::ARM::ACTUATOR::WRISTRIGHT): return "WRISTRIGHT";
    case(FW_CONSTANTS::ARM::ACTUATOR::CLAW) : return "CLAW";
    default: return "ERROR";
  }
}

void pidApi::updatePIDGain(const uwrt_mars_rover_msgs::pid_api::Request &req, uwrt_mars_rover_msgs::pid_api::Response &res) {
    ROS_INFO_NAMED(LOG_FILTER, "Updating PID gains: {gain : %f, parameter : %s, vel/pos : %s, actuator : %s}", 
                   req.gain, stringifyParam(req.parameter), stringifyVelPos(req.vel_pos), stringifyActuatorID(req.actuatorID));
    uint16_t canID = req.paramater;
    // DO NOT MERGE: this struct is declared in the fw and the sw. should it be in the interface config file
    struct __attribute__((__packed__)) payload_st{
      float value;
      bool velocity;
      uint8_t actuatorID;
    };
    payload_st payload = {req.gain, req.vel_pos, req.actuatorID};
    res.success = CANMsg.writeToID<payload_st>(payload, canID);
}

void pidApi::run() {
  _ss = _nh.advertiseService("pid_api", &pidApi::updatePIDGain, this);
  ROS_INFO_NAMED(LOG_FILTER, "Ready to update PID gain paramaters.");
  while(ros::ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }
}