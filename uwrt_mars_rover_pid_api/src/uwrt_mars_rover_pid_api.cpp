#include "uwrt_mars_rover_pid_api.h"
#define LOG_FILTER "pidApi"
#ifdef PID_API_TESTING
#define CAN_INTERFACE "vcan0"
#else
#define CAN_INTERFACE "can0"
#endif
// will be part of new repo
std::string tringifyActuatorID(actuatorID_t actuator) {
    switch(actuator) {
        case(TURNTABLE): return "TURNTABLE";
        case(SHOULDER): return"SHOULDER";
        case(ELBOW): return "ELBOW";
        case(WRISTLEFT): return "WRISTLEFT";
        case(WRISTRIGHT): return "WRISTRIGHT";
        case(CLAW) : return "CLAW";
        default: return "ERROR";
    }
}

std::string HW_BRIDGE::ARM::PID::stringifyParam(parameter_t param) {
    switch(param) {
        case(P): return "P";
        case(I): return "I";
        case(D): return "D";
        case(DEADZONE): return "DEADZONE";
        case(BIAS): return "BIAS";
        default: return "ERROR";
    }
}

std::string stringifyVelPos(bool vel_pos) {
    return vel_pos ? "velocity" : "position";
}
// END

pidApi::pidApi(const ros::NodeHandle &nh, uint8_t loop_rate) : _nh(nh), _loop_rate(loop_rate) {
    CANMsg = uwrt_utils::UWRTCANWrapper("pid_api_can", CAN_INTERFACE, true);
    std::vector<int> dontmergethis;
    // fill this vector with arm board ID?
    CANMsg.init(dontmergethis);
}

void pidApi::updatePIDGain(const uwrt_mars_rover_msgs::pid_api::Request &req, uwrt_mars_rover_msgs::pid_api::Response &res) {
    ROS_INFO_NAMED(LOG_FILTER, "Updating PID gains: {gain : %f, parameter : %s, vel/pos : %s, actuator : %s}", 
                   req.gain, stringifyParam(req.parameter), stringifyVelPos(req.vel_pos), stringifyActuatorID(req.actuatorID));
    uint16_t canID = req.paramater;
    // PUT THIS IN FW_CONFIG.H
    struct __attribute__((__packed__)) payload_st{
      float value;
      bool velocity;
      uint8_t actuatorID;
    };
    // END
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