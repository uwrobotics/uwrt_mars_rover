#include "uwrt_mars_rover_pid_api.h"
#ifdef PID_API_TESTING
#define CAN_INTERFACE "vcan0"
#else
#define CAN_INTERFACE "can0"
#endif
// will be part of new repo -------------------------------------------------------------------------
#define SET_JOINT_PID_DEADZONE 0x751
#define SET_JOINT_PID_P 0x752
#define SET_JOINT_PID_I 0x753
#define SET_JOINT_PID_D 0x754
#define SET_JOINT_PID_BIAS 0x755
enum actuatorID_t : uint8_t {
    TURNTABLE,
    SHOULDER,
    ELBOW,
    WRISTLEFT,
    WRISTRIGHT,
    CLAW
};

enum parameter_t : uint8_t {
    P,
    I,
    D,
    DEADZONE,
    BIAS
};

struct __attribute__((__packed__)) payload_st{
    float value;
    bool velocity;
    actuatorID_t actuatorID;
};

std::string stringifyActuatorID(uint8_t actuator) {
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

std::string stringifyParam(uint8_t param) {
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
// END ----------------------------------------------------------------------------------

pidApi::pidApi(const ros::NodeHandle &nh, uint8_t loop_rate, std::string log_filter) : _nh(nh), _loop_rate(loop_rate), _log_filter(log_filter) {
    CANMsg = uwrt_utils::UWRTCANWrapper("pid_api_can", CAN_INTERFACE, true);
    std::vector<uint32_t> canIDs = {SET_JOINT_PID_P, SET_JOINT_PID_I, SET_JOINT_PID_D, SET_JOINT_PID_DEADZONE, SET_JOINT_PID_BIAS};
    CANMsg.init(canIDs);
}

bool pidApi::updatePIDParam(uwrt_mars_rover_msgs::pid_api_service::Request &req, uwrt_mars_rover_msgs::pid_api_service::Response &res) {
    ROS_INFO_STREAM("Updating PID gains: {gain : " << req.request.gain << ", parameter : " << stringifyParam(req.request.parameter) 
                    << ", vel/pos : " << stringifyVelPos(req.request.vel_pos) << ", actuator : " << stringifyActuatorID(req.request.actuatorID) <<"}");
    uint32_t canID(0);
    switch (req.request.parameter) {
    case P:
        canID = SET_JOINT_PID_P;
        break;
    case I:
        canID = SET_JOINT_PID_I;
        break;
    case D:
        canID = SET_JOINT_PID_D;
        break;
    case DEADZONE:
        canID = SET_JOINT_PID_DEADZONE;
        break;
    case BIAS:
        canID = SET_JOINT_PID_BIAS;
        break;
    default:
        ROS_WARN_NAMED(_log_filter, "Invalid PID parameter. Service call failed. Message not sent");
    }
    if(req.request.actuatorID != TURNTABLE && req.request.actuatorID != SHOULDER && req.request.actuatorID !=ELBOW &&
       req.request.actuatorID != WRISTLEFT && req.request.actuatorID != WRISTRIGHT && req.request.actuatorID != CLAW) {
        ROS_WARN_NAMED(_log_filter, "Invalid actuator ID. Service call failed. Message not sent");
        res.success = false;
        return res.success;
    }
    // add some basic validation
    payload_st payload = {req.request.gain, req.request.vel_pos, (actuatorID_t)req.request.actuatorID};
    res.success = CANMsg.writeToID<payload_st>(payload, canID);
    return res.success;
}

void pidApi::run() {
  ros::ServiceServer _ss = _nh.advertiseService("pid_api", &pidApi::updatePIDParam, this);
  ROS_INFO_NAMED(_log_filter, "Ready to update PID gain paramaters.");
  while(ros::ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }
}