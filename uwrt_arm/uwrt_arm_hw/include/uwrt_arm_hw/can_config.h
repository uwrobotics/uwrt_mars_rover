#pragma once

#include <linux/can.h>

namespace uwrt
{
namespace arm
{
const uint8_t CAN_FRAME_SIZE_BYTES_ = 8;
namespace can_id
{
struct Get
{
  static const canid_t ARM_ERROR              = 0x100;

  static const canid_t TURNTABLE_FEEDBACK     = 0x758;
  static const canid_t SHOULDER_FEEDBACK      = 0x759;
  static const canid_t ELBOW_FEEDBACK         = 0x75A;
  static const canid_t WRIST_PITCH_FEEDBACK   = 0x75B;
  static const canid_t WRIST_ROLL_FEEDBACK    = 0x75C;
  static const canid_t CLAW_FEEDBACK          = 0x75D;
  static const canid_t FORCE_SENSOR_VALUE     = 0x75E;
};

struct Set
{
  static const canid_t TURNTABLE_CONTROL_MODE = 0x740;
  static const canid_t SHOULDER_CONTROL_MODE  = 0x741;
  static const canid_t ELBOW_CONTROL_MODE     = 0x742;
  static const canid_t WRIST_CONTROL_MODE     = 0x743;
  static const canid_t CLAW_CONTROL_MODE      = 0x744;

  static const canid_t TURNTABLE_COMMAND      = 0x745;
  static const canid_t SHOULDER_COMMAND       = 0x746;
  static const canid_t ELBOW_COMMAND          = 0x747;
  static const canid_t WRIST_PITCH_COMMAND    = 0x748;
  static const canid_t WRIST_ROLL_COMMAND     = 0x749;
  static const canid_t CLAW_COMMAND           = 0x74A;
  static const canid_t TOOLTIP_DEPLOY         = 0x74B;

  static const canid_t PID_TUNING_MODE        = 0x750;
  static const canid_t PID_DEADZONE           = 0x751;
  static const canid_t PID_P                  = 0x752;
  static const canid_t PID_I                  = 0x753;
  static const canid_t PID_D                  = 0x754;
  static const canid_t PID_BIAS               = 0x755;
};

struct Calibrate
{
  static const canid_t WRIST                  = 0x74C;
  static const canid_t CLAW                   = 0x74D;
};

}
}
}