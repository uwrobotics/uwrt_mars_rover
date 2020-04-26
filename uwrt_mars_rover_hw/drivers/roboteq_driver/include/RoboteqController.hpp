#pragma once

#include <iostream>
#include <memory>

//#include "CommunicationInterface.hpp"
#include "CanopenInterface.hpp"
namespace roboteq {

class RoboteqController {
 public:
  explicit RoboteqController(std::unique_ptr<CanopenInterface>&& comm);
  RoboteqController(RoboteqController&&) = default;
  RoboteqController& operator=(RoboteqController&&) = default;
  RoboteqController(const RoboteqController&) = delete;
  RoboteqController& operator=(const RoboteqController&) = delete;
  ~RoboteqController() = default;

  // set methods
  bool setMotorCommand(int32_t command, uint8_t channel);
  bool setPosition(int32_t position, uint8_t channel);
  bool setVelocity(int32_t speed, uint8_t channel);
  bool setEncoderCounter(int32_t counter, uint8_t channel);
  bool setBrushlessCounter(int32_t counter, uint8_t channel);
  bool setUserIntVariable(int32_t var, uint8_t nbvar);
  bool setAcceleration(int32_t accel, uint8_t channel);
  bool setDeceleration(int32_t decel, uint8_t channel);
  bool setAllDigitalOutBits(uint8_t out_bits);
  bool setIndividualDigitalOutBits(uint8_t out_bits);
  bool resetIndividualOutBits(uint8_t out_bits);
  bool loadHomeCounter(uint8_t channel);
  bool emergencyShutdown();
  bool releaseShutdown();
  bool stopInAllModes(uint8_t channel);
  bool setPosRelative(int32_t position, uint8_t channel);
  bool setNextPosAbsolute(int32_t position, uint8_t channel);
  bool setNextPosRelative(int32_t position, uint8_t channel);
  bool setNextAcceleration(int32_t accel, uint8_t channel);
  bool setNextDeceleration(int32_t decel, uint8_t channel);
  bool setNextVelocity(int32_t speed, uint8_t channel);
  bool setUserBoolVariable(uint32_t var, uint8_t nbvar);
  bool saveConfigToFlash();

  // read methods
  int16_t readMotorAmps(uint8_t channel);
  int16_t readActualMotorCommand(uint8_t channel);
  int16_t readAppliedPowerLevel(uint8_t channel);
  int32_t readEncoderMotorSpeed(uint8_t channel);
  int32_t readAbsoluteEncoderCount(uint8_t channel);
  int32_t readAbsoluteBrushlessCounter(uint8_t channel);
  int32_t readUserIntegerVariable(int32_t nbvar);
  int16_t readEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel);
  int32_t readEncoderCountRelative(uint8_t channel);
  int32_t readBrushlessCountRelative(uint8_t channel);
  int16_t readBrushlessMotorSpeed(uint8_t channel);
  int16_t readBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel);
  int16_t readBatteryAmps(uint8_t channel);
  uint16_t readInternalVoltages(uint8_t param);
  uint32_t readAllDigitalInputs();
  int8_t readCaseAndInternalTemperatures(uint8_t param);
  int16_t readFeedback(uint8_t channel);
  uint16_t readStatusFlags();
  uint16_t readFaultFlags();
  uint16_t readCurrentDigitalOutputs();
  int32_t readClosedLoopError(uint8_t channel);
  bool readUserBooleanVariable(uint32_t nbvar);
  int32_t readInternalSerialCommand(uint8_t channel);
  int32_t readInternalAnalogCommand(uint8_t channel);
  int32_t readInternalInternalPulseCommand(uint8_t channel);
  uint32_t readTime(uint8_t param);
  uint16_t readSpektrumRadioCapture(uint8_t nb_capture);
  uint8_t readDestinationPositionReachedFlag(uint8_t channel);
  int32_t readMEMSAccelerometerAxis(uint8_t axis);
  uint8_t readMagsensorTrackDetect();
  int16_t readMagsensorTrackPosition(uint8_t nb_pulse);
  uint8_t readMagsensorMarkers(uint8_t nb_pulse);
  uint16_t readMagsensorStatus(uint8_t nb_pulse);
  uint16_t readMotorStatusFlags(uint8_t channel);

 private:
  std::unique_ptr<CanopenInterface> canopen_interface_;
};

}  // namespace roboteq
