#include "RoboteqController.hpp"

#include <memory>

#include "CommunicationInterface.hpp"

namespace roboteq {

RoboteqController::RoboteqController(std::unique_ptr<CanopenInterface> &&comm) : canopen_interface_(std::move(comm)) {}

// set methods
bool RoboteqController::setMotorCommand(int32_t command, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_MOTOR_COMMAND, channel, command);
}

bool RoboteqController::setPosition(int32_t position, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_POSITION, channel, position);
}

bool RoboteqController::setVelocity(int32_t speed, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_VELOCITY, channel, speed);
}

bool RoboteqController::setEncoderCounter(int32_t counter, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_ENCODER_COUNTER, channel, counter);
}

bool RoboteqController::setBrushlessCounter(int32_t counter, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_BRUSHLESS_COUNTER, channel, counter);
}

bool RoboteqController::setUserIntVariable(int32_t var, uint8_t nbvar) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_USER_INT_VARIABLE, nbvar, var);
}

bool RoboteqController::setAcceleration(int32_t accel, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_ACCELERATION, channel, accel);
}

bool RoboteqController::setDeceleration(int32_t decel, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_DECELERATION, channel, decel);
}

bool RoboteqController::setAllDigitalOutBits(uint8_t out_bits) {
  return canopen_interface_->sendCommand<uint8_t>(roboteq::RuntimeCommand::SET_ALL_DIGITAL_OUT_BITS, 0, out_bits);
}

bool RoboteqController::setIndividualDigitalOutBits(uint8_t out_bits) {
  return canopen_interface_->sendCommand<uint8_t>(roboteq::RuntimeCommand::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0,
                                                  out_bits);
}

bool RoboteqController::resetIndividualOutBits(uint8_t out_bits) {
  return canopen_interface_->sendCommand<uint8_t>(roboteq::RuntimeCommand::RESET_INDIVIDUAL_OUT_BITS, 0, out_bits);
}

bool RoboteqController::loadHomeCounter(uint8_t channel) {
  return canopen_interface_->sendCommand<empty_data_payload>(roboteq::RuntimeCommand::LOAD_HOME_COUNTER, channel);
}

bool RoboteqController::emergencyShutdown() {
  return canopen_interface_->sendCommand<empty_data_payload>(roboteq::RuntimeCommand::EMERGENCY_SHUTDOWN);
}

bool RoboteqController::releaseShutdown() {
  return canopen_interface_->sendCommand<empty_data_payload>(roboteq::RuntimeCommand::RELEASE_SHUTDOWN);
}

bool RoboteqController::stopInAllModes(uint8_t channel) {
  return canopen_interface_->sendCommand<empty_data_payload>(roboteq::RuntimeCommand::STOP_IN_ALL_MODES, channel);
}

bool RoboteqController::setPosRelative(int32_t position, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_POS_RELATIVE, channel, position);
}

bool RoboteqController::setNextPosAbsolute(int32_t position, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_NEXT_POS_ABSOLUTE, channel, position);
}

bool RoboteqController::setNextPosRelative(int32_t position, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_NEXT_POS_RELATIVE, channel, position);
}

bool RoboteqController::setNextAcceleration(int32_t accel, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_NEXT_ACCELERATION, channel, accel);
}

bool RoboteqController::setNextDeceleration(int32_t decel, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_NEXT_DECELERATION, channel, decel);
}

bool RoboteqController::setNextVelocity(int32_t speed, uint8_t channel) {
  return canopen_interface_->sendCommand<int32_t>(roboteq::RuntimeCommand::SET_NEXT_VELOCITY, channel, speed);
}

bool RoboteqController::setUserBoolVariable(uint32_t var, uint8_t nbvar) {
  return canopen_interface_->sendCommand<uint32_t>(roboteq::RuntimeCommand::SET_USER_BOOL_VARIABLE, nbvar, var);
}

bool RoboteqController::saveConfigToFlash() {
  return canopen_interface_->sendCommand<empty_data_payload>(roboteq::RuntimeCommand::SAVE_CONFIG_TO_FLASH);
}

// read methods
int16_t RoboteqController::readMotorAmps(uint8_t channel) {
  return canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_MOTOR_AMPS, channel);
}

int16_t RoboteqController::readActualMotorCommand(uint8_t channel) {
  return canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_ACTUAL_MOTOR_COMMAND, channel);
}

int16_t RoboteqController::readAppliedPowerLevel(uint8_t channel) {
  return canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_ACTUAL_POWER_LEVEL, channel);
}

int32_t RoboteqController::readEncoderMotorSpeed(uint8_t channel) {
  return canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_ENCODER_MOTOR_SPEED, channel);
}

int32_t RoboteqController::readAbsoluteEncoderCount(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_ABSOLUTE_ENCODER_COUNT, channel);
}

int32_t RoboteqController::readAbsoluteBrushlessCounter(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_ABSOLUTE_BRUSHLESS_COUNTER, channel);
}

int32_t RoboteqController::readUserIntegerVariable(int32_t nbvar) {
  return canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_USER_INTEGER_VARIABLE, nbvar);
}

int16_t RoboteqController::readEncoderMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
                                         channel);
}

int32_t RoboteqController::readEncoderCountRelative(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_ENCODER_COUNT_RELATIVE, channel);
}

int32_t RoboteqController::readBrushlessCountRelative(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_BRUSHLESS_COUNT_RELATIVE, channel);
}

int16_t RoboteqController::readBrushlessMotorSpeed(uint8_t channel) {
  canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED, channel);
}

int16_t RoboteqController::readBrushlessMotorSpeedRelativeToMaxSpeed(uint8_t channel) {
  canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
                                         channel);
}

int16_t RoboteqController::readBatteryAmps(uint8_t channel) {
  return canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_BATTERY_AMPS, channel);
}

uint16_t RoboteqController::readInternalVoltages(uint8_t param) {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_INTERNAL_VOLTAGES, param);
}

uint32_t RoboteqController::readAllDigitalInputs() {
  return canopen_interface_->sendQuery<uint32_t>(roboteq::RuntimeQuery::READ_ALL_DIGITAL_INPUTS);
}

int8_t RoboteqController::readCaseAndInternalTemperatures(uint8_t param) {
  return canopen_interface_->sendQuery<int8_t>(roboteq::RuntimeQuery::READ_CASE_AND_INTERNAL_TEMPERATURES, param);
}

int16_t RoboteqController::readFeedback(uint8_t channel) {
  return canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_FEEDBACK, channel);
}

uint16_t RoboteqController::readStatusFlags() {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_STATUS_FLAGS);
}

uint16_t RoboteqController::readFaultFlags() {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_FAULT_FLAGS);
}

uint16_t RoboteqController::readCurrentDigitalOutputs() {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_CURRENT_DIGITAL_OUTPUTS);
}

int32_t RoboteqController::readClosedLoopError(uint8_t channel) {
  return canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_CLOSED_LOOP_ERROR, channel);
}

bool RoboteqController::readUserBooleanVariable(uint32_t nbvar) {
  return canopen_interface_->sendQuery<bool>(roboteq::RuntimeQuery::READ_USER_BOOLEAN_VARIABLE, nbvar);
}

int32_t RoboteqController::readInternalSerialCommand(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_INTERNAL_SERIAL_COMMAND, channel);
}

int32_t RoboteqController::readInternalAnalogCommand(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_INTERNAL_ANALOG_COMMAND, channel);
}

int32_t RoboteqController::readInternalInternalPulseCommand(uint8_t channel) {
  canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_INTERNAL_PULSE_COMMAND, channel);
}

uint32_t RoboteqController::readTime(uint8_t param) {
  return canopen_interface_->sendQuery<uint32_t>(roboteq::RuntimeQuery::READ_TIME, param);
}

uint16_t RoboteqController::readSpektrumRadioCapture(uint8_t nb_capture) {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_SPEKTRUM_RADIO_CAPTURE, nb_capture);
}

uint8_t RoboteqController::readDestinationPositionReachedFlag(uint8_t channel) {
  return canopen_interface_->sendQuery<uint8_t>(roboteq::RuntimeQuery::READ_DESTINATION_POSITION_REACHED_FLAG, channel);
}

int32_t RoboteqController::readMEMSAccelerometerAxis(uint8_t axis) {
  return canopen_interface_->sendQuery<int32_t>(roboteq::RuntimeQuery::READ_MEMS_ACCELEROMETER_AXIS, axis);
}

uint8_t RoboteqController::readMagsensorTrackDetect() {
  return canopen_interface_->sendQuery<uint8_t>(roboteq::RuntimeQuery::READ_MAGSENSOR_TRACK_DETECT);
}

int16_t RoboteqController::readMagsensorTrackPosition(uint8_t nb_pulse) {
  canopen_interface_->sendQuery<int16_t>(roboteq::RuntimeQuery::READ_MAGSENSOR_TRACK_POSITION, nb_pulse);
}

uint8_t RoboteqController::readMagsensorMarkers(uint8_t nb_pulse) {
  return canopen_interface_->sendQuery<uint8_t>(roboteq::RuntimeQuery::READ_MAGSENSOR_MARKERS, nb_pulse);
}

uint16_t RoboteqController::readMagsensorStatus(uint8_t nb_pulse) {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_MAGSENSOR_STATUS, nb_pulse);
}

uint16_t RoboteqController::readMotorStatusFlags(uint8_t channel) {
  return canopen_interface_->sendQuery<uint16_t>(roboteq::RuntimeQuery::READ_MOTOR_STATUS_FLAGS, channel);
}

}  // namespace roboteq
