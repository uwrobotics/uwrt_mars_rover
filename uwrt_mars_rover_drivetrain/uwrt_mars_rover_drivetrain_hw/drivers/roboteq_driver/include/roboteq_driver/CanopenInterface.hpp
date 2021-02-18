#pragma once

#include <linux/can.h>

#include <unordered_map>
#include <uwrt_mars_rover_utils/uwrt_can.h>

using namespace uwrt_mars_rover_utils;

namespace roboteq {
using empty_data_payload = std::nullptr_t;

enum class RuntimeCommand {
  SET_MOTOR_COMMAND = 0,
  SET_POSITION,
  SET_VELOCITY,
  SET_ENCODER_COUNTER,
  SET_BRUSHLESS_COUNTER,
  SET_USER_INT_VARIABLE,
  SET_ACCELERATION,
  SET_DECELERATION,
  SET_ALL_DIGITAL_OUT_BITS,
  SET_INDIVIDUAL_DIGITAL_OUT_BITS,
  RESET_INDIVIDUAL_OUT_BITS,
  LOAD_HOME_COUNTER,
  EMERGENCY_SHUTDOWN,
  RELEASE_SHUTDOWN,
  STOP_IN_ALL_MODES,
  SET_POS_RELATIVE,
  SET_NEXT_POS_ABSOLUTE,
  SET_NEXT_POS_RELATIVE,
  SET_NEXT_ACCELERATION,
  SET_NEXT_DECELERATION,
  SET_NEXT_VELOCITY,
  SET_USER_BOOL_VARIABLE,
  SAVE_CONFIG_TO_FLASH,
};

enum class RuntimeQuery {

  READ_MOTOR_AMPS = 0,
  READ_ACTUAL_MOTOR_COMMAND,
  READ_ACTUAL_POWER_LEVEL,
  READ_ENCODER_MOTOR_SPEED,
  READ_ABSOLUTE_ENCODER_COUNT,
  READ_ABSOLUTE_BRUSHLESS_COUNTER,
  READ_USER_INTEGER_VARIABLE,
  READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
  READ_ENCODER_COUNT_RELATIVE,
  READ_BRUSHLESS_COUNT_RELATIVE,
  READ_BRUSHLESS_MOTOR_SPEED,
  READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED,
  READ_BATTERY_AMPS,
  READ_INTERNAL_VOLTAGES,
  READ_ALL_DIGITAL_INPUTS,
  READ_CASE_AND_INTERNAL_TEMPERATURES,
  READ_FEEDBACK,
  READ_STATUS_FLAGS,
  READ_FAULT_FLAGS,
  READ_CURRENT_DIGITAL_OUTPUTS,
  READ_CLOSED_LOOP_ERROR,
  READ_USER_BOOLEAN_VARIABLE,
  READ_INTERNAL_SERIAL_COMMAND,
  READ_INTERNAL_ANALOG_COMMAND,
  READ_INTERNAL_PULSE_COMMAND,
  READ_TIME,
  READ_SPEKTRUM_RADIO_CAPTURE,
  READ_DESTINATION_POSITION_REACHED_FLAG,
  READ_MEMS_ACCELEROMETER_AXIS,
  READ_MAGSENSOR_TRACK_DETECT,
  READ_MAGSENSOR_TRACK_POSITION,
  READ_MAGSENSOR_MARKERS,
  READ_MAGSENSOR_STATUS,
  READ_MOTOR_STATUS_FLAGS,
  READ_INDIVIDUAL_DIGITAL_INPUTS,
  READ_ANALOG_INPUTS,
  READ_ANALOG_INPUTS_CONVERTED,
  READ_PULSE_INPUTS,
  READ_PULSE_INPUTS_CONVERTED,
};

class CanopenInterface {
 public:
  explicit CanopenInterface(canid_t roboteq_can_id = 0x1, const std::string& ifname = "can0");
  CanopenInterface(CanopenInterface&&) = default;
  CanopenInterface& operator=(CanopenInterface&&) = default;
  CanopenInterface(const CanopenInterface&) = default;
  CanopenInterface& operator=(const CanopenInterface&) = default;
  ~CanopenInterface() = default;

  template <typename DataType>
  bool sendCommand(RuntimeCommand command, uint8_t subindex = 0, DataType data = nullptr);

  template <typename DataType>
  DataType sendQuery(RuntimeQuery query, uint8_t subindex = 0);

 private:
  static inline constexpr unsigned bytesToBits(const unsigned num_bytes) {
    constexpr unsigned BITS_PER_BYTE = 8;
    return num_bytes * BITS_PER_BYTE;
  }

  UWRTCANWrapper wrapper_;

  int roboteq_can_id_;
  int socket_handle_;

  static const std::unordered_map<RuntimeCommand, uint16_t> COMMAND_CANOPEN_ID_MAP_;
  static const std::unordered_map<RuntimeQuery, uint16_t> QUERY_CANOPEN_ID_MAP_;

  static constexpr uint16_t SDO_COMMAND_ID{2};
  static constexpr uint16_t SDO_QUERY_ID{4};
  static constexpr uint16_t SDO_COB_ID_OFFSET{0x600};
  static constexpr uint16_t SDO_RESPONSE_COB_ID_OFFSET{0x580};
  static constexpr uint8_t SDO_MAX_DATA_SIZE{4};
  static constexpr uint8_t CAN_FRAME_SIZE_BYTES{8};

  static constexpr uint8_t UNUSED_BYTES_MASK{0x3 << 2};
  static constexpr uint8_t RESPONSE_TYPE_MASK{0xF << 4};
  static constexpr uint8_t SUCCESSFUL_COMMAND_RESPONSE{6 << 4};
  static constexpr uint8_t SUCCESSFUL_QUERY_RESPONSE{4 << 4};
};

template <>
bool CanopenInterface::sendCommand<empty_data_payload>(RuntimeCommand command, uint8_t subindex, empty_data_payload);

}  // namespace roboteq
