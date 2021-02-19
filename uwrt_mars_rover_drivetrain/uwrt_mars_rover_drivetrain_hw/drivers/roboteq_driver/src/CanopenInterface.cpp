#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <ros/console.h>
#include <sys/ioctl.h>

#include <roboteq_driver/CanopenInterface.hpp>
#include <uwrt_mars_rover_utils/uwrt_can.h>

using namespace uwrt_mars_rover_utils;

namespace roboteq {

const std::unordered_map<RuntimeCommand, uint16_t> CanopenInterface::COMMAND_CANOPEN_ID_MAP_ = {

    {RuntimeCommand::SET_MOTOR_COMMAND, 0x2000},
    {RuntimeCommand::SET_POSITION, 0x2001},
    {RuntimeCommand::SET_VELOCITY, 0x2002},
    {RuntimeCommand::SET_ENCODER_COUNTER, 0x2003},
    {RuntimeCommand::SET_BRUSHLESS_COUNTER, 0x2004},
    {RuntimeCommand::SET_USER_INT_VARIABLE, 0x2005},
    {RuntimeCommand::SET_ACCELERATION, 0x2006},
    {RuntimeCommand::SET_DECELERATION, 0x2007},
    {RuntimeCommand::SET_ALL_DIGITAL_OUT_BITS, 0x2008},
    {RuntimeCommand::SET_INDIVIDUAL_DIGITAL_OUT_BITS, 0x2009},
    {RuntimeCommand::RESET_INDIVIDUAL_OUT_BITS, 0x200a},
    {RuntimeCommand::LOAD_HOME_COUNTER, 0x200b},
    {RuntimeCommand::EMERGENCY_SHUTDOWN, 0x200c},
    {RuntimeCommand::RELEASE_SHUTDOWN, 0x200d},
    {RuntimeCommand::STOP_IN_ALL_MODES, 0x200e},
    {RuntimeCommand::SET_POS_RELATIVE, 0x200f},
    {RuntimeCommand::SET_NEXT_POS_ABSOLUTE, 0x2010},
    {RuntimeCommand::SET_NEXT_POS_RELATIVE, 0x2011},
    {RuntimeCommand::SET_NEXT_ACCELERATION, 0x2012},
    {RuntimeCommand::SET_NEXT_DECELERATION, 0x2013},
    {RuntimeCommand::SET_NEXT_VELOCITY, 0x2014},
    {RuntimeCommand::SET_USER_BOOL_VARIABLE, 0x2015},
    {RuntimeCommand::SAVE_CONFIG_TO_FLASH, 0x2017},
};

const std::unordered_map<RuntimeQuery, uint16_t> CanopenInterface::QUERY_CANOPEN_ID_MAP_ = {
    {RuntimeQuery::READ_MOTOR_AMPS, 0x2100},
    {RuntimeQuery::READ_ACTUAL_MOTOR_COMMAND, 0x2101},
    {RuntimeQuery::READ_ACTUAL_POWER_LEVEL, 0x2102},
    {RuntimeQuery::READ_ENCODER_MOTOR_SPEED, 0x2103},
    {RuntimeQuery::READ_ABSOLUTE_ENCODER_COUNT, 0x2104},
    {RuntimeQuery::READ_ABSOLUTE_BRUSHLESS_COUNTER, 0x2105},
    {RuntimeQuery::READ_USER_INTEGER_VARIABLE, 0x2106},
    {RuntimeQuery::READ_ENCODER_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, 0x2107},
    {RuntimeQuery::READ_ENCODER_COUNT_RELATIVE, 0x2108},
    {RuntimeQuery::READ_BRUSHLESS_COUNT_RELATIVE, 0x2109},
    {RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED, 0x210a},
    {RuntimeQuery::READ_BRUSHLESS_MOTOR_SPEED_RELATIVE_TO_MAX_SPEED, 0x210b},
    {RuntimeQuery::READ_BATTERY_AMPS, 0x210c},
    {RuntimeQuery::READ_INTERNAL_VOLTAGES, 0x210D},
    {RuntimeQuery::READ_ALL_DIGITAL_INPUTS, 0x210e},
    {RuntimeQuery::READ_CASE_AND_INTERNAL_TEMPERATURES, 0x210f},
    {RuntimeQuery::READ_FEEDBACK, 0x2110},
    {RuntimeQuery::READ_STATUS_FLAGS, 0x2111},
    {RuntimeQuery::READ_FAULT_FLAGS, 0x2112},
    {RuntimeQuery::READ_CURRENT_DIGITAL_OUTPUTS, 0x2113},
    {RuntimeQuery::READ_CLOSED_LOOP_ERROR, 0x2114},
    {RuntimeQuery::READ_USER_BOOLEAN_VARIABLE, 0x2115},
    {RuntimeQuery::READ_INTERNAL_SERIAL_COMMAND, 0x2116},
    {RuntimeQuery::READ_INTERNAL_ANALOG_COMMAND, 0x2117},
    {RuntimeQuery::READ_INTERNAL_PULSE_COMMAND, 0x2118},
    {RuntimeQuery::READ_TIME, 0x2119},
    {RuntimeQuery::READ_SPEKTRUM_RADIO_CAPTURE, 0x211a},
    {RuntimeQuery::READ_DESTINATION_POSITION_REACHED_FLAG, 0x211b},
    {RuntimeQuery::READ_MEMS_ACCELEROMETER_AXIS, 0x211c},
    {RuntimeQuery::READ_MAGSENSOR_TRACK_DETECT, 0x211d},
    {RuntimeQuery::READ_MAGSENSOR_TRACK_POSITION, 0x211e},
    {RuntimeQuery::READ_MAGSENSOR_MARKERS, 0x211f},
    {RuntimeQuery::READ_MAGSENSOR_STATUS, 0x2120},
    {RuntimeQuery::READ_MOTOR_STATUS_FLAGS, 0x2122},
};

CanopenInterface::CanopenInterface(canid_t roboteq_can_id, const std::string& ifname)
    : roboteq_can_id_(roboteq_can_id) {
  
  socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  std::vector<uint32_t> ids;
  ids.push_back(SDO_COB_ID_OFFSET + CanopenInterface::roboteq_can_id_);
  ids.push_back(SDO_RESPONSE_COB_ID_OFFSET + CanopenInterface::roboteq_can_id_);

  uwrt_mars_rover_utils::UWRTCANWrapper wrapper_("roboteq", ifname, false);
  wrapper_.init(ids);
}

template <typename DataType>
bool CanopenInterface::sendCommand(RuntimeCommand command, uint8_t subindex, DataType data) {

  std::vector<uint8_t> package;
  package[0] = (SDO_COMMAND_ID << 4) | ((SDO_MAX_DATA_SIZE - sizeof(DataType)) << 2);
  package[1] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command);
  package[2] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command) >> bytesToBits(1);
  package[3] = subindex;
  package[4] = data;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[5] = data >> bytesToBits(1);
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[6] = data >> bytesToBits(2);
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[7] = data >> bytesToBits(3);

  bool check = wrapper_.writeToIDwithAck(package, roboteq_can_id_);

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  ROS_DEBUG_STREAM(
      std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
               << static_cast<unsigned>(response_frame.data[0]) << "\t" << static_cast<unsigned>(response_frame.data[1])
               << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
               << static_cast<unsigned>(response_frame.data[3]) << "\t" << static_cast<unsigned>(response_frame.data[4])
               << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
               << static_cast<unsigned>(response_frame.data[6]) << "\t"
               << static_cast<unsigned>(response_frame.data[7]));

  return check ? false : true;
}

template <>
bool CanopenInterface::sendCommand<empty_data_payload>(RuntimeCommand command, uint8_t subindex, empty_data_payload) {
  
  std::vector<uint8_t> package;
  package[0] = (SDO_COMMAND_ID << 4) | (3 << 2);
  package[1] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command);
  package[2] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command) >> bytesToBits(1);
  package[3] = subindex;
  package[4] = 0;
  package[5] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[6] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[7] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  bool check = wrapper_.writeToIDwithAck(package, roboteq_can_id_);

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  ROS_DEBUG_STREAM(
      std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
               << static_cast<unsigned>(response_frame.data[0]) << "\t" << static_cast<unsigned>(response_frame.data[1])
               << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
               << static_cast<unsigned>(response_frame.data[3]) << "\t" << static_cast<unsigned>(response_frame.data[4])
               << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
               << static_cast<unsigned>(response_frame.data[6]) << "\t"
               << static_cast<unsigned>(response_frame.data[7]));

  return check ? false : true;
}

template <typename DataType>
DataType CanopenInterface::sendQuery(RuntimeQuery query, uint8_t subindex) {

  std::vector<uint8_t> package;
  package[0] = (SDO_QUERY_ID << 4) | ((SDO_MAX_DATA_SIZE - sizeof(DataType)) << 2);
  package[1] = CanopenInterface::QUERY_CANOPEN_ID_MAP_.at(query);
  package[2] = CanopenInterface::QUERY_CANOPEN_ID_MAP_.at(query) >> bytesToBits(1);
  package[3] = subindex;
  package[4] = 0;
  package[5] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[6] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  package[7] = 0; // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  wrapper_.writeToIDwithAck(package, roboteq_can_id_);

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  ROS_DEBUG_STREAM(
      std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
               << static_cast<unsigned>(response_frame.data[0]) << "\t" << static_cast<unsigned>(response_frame.data[1])
               << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
               << static_cast<unsigned>(response_frame.data[3]) << "\t" << static_cast<unsigned>(response_frame.data[4])
               << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
               << static_cast<unsigned>(response_frame.data[6]) << "\t"
               << static_cast<unsigned>(response_frame.data[7]));


  const size_t data_response_size = SDO_MAX_DATA_SIZE - ((response_frame.data[0] & UNUSED_BYTES_MASK) >> 2);

  uint32_t raw_response_data{};
  static constexpr size_t START_OF_DATA_INDEX{4};
  for (size_t data_index = START_OF_DATA_INDEX; data_index < START_OF_DATA_INDEX + data_response_size; data_index++) {
    raw_response_data |= (response_frame.data[data_index] << bytesToBits(data_index - START_OF_DATA_INDEX));
  }

  // Pad unused bytes with 0xFF if negative and signed
  if (std::is_signed<DataType>() && std::signbit(response_frame.data[START_OF_DATA_INDEX + data_response_size - 1])) {
    constexpr uint8_t NEGATIVE_PADDING_BYTE = 0xFF;
    for (size_t byte_number = data_response_size; byte_number < sizeof(uint32_t); byte_number++) {
      raw_response_data |= NEGATIVE_PADDING_BYTE << bytesToBits(byte_number);
    }
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  DataType response_data = reinterpret_cast<DataType&>(raw_response_data);
  return response_data;
}

// Explicit Template Instantiation
template bool CanopenInterface::sendCommand<uint32_t>(RuntimeCommand command, uint8_t subindex, uint32_t data);
template bool CanopenInterface::sendCommand<int32_t>(RuntimeCommand command, uint8_t subindex, int32_t data);
template bool CanopenInterface::sendCommand<uint8_t>(RuntimeCommand command, uint8_t subindex, uint8_t data);
template bool CanopenInterface::sendCommand<empty_data_payload>(RuntimeCommand command, uint8_t subindex,
                                                                empty_data_payload);

template uint32_t CanopenInterface::sendQuery<uint32_t>(RuntimeQuery query, uint8_t subindex);
template int32_t CanopenInterface::sendQuery<int32_t>(RuntimeQuery query, uint8_t subindex);
template uint16_t CanopenInterface::sendQuery<uint16_t>(RuntimeQuery query, uint8_t subindex);
template int16_t CanopenInterface::sendQuery<int16_t>(RuntimeQuery query, uint8_t subindex);
template uint8_t CanopenInterface::sendQuery<uint8_t>(RuntimeQuery query, uint8_t subindex);
template int8_t CanopenInterface::sendQuery<int8_t>(RuntimeQuery query, uint8_t subindex);
template bool CanopenInterface::sendQuery<bool>(RuntimeQuery query, uint8_t subindex);

}  // namespace roboteq
