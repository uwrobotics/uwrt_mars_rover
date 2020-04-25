#include "CanopenInterface.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>

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
  if ((socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    printf("Error while opening socket");
    throw - 1;
  }

  struct ifreq ifr {};

  std::strcpy(ifr.ifr_name, ifname.c_str());
  ioctl(socket_handle_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr {};

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  std::cout << ifname << " at index " << ifr.ifr_ifindex << std::endl;

  std::array<struct can_filter, 2> can_receive_filter{};

  can_receive_filter[0].can_id = SDO_COB_ID_OFFSET + roboteq::CanopenInterface::roboteq_can_id_;
  can_receive_filter[0].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);
  can_receive_filter[1].can_id = SDO_RESPONSE_COB_ID_OFFSET + roboteq::CanopenInterface::roboteq_can_id_;
  can_receive_filter[1].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_EFF_MASK);

  //   struct timeval receive_timeout = {.tv_usec=500000}; //0.5 seconds
  // setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout, sizeof(receive_timeout));

  int socket_opt_ret_val = setsockopt(socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER, can_receive_filter.data(),
                                      can_receive_filter.size() * sizeof(struct can_filter));
  if (socket_opt_ret_val != 0) {
    throw - 1;
  }

  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): reinterpret cast required by syscall
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    printf("Error in socket bind");
    throw - 1;
  }
}

template <typename DataType>
bool CanopenInterface::sendCommand(RuntimeCommand command, uint8_t subindex, DataType data) {
  struct can_frame command_frame {};

  command_frame.can_id = SDO_COB_ID_OFFSET + roboteq::CanopenInterface::roboteq_can_id_;
  command_frame.can_dlc = CAN_FRAME_SIZE_BYTES;
  command_frame.data[0] = (SDO_COMMAND_ID << 4) | ((SDO_MAX_DATA_SIZE - sizeof(DataType)) << 2);
  command_frame.data[1] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command);
  command_frame.data[2] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command) >> bytesToBits(1);
  command_frame.data[3] = subindex;
  command_frame.data[4] = data;
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  command_frame.data[5] = data >> bytesToBits(1);
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  command_frame.data[6] = data >> bytesToBits(2);
  // NOLINTNEXTLINE(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  command_frame.data[7] = data >> bytesToBits(3);

  ssize_t bytes_written = write(roboteq::CanopenInterface::socket_handle_, &command_frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;
  }

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  //  std::cout << std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
  //            << static_cast<unsigned>(response_frame.data[0]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[1])
  //            << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[3]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[4])
  //            << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[6]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[7])
  //            << std::endl;

  if (bytes_read != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;  // NOLINT(readability-simplify-boolean-expr): temp until error todo is finished
  }

  // TODO: throw appropriate errors
  if ((response_frame.data[0] & RESPONSE_TYPE_MASK) != SUCCESSFUL_COMMAND_RESPONSE) {
    std::cout << "Command unsuccessful response" << std::endl;
  } else if ((command_frame.data[0] & UNUSED_BYTES_MASK) != (response_frame.data[0] & UNUSED_BYTES_MASK)) {
    std::cout << "Command response mismatched unused bytes number" << std::endl;
  } else if (command_frame.data[1] != response_frame.data[1] || command_frame.data[2] != response_frame.data[2]) {
    std::cout << "Command response mismatched index" << std::endl;
  } else if (command_frame.data[3] != response_frame.data[3]) {
    std::cout << "Command response mismatched subindex" << std::endl;
  } else {
    return true;
  }
  std::cout << "COMMAND RESPONSE ID" << response_frame.can_id << std::endl;
  return false;
}

template <>
bool CanopenInterface::sendCommand<empty_data_payload>(RuntimeCommand command, uint8_t subindex, empty_data_payload) {
  struct can_frame command_frame {};

  std::cout << "no data SPECIAL" << std::endl;
  command_frame.can_id = SDO_COB_ID_OFFSET + roboteq::CanopenInterface::roboteq_can_id_;
  command_frame.can_dlc = CAN_FRAME_SIZE_BYTES;
  command_frame.data[0] = (SDO_COMMAND_ID << 4) | (3 << 2);
  command_frame.data[1] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command);
  command_frame.data[2] = CanopenInterface::COMMAND_CANOPEN_ID_MAP_.at(command) >> bytesToBits(1);
  command_frame.data[3] = subindex;
  command_frame.data[4] = 0;
  command_frame.data[5] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  command_frame.data[6] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  command_frame.data[7] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  ssize_t bytes_written = write(roboteq::CanopenInterface::socket_handle_, &command_frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;
  }

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  //  std::cout << std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
  //            << static_cast<unsigned>(response_frame.data[0]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[1])
  //            << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[3]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[4])
  //            << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[6]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[7])
  //            << std::endl;

  if (bytes_read != sizeof(struct can_frame)) {
    // TODO: throw error
    return false;  // NOLINT(readability-simplify-boolean-expr): temp until error todo is finished
  }

  // TODO: throw appropriate errors
  if ((response_frame.data[0] & RESPONSE_TYPE_MASK) != SUCCESSFUL_COMMAND_RESPONSE) {
    std::cout << "Command unsuccessful response" << std::endl;
  } else if ((command_frame.data[0] & UNUSED_BYTES_MASK) != (response_frame.data[0] & UNUSED_BYTES_MASK)) {
    std::cout << "Command response mismatched unused bytes number" << std::endl;
  } else if (command_frame.data[1] != response_frame.data[1] || command_frame.data[2] != response_frame.data[2]) {
    std::cout << "Command response mismatched index" << std::endl;
  } else if (command_frame.data[3] != response_frame.data[3]) {
    std::cout << "Command response mismatched subindex" << std::endl;
  } else {
    return true;
  }
  std::cout << "COMMAND RESPONSE ID" << response_frame.can_id << std::endl;
  return false;
}

template <typename DataType>
DataType CanopenInterface::sendQuery(RuntimeQuery query, uint8_t subindex) {
  struct can_frame query_frame {};

  query_frame.can_id = SDO_COB_ID_OFFSET + roboteq::CanopenInterface::roboteq_can_id_;
  query_frame.can_dlc = CAN_FRAME_SIZE_BYTES;
  query_frame.data[0] = (SDO_QUERY_ID << 4) | ((SDO_MAX_DATA_SIZE - sizeof(DataType)) << 2);
  query_frame.data[1] = CanopenInterface::QUERY_CANOPEN_ID_MAP_.at(query);
  query_frame.data[2] = CanopenInterface::QUERY_CANOPEN_ID_MAP_.at(query) >> bytesToBits(1);
  query_frame.data[3] = subindex;
  query_frame.data[4] = 0;
  query_frame.data[5] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  query_frame.data[6] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
  query_frame.data[7] = 0;  // NOLINT(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

  ssize_t bytes_written = write(roboteq::CanopenInterface::socket_handle_, &query_frame, sizeof(struct can_frame));
  if (bytes_written != sizeof(struct can_frame)) {
    // TODO: throw error
    return 0;
  }

  struct can_frame response_frame = {};
  ssize_t bytes_read = read(roboteq::CanopenInterface::socket_handle_, &response_frame, sizeof(struct can_frame));

  //  std::cout << std::hex << response_frame.can_id << "\t" << static_cast<unsigned>(response_frame.can_dlc) << "\t"
  //            << static_cast<unsigned>(response_frame.data[0]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[1])
  //            << "\t" << static_cast<unsigned>(response_frame.data[2]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[3]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[4])
  //            << "\t" << static_cast<unsigned>(response_frame.data[5]) << "\t"
  //            << static_cast<unsigned>(response_frame.data[6]) << "\t" <<
  //            static_cast<unsigned>(response_frame.data[7])
  //            << std::endl;

  if (bytes_read != sizeof(struct can_frame)) {
    // TODO: throw error
    return 0;
  }

  // TODO: throw appropriate errors
  if ((response_frame.data[0] & RESPONSE_TYPE_MASK) != SUCCESSFUL_QUERY_RESPONSE) {
    std::cout << "Query unsuccessful response" << std::endl;
  } else if (query_frame.data[1] != response_frame.data[1] || query_frame.data[2] != response_frame.data[2]) {
    std::cout << "Query response mismatched index" << std::endl;
  } else if (query_frame.data[3] != response_frame.data[3]) {
    std::cout << "Query response mismatched subindex" << std::endl;
  } else {
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

  std::cout << "QUERY RESPONSE ID" << response_frame.can_id << std::endl;
  return false;
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
