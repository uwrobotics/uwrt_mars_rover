#pragma once

#include <linux/can.h>

#include <unordered_map>

#include "CommunicationInterface.hpp"

namespace roboteq {
using empty_data_payload = std::nullptr_t;

class CanopenInterface : public CommunicationInterface {
 public:
  explicit CanopenInterface(canid_t roboteq_can_id = 0x1, const std::string& ifname = "can0");
  CanopenInterface(CanopenInterface&&) = default;
  CanopenInterface& operator=(CanopenInterface&&) = default;
  CanopenInterface(const CanopenInterface&) = default;
  CanopenInterface& operator=(const CanopenInterface&) = default;
  ~CanopenInterface() override = default;

  template <typename DataType>
  bool sendCommand(RuntimeCommand command, uint8_t subindex = 0, DataType data = nullptr);

  template <typename DataType>
  DataType sendQuery(RuntimeQuery query, uint8_t subindex = 0);

 private:
  static inline constexpr unsigned bytesToBits(const unsigned num_bytes) {
    constexpr unsigned BITS_PER_BYTE = 8;
    return num_bytes * BITS_PER_BYTE;
  }

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
