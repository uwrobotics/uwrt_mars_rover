// AUTO-GENERATED FILE. DO NOT MODIFY. GENERATED BY scripts/generate_can_wrapper.py

#pragma once

#include "CANMsgMap.h"
#include "uwrt_mars_rover_can.h"
#include "uwrt_mars_rover_can_enums.h"

namespace HWBRIDGE {

bool packCANMsg(uint8_t* raw, HWBRIDGE::CANID msgID, const HWBRIDGE::CANMsgMap* msgMap, size_t& len);
bool unpackCANMsg(uint8_t* raw, HWBRIDGE::CANID msgID, HWBRIDGE::CANMsgMap* msgMap);

}  // namespace HWBRIDGE