#pragma once
// Some includes needed for the CAN lib
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
// CAN lib includes done
#include <string>

#include "ros/ros.h"
// This is a library the Neopixel Node uses to send messages over CAN.
// If a general CAN library is added to the repo, delete this and use the general CAN library.
class NeopixelCan {
 private:
  int socket_handle_;
  can_frame outgoing_can_frame_{};
  std::string log_filter_;
  can_frame incoming_can_frame_{};

 public:
  // Constructor for my CAN class
  NeopixelCan(uint16_t can_id_outgoing, uint8_t dlc, const std::string &name, std::string log_filter);
  // sendCAN function that only sends 1 integer for neopixels
  void sendCAN(uint8_t data);
  // checks if the acknowledgement message was sent by the gimbtonomy board
  bool waitforAck();
};
