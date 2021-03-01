#include <linux/can.h>
#include <string>
#include "ros/ros.h"

class DrivetrainFakeSignalTest {
 private:
  can_frame outgoing_can_frame_{};
  can_frame incoming_can_frame_{};
  int socket_handle_;

 public:
  DrivetrainFakeSignalTest(uint16_t can_id_outgoing, const std::string &name);
  void receiveCAN(uint8_t data);
  void sendCAN(uint8_t data);
};