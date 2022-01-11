#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

#include <chrono>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uwrt_mars_rover_utils/visibility_control.h"

namespace uwrt_mars_rover_utils {

class UWRT_CAN {
 public:
  explicit UWRT_CAN() = default;
  explicit UWRT_CAN(std::string name, int default_endianess, std::string can_interface_name);
  virtual ~UWRT_CAN();

  bool configure();
  bool activate();
  bool deactivate();
  bool cleanup();


 private:
  std::string logger_;

  std::string can_interface_name_;
  int socket_handle_{};
  struct sockaddr_can sock_addr_ {};
  struct ifreq ifr_ {};

  int default_endianness_;

  bool initialized_{false};
  std::thread read_thread_;
  volatile bool read_thread_running_{};
  std::map<canid_t, struct can_frame> recv_map_;
  std::timed_mutex recv_map_mtx_;
  std::chrono::milliseconds thread_sleep_millis_{};
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

  // variables needed for can send & acknowledge function
  static constexpr int MAX_TRIES = 5;
  static constexpr int RATE = 10;

  // read function to be run in the thread
  void readSocketTask();

  template <class T>
  T correctEndianness(const T &data) {
    // if endianness is the same, just return the data back
    if (rcv_endianness_ == __BYTE_ORDER__) {
      return data;
    }

    // if not, we need to swap endianess
    T swapped_data;
    auto swapped_ptr =
        reinterpret_cast<uint8_t *>(&swapped_data);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    auto original_ptr =
        reinterpret_cast<const uint8_t *>(&data);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    for (int i = 0; i < sizeof(T); i++) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      std::memcpy(&swapped_ptr[i], &original_ptr[sizeof(T) - 1 - i], 1);
    }
    return swapped_data;
  }

 public:
//  template <class T>
//  bool getLatestFromID(T &data, uint32_t id) {
//    // make sure we have been initialized
//    if (!initialized_) {
//      RCLCPP_ERROR_STREAM_NAMED(logger_, "Please initialize CAN wrapper before using it");
//    }
//
//    // check that we have new data to read at specified id
//    if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
//      RCLCPP_ERROR_STREAM_NAMED(logger_, "Timed out while trying to lock read mutex");
//    }
//    auto it = recv_map_.find((canid_t)id);
//    if (it == recv_map_.end()) {
//      recv_map_mtx_.unlock();
//      return false;
//    }
//
//    // read and delete new data at specified id
//    struct can_frame frame = recv_map_[id];
//    recv_map_.erase((canid_t)id);
//    recv_map_mtx_.unlock();
//
//    // extract data from frame
//    if (frame.can_dlc != sizeof(T)) {
//      RCLCPP_ERROR_STREAM_NAMED(logger_, "Mismatch of size between recieved and requested data");
//    }
//    std::memcpy(&data, frame.data, sizeof(T));
//    data = correctEndianness<T>(data);
//    return true;
//  }
//
//  template <class T>
//  bool writeToID(T data, uint32_t id) {
//    // make sure we have been initialized
//    if (!initialized_) {
//      RCLCPP_ERROR_STREAM_NAMED(logger_, "Please initialize CAN wrapper before using it");
//    }
//
//    // make sure data isn't too big
//    if (sizeof(T) > CAN_MAX_DLEN) {
//      RCLCPP_ERROR_STREAM_NAMED(name_, "Size of this data structure is too big");
//    }
//
//    // construct data frame
//    struct can_frame frame {};
//    frame.can_id = (canid_t)id;
//    frame.can_dlc = sizeof(T);
//    data = correctEndianness<T>(data);
//    std::memcpy(frame.data, &data, sizeof(T));
//
//    int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), 0);
//
//    return bytes_sent == sizeof(struct can_frame);
//  }


};

}  // namespace uwrt_mars_rover_utils
