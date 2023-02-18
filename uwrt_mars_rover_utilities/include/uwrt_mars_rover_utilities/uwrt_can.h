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

#include "hw_bridge.h"

// #include "uwrt_params.h"
#include "rclcpp/logger.hpp"

namespace uwrt_mars_rover_utilities {

class UWRTCANWrapper {
  using CANMsgMap = HWBRIDGE::CANMsgMap;
  using CANID     = HWBRIDGE::CANID;
  using CANSIGNAL = HWBRIDGE::CANSIGNAL;
  using CanSignalValue_t = HWBRIDGE::CANSignalValue_t;

 public:
  explicit UWRTCANWrapper(): UWRTCANWrapper("uwrt_can", "can0", false, 1) {}

  explicit UWRTCANWrapper(
      std::string name, std::string interface_name, bool rcv_big_endian,
      int thread_sleep_millis = 10);  // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

  explicit UWRTCANWrapper(const UWRTCANWrapper &to_copy) = delete;
  // NOLINTNEXTLINE(performance-noexcept-move-constructor, bugprone-exception-escape)
  explicit UWRTCANWrapper(UWRTCANWrapper &&to_move);

  ~UWRTCANWrapper();

  UWRTCANWrapper &operator=(const UWRTCANWrapper &to_copy) = delete;
  // NOLINTNEXTLINE(performance-noexcept-move-constructor,bugprone-exception-escape)
  UWRTCANWrapper &operator=(UWRTCANWrapper &&to_move);

  void init(const std::vector<uint32_t> &ids);

 private:
  // description name for this wrapper
  std::string name_;

  // ROS2 logger variable for ROS2 errors
  rclcpp::Logger logger_;

  // variables needed for can socket
  std::string interface_name_;
  int socket_handle_{};
  struct sockaddr_can sock_addr_ {};
  struct ifreq ifr_ {};

  // variables for wrapper
  bool initialized_{false};
  int rcv_endianness_{__BYTE_ORDER__};
  std::thread read_thread_;
  volatile bool read_thread_running_{};
  
  HWBRIDGE::CANMsgMap msg_map_ { // example of how it would be created,
  // this map would be much larger so it might be better to create it in a separate file
  // and include it here
    // Msg 1
    {CANID::COMMON_DEBUG_MESSAGE1,
     {
         {CANSIGNAL::COMMON_DEBUG_SIGNAL1, 0},
     }},

    // Msg 2
    {CANID::COMMON_DEBUG_MESSAGE3,
     {
         {CANSIGNAL::COMMON_DEBUG_SIGNAL3, 0},
     }},
  };

  // Can be changed if necessary, but right now in order to
  // not break existing functionality I am keeping the old map
  // as well
  // This map is used to store the received can frames
  // When a value is read from this map it is deleted from the map
  // The actual CANMsgMap will still be storing the values
  // Not sure which behaviour we want to keep
  std::map<canid_t, struct can_frame> recv_map_;
  
  std::timed_mutex recv_map_mtx_;
  std::chrono::milliseconds thread_sleep_millis_{};
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

  // variables needed for can send & acknowledge function
  static constexpr int MAX_TRIES = 5;
  static constexpr int RATE = 10;

  // read function to be run in the thread
  void readSocketTask();


  // function to get swap endianness
  template <class T>
  T correctEndianness(const T &data) {
    // if endianness is the same, just return the data back
    if (rcv_endianness_ == __BYTE_ORDER__) {
      return data;
    }

    RCLCPP_INFO(logger_, "pre end data %ld",  data);
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
    RCLCPP_INFO(logger_, "post end data %ld",  swapped_data);
    return swapped_data;
  }

 public:
  
  // New CAN methods to read/write to CAN bus using enums
  /**
   * @brief Writes a signal to the CAN bus.
   * @param id The CAN ID to write to
   * @param signal The signal to write to (part of the 8 byte message)
   * @param value The value to write to the signal (CanSignalValue is an alias for double
   * at the moment)
   * 
   * @return True if the write was successful, false otherwise
   */
  bool writeToID(CANID id, CANSIGNAL signal, CanSignalValue_t value);

  /**
   * @brief Reads a signal from the CAN bus.
   * @param id The CAN ID to read from
   * @param signal The signal to read from (part of the 8 byte message)
   * @param value The value to read from the signal (CanSignalValue is an alias for double
   * at the moment)
   * 
   * @return True if the read was successful, false otherwise
   */
  bool getLatestFromID(CANID id, CANSIGNAL signal, CanSignalValue_t &value);

  // these two function definitions need to be in header because they use
  // templates (good one c++ 11)
  template <class T>
  bool getLatestFromID(T &data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      RCLCPP_ERROR_STREAM(logger_, "Please initialize CAN wrapper before using it");
    }

    // check that we have new data to read at specified id
    if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
      RCLCPP_ERROR_STREAM(logger_, "Timed out while trying to lock read mutex");
    }
    auto it = recv_map_.find((canid_t)id);
    if (it == recv_map_.end()) {
      recv_map_mtx_.unlock();
      return false;
    }

    // read and delete new data at specified id
    struct can_frame frame = recv_map_[id];
    recv_map_.erase((canid_t)id);
    recv_map_mtx_.unlock();

    // extract data from frame
    if (frame.can_dlc != sizeof(T)) {
      RCLCPP_ERROR_STREAM(logger_, "Mismatch of size between recieved and requested data");
    }
    std::memcpy(&data, frame.data, sizeof(T));
    data = correctEndianness<T>(data);
    return true;
  }

  template <class T>
  bool writeToID(T data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      RCLCPP_ERROR_STREAM(logger_, "Please initialize CAN wrapper before using it");
    }

    // make sure data isn't too big
    if (sizeof(T) > CAN_MAX_DLEN) {
      RCLCPP_ERROR_STREAM(logger_, "Size of this data structure is too big");
    }

    // construct data frame
    struct can_frame frame {};
    frame.can_id = (canid_t)id;
    frame.can_dlc = sizeof(T);
    // data = correctEndianness<T>(data);
    std::memcpy(frame.data, &data, sizeof(T));

    // unsigned char* data_ptr = frame.data;
    std::string str_data = (char*)frame.data;
    RCLCPP_INFO(logger_, "%x |  %s",  frame.can_id, frame.data);

    // auto t1 = std::chrono::high_resolution_clock::now();
    int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), 0);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);

    // RCLCPP_INFO(logger_, "CAN send time %d", ms_int);



    return bytes_sent == sizeof(struct can_frame);
  }

  template <class T>
  bool writeToIDwithAck(T data, uint32_t id) {
    // check socket has been initialized
    if (!initialized_) {
      RCLCPP_ERROR_STREAM(logger_, "Please intialize CAN wrapper before using it");
    }

    // makes sure data is not too big
    if (sizeof(T) > CAN_MAX_DLEN) {
      RCLCPP_ERROR_STREAM(logger_, "Size of this data structure is too big");
    }

    // construct data frame
    struct can_frame frame {};
    frame.can_id = (canid_t)id;
    frame.can_dlc = sizeof(T);
    data = correctEndianness<T>(data);
    std::memcpy(frame.data, &data, sizeof(T));

    // send data over can bus
    int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);

    // error checking
    if (bytes_sent != sizeof(struct can_frame)) {
      RCLCPP_ERROR_STREAM(logger_, "CAN MESSAGE FAILED TO SEND");
      return false;
    }

    // wait for one second then receive can frame
    std::this_thread::sleep_for(std::chrono::seconds(1));

    int attempts{0};                 // number of tries executed for reading from socket
    rclcpp::WallRate loop_rate{RATE};       // loop rate
    int bytes_recv{};                // holds the number of bytes received from can bus
    struct can_frame recv_frame {};  // empty can frame to be filled

    do {
      // recv can frame from can bus - non-blocking
      bytes_recv = recv(socket_handle_, &recv_frame, sizeof(struct can_frame), MSG_DONTWAIT);
      attempts++;
      loop_rate.sleep();
    } while (rclcpp::ok() && attempts < MAX_TRIES && bytes_recv != sizeof(struct can_frame));

    // check if message was successfully received by firmware
    if (bytes_recv == sizeof(struct can_frame)) {
      RCLCPP_INFO_STREAM(logger_, "MESSAGE SENT TO FW SUCCESSFULLY");
      return true;
    }

    RCLCPP_ERROR_STREAM(logger_, "FW MESSAGE FAILED TO LOAD");
    return false;
  }
};

}  // namespace uwrt_mars_rover_utils
