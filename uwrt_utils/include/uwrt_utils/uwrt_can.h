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

namespace uwrt_utils {

class UWRTCANWrapper {
 public:
  explicit UWRTCANWrapper() = default;
  explicit UWRTCANWrapper(std::string name, std::string interface_name, bool rcv_big_endian,
                          int thread_sleep_millis = 10);
  explicit UWRTCANWrapper(const UWRTCANWrapper& to_copy) = delete;
  // NOLINTNEXTLINE(performance-noexcept-move-constructor, bugprone-exception-escape)
  explicit UWRTCANWrapper(UWRTCANWrapper&& to_move);

  ~UWRTCANWrapper();

  UWRTCANWrapper& operator=(const UWRTCANWrapper& to_copy) = delete;
  // NOLINTNEXTLINE(performance-noexcept-move-constructor, bugprone-exception-escape)
  UWRTCANWrapper& operator=(UWRTCANWrapper&& to_move);

  void init(const std::vector<uint32_t>& ids);

 private:
  // description name for this wrapper
  std::string name_;

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
  std::map<canid_t, struct can_frame> recv_map_;
  std::timed_mutex recv_map_mtx_;
  std::chrono::milliseconds thread_sleep_millis_{};
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

  // read function to be run in the thread
  void readSocketTask();

  // function to get swap endianness
  template <class T>
  T correctEndianness(const T& data) {
    // if endianness is the same, just return the data back
    if (rcv_endianness_ == __BYTE_ORDER__) {
      return data;
    }

    // if not, we need to swap endianess
    T swapped_data;
    auto swapped_ptr =
        reinterpret_cast<uint8_t*>(&swapped_data);                // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    auto original_ptr = reinterpret_cast<const uint8_t*>(&data);  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
    for (int i = 0; i < sizeof(T); i++) {
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      std::memcpy(&swapped_ptr[i], &original_ptr[sizeof(T) - 1 - i], 1);
    }
    return swapped_data;
  }

 public:
  // these two function definitions need to be in header because they use templates (good one c++ 11)
  template <class T>
  bool getLatestFromID(T& data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      throw std::runtime_error("Please initialize CAN wrapper before using it");
    }

    // check that we have new data to read at specified id
    if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
      throw std::runtime_error("Timed out while trying to lock read mutex");
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
      throw std::runtime_error("Mismatch of size between recieved and requested data");
    }
    std::memcpy(&data, frame.data, sizeof(T));
    data = correctEndianness<T>(data);
    return true;
  }

  template <class T>
  bool writeToID(T data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      throw std::runtime_error("Please initialize CAN wrapper before using it");
    }

    // make sure data isn't too big
    if (sizeof(T) > CAN_MAX_DLEN) {
      throw std::runtime_error("Size of this data structure is too big");
    }

    // construct data frame
    struct can_frame frame {};
    frame.can_id = (canid_t)id;
    frame.can_dlc = sizeof(T);
    data = correctEndianness<T>(data);
    std::memcpy(frame.data, &data, sizeof(T));

    int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), 0);

    return bytes_sent == sizeof(struct can_frame);
  }
};

}  // namespace uwrt_utils
