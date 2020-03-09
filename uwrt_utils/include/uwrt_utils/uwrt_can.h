#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace uwrt_utils {

class UWRTCANWrapper {
 public:
  explicit UWRTCANWrapper() = default;
  explicit UWRTCANWrapper(std::string name, std::string interface_name, int thread_sleep_millis = 1)
      : name_(std::move(name)),
        interface_name_(std::move(interface_name)),
        thread_sleep_millis_(std::chrono::milliseconds(thread_sleep_millis)){};
  // TODO (wraftus) make other constructor for failover CAN

  explicit UWRTCANWrapper(const UWRTCANWrapper& to_copy) = delete;
  explicit UWRTCANWrapper(UWRTCANWrapper&& to_move) noexcept
      : name_(std::move(to_move.name_)),
        interface_name_(std::move(to_move.interface_name_)),
        thread_sleep_millis_(to_move.thread_sleep_millis_) {
    to_move.read_thread_running_ = false;
    if (to_move.read_thread_.joinable()) {
      to_move.read_thread_.join();
    }
  }

  UWRTCANWrapper& operator=(const UWRTCANWrapper& to_copy) = delete;
  UWRTCANWrapper& operator=(UWRTCANWrapper&& to_move) noexcept {
    name_ = std::move(to_move.name_);
    interface_name_ = (std::move(to_move.interface_name_));
    thread_sleep_millis_ = to_move.thread_sleep_millis_;
    initialized_ = false;

    to_move.read_thread_running_ = false;
    if (to_move.read_thread_.joinable()) {
      to_move.read_thread_.join();
    }
    read_thread_running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    return *this;
  }

  ~UWRTCANWrapper() {
    read_thread_running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

  enum class UWRTCANStatus {
    STATUS_OK = 0,
    NOT_INITED,
    SOCKET_CREATE_FAILED,
    SOCKET_BIND_FAILED,
    RECV_MUTEX_TIMEOUT,
    RECV_SIZE_MISMATCH,
    RECV_UNREGOCNIZED_ID,
    SEND_DATA_OVERSIZED,
    SEND_DATA_FAILED
  };

  UWRTCANStatus init(const std::vector<uint32_t>& ids);

  template <class T>
  UWRTCANStatus getLatestFromID(T& data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      return UWRTCANWrapper::UWRTCANStatus::NOT_INITED;
    }

    // check that we have new data to read at specified id
    std::map<canid_t, struct can_frame>::iterator it;
    if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT_)) {
      return UWRTCANWrapper::UWRTCANStatus::RECV_MUTEX_TIMEOUT;
    }
    it = recv_map_.find((canid_t)id);
    if (it == recv_map_.end()) {
      recv_map_mtx_.unlock();
      return UWRTCANWrapper::UWRTCANStatus::RECV_UNREGOCNIZED_ID;
    }

    // read and delete new data at specified id
    struct can_frame frame = recv_map_[id];
    recv_map_.erase((canid_t)id);
    recv_map_mtx_.unlock();

    // extract data from frame
    if (frame.can_dlc != sizeof(data)) {
      return UWRTCANWrapper::UWRTCANStatus::RECV_SIZE_MISMATCH;
    }
    memcpy(&data, frame.data, sizeof(T));
    return UWRTCANWrapper::UWRTCANStatus::STATUS_OK;
  }

  template <class T>
    UWRTCANStatus writeToID(const T& data, uint32_t id) {
    // make sure we have been initialized
    if (!initialized_) {
      return UWRTCANWrapper::UWRTCANStatus::NOT_INITED;
    }

    // make sure data isn't too big
    if (sizeof(data) > CAN_MAX_DLEN) {
      return UWRTCANWrapper::UWRTCANStatus::SEND_DATA_OVERSIZED;
    }

    // construct data frame
    struct can_frame frame {};
    frame.can_id = (canid_t) id;
    frame.can_dlc = sizeof(data);
    memcpy(frame.data, &data, sizeof(data));

    int bytes_sent = send(socket_handle_, &frame, sizeof(struct can_frame), 0);

    if (bytes_sent == sizeof(struct can_frame)) {
      return UWRTCANWrapper::UWRTCANStatus::STATUS_OK;
    }
    return UWRTCANWrapper::UWRTCANStatus::SEND_DATA_FAILED;
  }
  // TODO (wraftus) add write_and_wait function to write and wait for ack

 private:
  // description name for this wrapper
  std::string name_;
  bool initialized_ = false;

  // variables needed for can socket
  std::string interface_name_;
  int socket_handle_{};
  struct sockaddr_can sock_addr_ {};
  struct ifreq ifr_ {};

  // variables for wrapper
  std::thread read_thread_;
  volatile bool read_thread_running_{};
  std::map<canid_t, struct can_frame> recv_map_;
  std::chrono::milliseconds thread_sleep_millis_{};
  std::timed_mutex recv_map_mtx_;
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT_{1};

  // read function to be run in the thread
  void readSocketTask();
};

}  // namespace uwrt_utils
