#include "uwrt_utils/uwrt_can.h"

#include <ros/ros.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cerrno>
#include <cstring>

namespace uwrt_utils {

// static constexpr need to be declared again in cpp file (fixed in c++17)
constexpr std::chrono::milliseconds UWRTCANWrapper::MUTEX_LOCK_TIMEOUT;

UWRTCANWrapper::UWRTCANWrapper(std::string name, std::string interface_name, bool rcv_big_endian,
                               int thread_sleep_millis)
    : name_(std::move(name)),
      interface_name_(std::move(interface_name)),
      rcv_endianness_(rcv_big_endian ? __ORDER_BIG_ENDIAN__ : __ORDER_LITTLE_ENDIAN__),
      thread_sleep_millis_(std::chrono::milliseconds(thread_sleep_millis)){};

// NOLINTNEXTLINE(performance-noexcept-move-constructor, bugprone-exception-escape)
UWRTCANWrapper::UWRTCANWrapper(UWRTCANWrapper&& to_move)
    : name_(std::move(to_move.name_)),
      interface_name_(std::move(to_move.interface_name_)),
      initialized_(to_move.initialized_),
      rcv_endianness_(to_move.rcv_endianness_),
      thread_sleep_millis_(std::chrono::milliseconds(to_move.thread_sleep_millis_)) {
  // if to_move has been initialized, move over other variables
  if (initialized_) {
    socket_handle_ = to_move.socket_handle_;
    sock_addr_ = to_move.sock_addr_;
    ifr_ = to_move.ifr_;

    read_thread_ = std::thread(std::move(to_move.read_thread_));
    read_thread_running_ = to_move.read_thread_running_;
    if (!std::unique_lock<std::timed_mutex>(to_move.recv_map_mtx_, MUTEX_LOCK_TIMEOUT)) {
      throw std::runtime_error("Timed out while trying to lock vector mutex");
    }
    recv_map_ = std::move(to_move.recv_map_);
  }
};

// NOLINTNEXTLINE(performance-noexcept-move-constructor, bugprone-exception-escape)
UWRTCANWrapper& UWRTCANWrapper::operator=(UWRTCANWrapper&& to_move) {
  if (this != &to_move) {
    // move constructor variables
    name_ = std::move(to_move.name_);
    interface_name_ = std::move(to_move.interface_name_);
    rcv_endianness_ = to_move.rcv_endianness_;
    thread_sleep_millis_ = to_move.thread_sleep_millis_;

    // stop our thread
    read_thread_running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }

    // if to_move has been initialized, move over other variables
    initialized_ = to_move.initialized_;
    if (initialized_) {
      socket_handle_ = to_move.socket_handle_;
      sock_addr_ = to_move.sock_addr_;
      ifr_ = to_move.ifr_;

      read_thread_ = std::thread(std::move(to_move.read_thread_));
      read_thread_running_ = to_move.read_thread_running_;
      if (!std::unique_lock<std::timed_mutex>(to_move.recv_map_mtx_, MUTEX_LOCK_TIMEOUT)) {
        throw std::runtime_error("Timed out while trying to lock vector mutex");
      }
      recv_map_ = std::move(to_move.recv_map_);
    }
  }

  return *this;
}

UWRTCANWrapper::~UWRTCANWrapper() {
  read_thread_running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void UWRTCANWrapper::init(const std::vector<uint32_t>& ids) {
  // if already initialized, stop the thread and clear rcv map
  if (initialized_) {
    initialized_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    recv_map_.clear();
  }

  // set up can socket
  socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_handle_ < 0) {
    throw std::runtime_error("Failed to set up a socket for CAN");
  }
  strcpy(ifr_.ifr_name, interface_name_.c_str());
  ioctl(socket_handle_, SIOCGIFINDEX, &ifr_);
  sock_addr_.can_family = AF_CAN;
  sock_addr_.can_ifindex = ifr_.ifr_ifindex;

  // set software filters on can_id
  std::vector<struct can_filter> filters;
  filters.resize(ids.size());
  for (int i = 0; i < ids.size(); i++) {
    filters[i].can_id = (canid_t)ids[i];
    filters[i].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  }
  setsockopt(socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter) * filters.size());

  // set read timeout to be very small, so it's not blocking
  struct timeval timeout {};
  timeout.tv_usec = 1;
  setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  // finally bind can socket to can addr
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&sock_addr_), sizeof(sock_addr_)) < 0) {
    throw std::runtime_error("Failed to bind can device to socket");
  }

  // start read thread
  read_thread_running_ = true;
  read_thread_ = std::thread(&UWRTCANWrapper::readSocketTask, this);
  initialized_ = true;
}

void UWRTCANWrapper::readSocketTask() {
  struct can_frame frame {};
  int bytes_read;

  while (read_thread_running_) {
    // keep performing read until buffer is emptied
    do {
      bytes_read = recv(socket_handle_, &frame, sizeof(struct can_frame), 0);
      if (bytes_read == sizeof(struct can_frame)) {
        if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
          ROS_WARN_NAMED(name_, "CAN wrapper failed to lock mutex, resulting in a lost CAN frame!");
          break;
        }
        recv_map_[frame.can_id] = frame;
        recv_map_mtx_.unlock();
      } else if (bytes_read < 0 && errno != EWOULDBLOCK) {
        ROS_WARN_STREAM_NAMED(name_, "CAN wrapper failed to recieve CAN message because: " << strerror(errno));
      }
    } while (bytes_read == sizeof(struct can_frame));

    // buffer is emptied, lets wait
    std::this_thread::sleep_for(thread_sleep_millis_);
  }
}

}  // namespace uwrt_utils