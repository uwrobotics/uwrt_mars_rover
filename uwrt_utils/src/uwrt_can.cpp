#include "uwrt_utils/uwrt_can.h"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>

namespace uwrt_utils {

constexpr std::chrono::milliseconds UWRTCANWrapper::MUTEX_LOCK_TIMEOUT_;

UWRTCANWrapper::UWRTCANStatus UWRTCANWrapper::init(const std::vector<uint32_t>& ids) {
  // set up can socket
  socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_handle_ < 0) {
    return UWRTCANWrapper::UWRTCANStatus::SOCKET_CREATE_FAILED;
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
  timeout.tv_usec = 500;
  setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  // finally bind can socket to can addr
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&sock_addr_), sizeof(sock_addr_)) < 0) {
    return UWRTCANWrapper::UWRTCANStatus::SOCKET_BIND_FAILED;
  }

  // start read thread
  read_thread_running_ = true;
  read_thread_ = std::thread(&UWRTCANWrapper::readSocketTask, this);

  return UWRTCANWrapper::UWRTCANStatus::STATUS_OK;
}

// TODO (wraftus) implement heartbeat logic for failover CAN
void UWRTCANWrapper::readSocketTask() {
  struct can_frame frame {};
  int bytes_read;

  while (read_thread_running_) {
    // keep performing read until buffer is emptied
    do {
      bytes_read = recv(socket_handle_, &frame, sizeof(struct can_frame), 0);
      if (bytes_read == sizeof(struct can_frame)) {
        if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT_)) {
          break;
        }
        recv_map_[frame.can_id] = frame;
        recv_map_mtx_.unlock();
      } else {
        // ruh roh
      }
    } while (bytes_read == sizeof(struct can_frame));

    // buffer is emptied, lets wait
    std::this_thread::sleep_for(thread_sleep_millis_);
  }
}

}  // namespace uwrt_utils