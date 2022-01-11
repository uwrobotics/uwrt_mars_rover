#include "uwrt_mars_rover_utils/uwrt_can.hpp"

namespace uwrt_mars_rover_utils {

UWRT_CAN::UWRT_CAN(std::string name, int default_endianess, std::string can_interface_name)
    : logger_(rclcpp::get_logger_name(name)), can_interface_name_(std::move(can_interface_name)) {
  switch (default_endianess) {
    case __ORDER_LITTLE_ENDIAN__:
    case __ORDER_BIG_ENDIAN__:
      break;
    case __ORDER_PDP_ENDIAN__:
      throw std::invalid_argument("__ORDER_PDP_ENDIAN__ is not supported.");
      break;
    default:
      throw std::invalid_argument("Endianess(" + std::to_string(default_endianess) + ") is not supported.");
      break;
  }
  default_endianness_ = default_endianess;
};

UWRT_CAN::~UWRT_CAN() {
  read_thread_running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool UWRT_CAN::configure() {
  bool success = false;

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
    throw std::runtime_error("Failed to set up a socket for CAN in " __FILE__);
  }
  strcpy(ifr_.ifr_name, can_interface_name_.c_str());
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

  // set receive timeout to be very small, so it's not blocking
  struct timeval timeout {};
  timeout.tv_usec = 1;
  setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  // bind can socket to can addr
  if (bind(socket_handle_, reinterpret_cast<struct sockaddr*>(&sock_addr_), sizeof(sock_addr_)) < 0) {
    throw std::runtime_error("Failed to bind can device to socket in " __FILE__);
  }

  // start read thread
  read_thread_running_ = true;
  read_thread_ = std::thread(&UWRT_CAN::readSocketTask, this);
  initialized_ = true;

  return success;
}

bool UWRT_CAN::activate() {
  bool success = false;

  return success;
}

bool UWRT_CAN::deactivate() {
  bool success = false;

  return success;
}

bool UWRT_CAN::cleanup() {
  bool success = false;
  success = (close(socket_handle_) == 0);
  // TODO: cleanup maps

  return true;
}

void UWRT_CAN::readSocketTask() {
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
        ROS_WARN_STREAM_NAMED(name_, "CAN wrapper failed to receive CAN message because: " << strerror(errno));
      }
    } while (bytes_read == sizeof(struct can_frame));

    // Allow other threads to run
    std::this_thread::yield();
  }
}

}  // namespace uwrt_mars_rover_utils
