#include "uwrt_utils/uwrt_can.h"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <ros/ros.h>

namespace uwrt_can {

bool UWRTCANWrapper::init(const std::vector<canid_t>& ids){  
  // set up can socket
  // TODO (wraftus) maybe look into these flags a bit more, and customtize it further
  socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_handle_ < 0) {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to initiate CAN socket for CAN wrapper " << name_);
    return false;
  }
  strcpy(ifr_.ifr_name, interface_name_.c_str());
  ioctl(socket_handle_,SIOCGIFINDEX, &ifr_);
  sock_addr_.can_family = AF_CAN;
  sock_addr_.can_ifindex = ifr_.ifr_ifindex;

  // set software filters on can_id
  std::vector<struct can_filter> filters;
  filters.resize(ids.size());
  for (int i = 0; i < ids.size(); i++) {
    filters[i].can_id = ids[i];
    filters[i].can_mask = (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_SFF_MASK);
  }
  setsockopt(socket_handle_, SOL_CAN_RAW, CAN_RAW_FILTER,
             filters.data(), sizeof(struct can_filter) * filters.size());

  // set read timeout to be very small, so it's not blocking
  struct timeval timeout;
  timeout.tv_usec = 500;
  setsockopt(socket_handle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  // finally bind can socket to can addr
  if (bind(socket_handle_, (struct sockaddr *)&sock_addr_, sizeof(sock_addr_)) < 0) {
    ROS_ERROR_STREAM_NAMED(name_, "Failed to bind CAN socket to CAN address for CAN wrapper " << name_);
    return false;
  }

  // start read thread
  read_thread_running_ = true;
  read_thread_ = std::thread(&UWRTCANWrapper::readSocketTask, this);

  return true;
}

// TODO (wraftus) implement heartbeat logic for failover CAN
void UWRTCANWrapper::readSocketTask() {
  struct can_frame frame;
  int bytes_read;

  while(read_thread_running_) {
    // keep performing read until buffer is emptied
    do {
      bytes_read = recv(socket_handle_, &frame, sizeof(struct can_frame), 0);
      if(bytes_read == sizeof(struct can_frame)) {
        if(!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) return; //TODO (wraftus) handle this case better
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

// TODO (wraftus) abstract struct can_frame away from user
bool UWRTCANWrapper::getLatestFromID(struct can_frame *frame, canid_t id) {
  // check that we have new data to read at specified id
  std::map<canid_t, struct can_frame>::iterator it;
  if(!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) return false; //TODO (wraftus) handle this case better
  it = recv_map_.find(id);
  if (it == recv_map_.end()) {
    recv_map_mtx_.unlock();
    return false;
  }

  // read and delete new data at specified id
  *frame = recv_map_[id];
  recv_map_.erase(it);
  recv_map_mtx_.unlock();
  return true;
}

// TODO (wraftus) abstract struct can_frame away from user
bool UWRTCANWrapper::writeToID(const struct can_frame *frame) {
  int bytes = send(socket_handle_, frame, sizeof(struct can_frame), 0);
  
  if (bytes == sizeof(struct can_frame))
    return true;
  
  return false;
}

} // namespace uwrt_can