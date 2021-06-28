#include <uwrt_mars_rover_utils/uwrt_can.hpp>
#include <vector>

namespace uwrt_mars_rover_utils {

CANInterface::CANInterface(const Config &config) : rxMsgMap_(config.rxMsgMap), txMsgMap_(config.txMsgMap), rxOneShotMsgHandler_(config.rxOneShotMsgHandler) {
    // Init socket CAN
    socketHandle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketHandle_ < 0) {
        throw std::runtime_error("Failed to set up a socket for CAN in " __FILE__);
    }

    // Set up filters
    if (rxMsgMap_ != nullptr) {
        std::vector<struct can_filter> filters;
        for (auto it = rxMsgMap_->begin(); it != rxMsgMap_->end(); it++) {
            struct can_filter canFilter = {
                .can_id = (canid_t)(it->first),
                .can_mask = (canid_t)HWBRIDGE::ROVER_CANID_FILTER_MASK
            };
            filters.push_back(canFilter);
        }
        setsockopt(socketHandle_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(struct can_filter)*filters.size());
    }

    // Set read timeout
    struct timeval timeout;
    timeout.tv_usec = 1;
    setsockopt(socketHandle_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Bind socket to CAN addr
    if (bind(socketHandle_, reinterpret_cast<struct sockaddr*>(&sockaddr_), sizeof(sockaddr_)) < 0) {
        throw std::runtime_error("Failed to bind can device to socket in " __FILE__);
    }

    // Start RX and TX threads
    rxPostmanThread_ = std::thread(&CANInterface::rxPostman, this);
    rxClientThread_ = std::thread(&CANInterface::rxClient, this);
    txProcessorThread_ = std::thread(&CANInterface::txProcessor, this);
}

CANInterface::rxPostman(void) {
    struct can_frame frame;
    int bytesRead;

    while (true) {
        do {
            bytesRead = recv(socketHandle_, &frame, sizeof(struct can_frame), 0);
            if (bytesRead == sizeof(struct can_frame)) {
                // TODO: put msg in queue
                // if (!recv_map_mtx_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
                //     ROS_WARN_NAMED(name_, "CAN wrapper failed to lock mutex, resulting in a lost CAN frame!");
                //     break;
                // }
                // recv_map_[frame.can_id] = frame;
                // recv_map_mtx_.unlock();
            } 
            else if (bytesRead < 0 && errno != EWOULDBLOCK) {
                ROS_WARN_STREAM_NAMED(name_, "CAN interface failed to recieve CAN message because: " << strerror(errno));
            }
        } while (bytesRead == sizeof(struct can_frame));

        std::this_thread::sleep_for(thread_sleep_millis_);
    }
}

}
