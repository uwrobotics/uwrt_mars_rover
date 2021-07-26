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

void CANInterface::rxPostman(void) {
    struct can_frame frame;
    int bytesRead;

    while (true) {
        do {
            bytesRead = recv(socketHandle_, &frame, sizeof(struct can_frame), 0);
            if (bytesRead == sizeof(struct can_frame)) {
                // Put message in queue
                if (!rxMailboxMutex_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN wrapper failed to lock mutex, resulting in a lost CAN frame!");
                    break;
                }
                rxMailbox_.push(frame);
                rxMailboxMutex_.unlock();
            } 
            else if (bytesRead < 0 && errno != EWOULDBLOCK) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to recieve CAN message");
            }
        } while (bytesRead == sizeof(struct can_frame));

        std::this_thread::sleep_for(THREAD_SLEEP_MILLISEC);
    }
}

void CANInterface::rxClient(void) {
    struct can_frame frame;

    while (true) {
        // Wait for a message to arrive
        while (rxMailbox_.empty()) {
            std::this_thread::sleep_for(THREAD_SLEEP_MILLISEC);
        }

        // Grab message from queue
        frame = rxMailbox_.front();

        // Check if message is intended to be received by this node
        rxMutex_.lock();
        bool validMsgReceived = (rxMsgMap_ != nullptr) && rxMsgMap_->contains(static_cast<HWBRIDGE::CANID>(frame.can_id));
        rxMutex_.unlock();

        if (validMsgReceived) {
            // Extract message signals and put into RX message map
            rxMutex_.lock();
            bool msgUnpacked = HWBRIDGE::unpackCANMsg(frame.data, static_cast<HWBRIDGE::CANID>(frame.can_id), rxMsgMap_);
            rxMutex_.unlock();

            if (msgUnpacked) {
                // If message is one-shot, process message
                if ((rxOneShotMsgHandler_ != nullptr) && rxOneShotMsgHandler_->contains(static_cast<HWBRIDGE::CANID>(frame.can_id))) {
                    if (rxOneShotMsgHandler_->at(static_cast<HWBRIDGE::CANID>(frame.can_id))() == false) {
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to process CAN message");
                    }
                }
                // Otherwise message is streamed
                else {
                }
            }
            else {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to unpack CAN message");
            }
        }

        // Otherwise invalid message was received
        else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface received invalid CAN message");
        }

        // Remove message from queue
        rxMailbox_.pop();
    }
}

void CANInterface::txProcessor(void) {
    struct can_frame frame;
    int bytesSent;

    while (true) {
        auto startTime = std::chrono::steady_clock::now();

        // Send all one-shot messages that were queued
        while (!txMailboxOneShot_.empty()) {
            txMailboxOneShotMutex_.lock();
            frame = txMailboxOneShot_.front();
            bytesSent = write(socketHandle_, &frame, sizeof(struct can_frame));
            if (bytesSent != sizeof(struct can_frame)) {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to send CAN message");
            }
            txMailboxOneShot_.pop();
            txMailboxOneShotMutex_.unlock();
        }

        // Send all streamed messages
        if (txMsgMap_ != nullptr) {
            for (auto it = txMsgMap_->begin(); it != txMsgMap_->end(); it++) {
                HWBRIDGE::CANID msgID          = it->first;
                HWBRIDGE::CANMsgData_t msgData = {0};
                size_t len                     = 0;

                txMutex_.lock();
                bool msgPacked = HWBRIDGE::packCANMsg(msgData.raw, msgID, txMsgMap_, len);
                txMutex_.unlock();

                if (msgPacked) {
                    // Send message
                    frame.can_id = static_cast<canid_t>(msgID);
                    memcpy(frame.data, &msgData, len);
                    bytesSent = write(socketHandle_, &frame, sizeof(struct can_frame));
                    if (bytesSent != sizeof(struct can_frame)) {
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to send CAN message");
                    }
                } 
                else {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN interface failed to pack CAN message");
                }
            }
        }

        std::this_thread::sleep_until(startTime + TX_PERIOD_MILLISEC);
    }
}

bool CANInterface::sendOneShotMessage(struct can_frame& frame) {
    // Put message in queue
    if (!rxMailboxMutex_.try_lock_for(MUTEX_LOCK_TIMEOUT)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "CAN wrapper failed to lock mutex, resulting in a lost CAN frame!");
        return false;
    }
    txMailboxOneShot_.push(frame);
    rxMailboxMutex_.unlock();
    return true;
}

bool CANInterface::setTXSignalValue(HWBRIDGE::CANID msgID, HWBRIDGE::CANSIGNAL signalName, HWBRIDGE::CANSignalValue_t signalValue) {
    txMutex_.lock();
    bool success = (txMsgMap_ != nullptr) && txMsgMap_->setSignalValue(msgID, signalName, signalValue);
    txMutex_.unlock();
    return success;
}

bool CANInterface::getRXSignalValue(HWBRIDGE::CANID msgID, HWBRIDGE::CANSIGNAL signalName, HWBRIDGE::CANSignalValue_t &signalValue) {
    rxMutex_.lock();
    bool success = (rxMsgMap_ != nullptr) && rxMsgMap_->getSignalValue(msgID, signalName, signalValue);
    rxMutex_.unlock();
    return success;
}

}
