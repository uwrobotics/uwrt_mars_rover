#pragma once

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <hw_bridge.h>

#include <unordered_map>
#include <mutex>
#include <thread>

namespace uwrt_mars_rover_utils {

class CANInterface {
    public:
        using CANMsgHandler = bool (*)(void);
        using CANMsgHandlerMap = std::unordered_map<HWBRIDGE::CANID, CANMsgHandler>;

        typedef struct {
            // Message maps and handlers
            HWBRIDGE::CANMsgMap* rxMsgMap;
            HWBRIDGE::CANMsgMap* txMsgMap;
            const CANMsgHandlerMap* rxOneShotMsgHandler;

            // Bus frequency
            uint32_t frequency_hz = HWBRIDGE::ROVER_CANBUS_FREQUENCY_HZ;
        } Config;

        // Initialize CAN interface
        CANInterface(const Config &config);

        // Send a one shot message
        bool sendOneShotMessage(struct can_frame* frame);

        // Update a TX CAN signal
        bool setTXSignalValue(HWBRIDGE::CANID msgID, HWBRIDGE::CANSIGNAL signalName, HWBRIDGE::CANSignalValue_t signalValue);

        // Read a RX CAN signal
        bool getRXSignalValue(HWBRIDGE::CANID msgID, HWBRIDGE::CANSIGNAL signalName, HWBRIDGE::CANSignalValue_t &signalValue);

        // TODO: add dual CAN bus support
    
    private:
        // Socket CAN
        int socketHandle_{};
        struct sockaddr_can sockaddr_{};
        struct ifreq ifreq_{};

        // Threads
        std::thread rxPostmanThread_;
        std::thread rxClientThread_;
        std::thread txProcessorThread_;

        void rxPostman(void);
        void rxClient(void);
        void txProcessor(void);

        // Mutexes
        std::timed_mutex rxMutex_;
        std::timed_mutex txMutex_;
        static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

        // Msg maps and handlers
        HWBRIDGE::CANMsgMap* rxMsgMap_;
        HWBRIDGE::CANMsgMap* txMsgMap_;
        const CANMsgHandlerMap* rxOneShotMsgHandler_;
};
}
