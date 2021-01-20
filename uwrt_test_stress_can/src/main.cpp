#include <ros/ros.h>
#include <uwrt_mars_rover_utils/uwrt_can.h>
#include <thread>
#include <ctime>


void sendCANMessages() {
    constexpr uint16_t CAN_ID = 0x123;
    constexpr ITERATIONS = 1000;
    std::string can_interface = "can0";
    uwrt_mars_rover_utils::UWRTCANWrapper can_wrapper_int = uwrt_mars_rover_utils::UWRTCANWrapper("can_stress_test", can_interface, true);
    uint16_t msg = 0;

    for (int16_t msg = 0; msg < ITERATIONS; msg++) {
        if (can_wrapper_int.writeToID<uint16_t>(msg, CAN_ID)) {
            if (msg%100 == 0)
                ROS_INFO_STREAM("Successfully sent int msg " << msg << " to id 0x123");
        } else {
            ROS_INFO_STREAM("Failed to send int msg " << msg << " to id 0x123")
            exit(EXIT_FAILURE);
        }        
    }
}

void taskTimer() {
    using namespace std::chrono;
    std::this_thread::sleep_for(duration(seconds(30)));

}

int main(int argc, char* argv[]) {
    std::thread t1(&taskTimer);
    sendCANMessages();
    t1.join(); // Waits for the first thread to complete it's 30 second wait before starting sending of second set
    std::thread t2(&taskTimer); 
    sendCANMessages(); 
    t2.join();

    ros::init(argc, argv, "uwrt_test_stress_can");
}