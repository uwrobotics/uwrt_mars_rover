#include <uwrt_mars_drive_train/uwrt_mars_drivetrain_fake_signal_test.h>

DrivetrainFakeSignalTest::DrivetrainFakeSignalTest(uint16_t can_id_outgoing, const std::string &name){
  if ((socket_handle_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
    ROS_ERROR("Error opening socket\n");
  }

}

int main(int argc, char **argv){
  ros::init(argc, argv, "fake_signal_test");
  ros::NodeHandle fake_signal_nh;


  while (ros::ok()){
    struct can_frame response_frame = {};
    ssize_t bytes_read = read(socket_handle_, &response_frame, sizeof(struct can_frame));

    if (bytes_read < 0) std::cout << "Nothing is read" << std::endl;

    printf("0x%03X [%d]", response_frame.can_id, response_frame.can_dic);
    for (int i = 0; i < response_frame.can_dic; i++) printf("%02X", response_frame.data[i]);
    printf("\r\n");
  }

  return 0;
}