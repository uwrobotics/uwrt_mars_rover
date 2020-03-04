#include <string>
#include<vector>
#include<map>
#include <thread>
#include <mutex>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace uwrt_can {

class UWRTCANWrapper {
 public:
  explicit UWRTCANWrapper(const std::string& name, const std::string& device, int delay_time = 1) 
    : name_(std::move(name)),
      device_(std::move(device)),
      delay_time_(std::chrono::milliseconds(delay_time)) {};
  ~UWRTCANWrapper() { read_thread_running_ = false; }

  bool init(const std::vector<canid_t>& ids);

  bool read(struct can_frame *frame, canid_t id);
  bool write(const struct can_frame *frame);

 private:
  // description name for this wrapper
  std::string name_;

  // variables needed for can socket
  std::string device_;
  int socket_handle_;
  struct sockaddr_can sock_addr_;
  struct ifreq ifr_;
  
  // variables for wrapper
  std::thread read_thread_;
  volatile bool read_thread_running_;
  std::map<canid_t, struct can_frame> recv_map_;
  std::mutex recv_map_mtx_;
  std::chrono::milliseconds delay_time_;

  // read function to be run in the thread
  void rawRead();

};

} // namespace uwrt_can