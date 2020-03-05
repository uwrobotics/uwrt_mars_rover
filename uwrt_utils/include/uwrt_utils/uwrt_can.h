#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

namespace uwrt_can {

class UWRTCANWrapper {
 public:
  explicit UWRTCANWrapper() {}
  explicit UWRTCANWrapper(const std::string& name, const std::string& interface_name, int thread_sleep_millis = 1) 
    : name_(std::move(name)),
      interface_name_(std::move(interface_name)),
      thread_sleep_millis_(std::chrono::milliseconds(thread_sleep_millis)) {};
  // TODO (wraftus) make other constructor for failover CAN

  ~UWRTCANWrapper() { 
    read_thread_running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
  }

  enum class UWRTCANStatus {
    STATUS_OK = 0,
    SOCKET_CREATE_FAILED,
    SOCKET_BIND_FAILED,
    RECV_MUTEX_TIMEOUT,
    RECV_SIZE_MISMATCH,
    SEND_DATA_OVERSIZED,
    SEND_DATA_FAILED
  };

  UWRTCANStatus init(const std::vector<canid_t>& ids);

  template <class T>
  UWRTCANStatus getLatestFromID(T& data, canid_t id);
  template <class T>
  UWRTCANStatus writeToID(const T& data, canid_t id);
  // TODO (wraftus) add write_and_wait function to write and wait for ack

 private:
  // description name for this wrapper
  std::string name_;

  // variables needed for can socket
  std::string         interface_name_;
  int                 socket_handle_;
  struct sockaddr_can sock_addr_;
  struct ifreq        ifr_;
  
  // variables for wrapper
  std::thread                            read_thread_;
  volatile bool                          read_thread_running_;
  std::map<canid_t, struct can_frame>    recv_map_;
  std::chrono::milliseconds              thread_sleep_millis_;
  std::timed_mutex                       recv_map_mtx_;
  static constexpr std::chrono::milliseconds MUTEX_LOCK_TIMEOUT{1};

  // read function to be run in the thread
  void readSocketTask();

};

} // namespace uwrt_can