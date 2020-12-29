#include <ros/console.h>
#include <sys/mman.h>

#include <cstdlib>

#include "uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop_real.h"

bool hasRealtimeKernel() {
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  realtime_file >> has_realtime;
  return has_realtime;
}

void enableRealtimeScheduling(const std::string& node_name, int scheduling_policy) {
  if (!hasRealtimeKernel()) {
    ROS_FATAL_STREAM_NAMED(
        node_name, "Kernel-level realtime scheduling requested but system does not have realtime capabilities!");
    exit(EXIT_FAILURE);
  }

  const int max_thread_priority = sched_get_priority_max(scheduling_policy);
  if (max_thread_priority == -1) {
    ROS_FATAL_STREAM_NAMED(node_name, "Kernel failed to provide kernel max priority!");
    exit(EXIT_FAILURE);
  }

  pthread_t this_thread = pthread_self();
  struct sched_param scheduling_params = {.sched_priority = max_thread_priority};

  int ret = pthread_setschedparam(this_thread, scheduling_policy, &scheduling_params);
  if (0 != ret) {
    ROS_FATAL_STREAM_NAMED(node_name, "Failed to set realtime thread priority to "
                                          << max_thread_priority
                                          << " using pthread_setschedparam. Return Code: " << ret);
    exit(EXIT_FAILURE);
  }

  int current_policy = -1;
  ret = pthread_getschedparam(this_thread, &current_policy, &scheduling_params);
  if (ret != 0) {
    ROS_FATAL_STREAM_NAMED(node_name, "Failed to retrieve current thread scheduling parameters. Return Code: " << ret);
    exit(EXIT_FAILURE);
  }

  if (scheduling_policy != current_policy) {
    ROS_FATAL_STREAM_NAMED(node_name, "Thread Scheduling policy failed to be set to "
                                          << scheduling_policy << ". Currently Set Policy: " << current_policy);
    exit(EXIT_FAILURE);
  }

  ROS_INFO_STREAM_NAMED(node_name, "Successfully set realtime thread scheduling to policy "
                                       << scheduling_policy << " with priority " << max_thread_priority << ".");
}

void preventMemoryPageFaults(const std::string& node_name) {
  const int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (ret != 0) {
    ROS_FATAL_STREAM_NAMED(node_name, "Failed to lock memory pages for realtime execution using mlockall. Error: "
                                          << std::strerror(errno));
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv) {
  std::string node_name = "uwrt_mars_rover_control";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Startup enough spinners to automatically match number of CPU cores
  ros::AsyncSpinner async_spinner(0);
  async_spinner.start();

  //  TODO: I know this param block looks ugly and overly verbose. This is just here until #48 is implemented(soon).
  bool use_realtime_kernel = false;
  bool default_realtime_mode = true;
  bool param_retrieved = nh.param<bool>("use_realtime_kernel", use_realtime_kernel, default_realtime_mode);
  ROS_WARN_STREAM_COND_NAMED(
      !param_retrieved, node_name,
      nh.getNamespace()
          << "/use_realtime_kernel could not be found and loaded from parameter server. Using default value of "
          << default_realtime_mode);

  if (use_realtime_kernel) {
    constexpr auto REALTIME_SCHEDULING_POLICY = SCHED_FIFO;

    enableRealtimeScheduling(node_name, REALTIME_SCHEDULING_POLICY);

    // Prevents RAM from being swapped out
    preventMemoryPageFaults(node_name);

  } else {
    ROS_WARN_STREAM_NAMED(node_name, "Not using kernel-level realtime scheduling. Deadlines may be missed!");
  }

  uwrt_mars_rover_control::MarsRoverHWControlLoopReal rover_hw_control_loop_real(nh);
  rover_hw_control_loop_real.runForeverBlocking();

  async_spinner.stop();

  return EXIT_SUCCESS;
}
