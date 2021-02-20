#include <ros/console.h>
#include <sys/mman.h>
#include <uwrt_mars_rover_control/uwrt_mars_rover_hw_control_loop_real.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

/** hasRealtimeKernel - Determine whether or not kernel has realtime patch by checking /sys/kernel/realtime
 *
 * @return True if /sys/kernel/realtime indicates realtime kernel capabilites, else false.
 */
bool hasRealtimeKernel() {
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  realtime_file >> has_realtime;
  return has_realtime;
}

/** enableRealtimeScheduling - Set the kernel scheduling policy to one of the realtime policies
 *
 * @param node_name This node's name used for rosconsole logger name
 * @param scheduling_policy One of the policies from http://manpages.ubuntu.com/manpages/focal/man7/sched.7.html. Should
 * be one of the realtime policies(SCHED_FIFO, SCHED_RR)
 */
void enableRealtimeScheduling(const std::string& node_name, int scheduling_policy) {
  if (!hasRealtimeKernel()) {
    ROS_FATAL_STREAM_NAMED(
        node_name, "Kernel-level realtime scheduling requested but system does not have realtime capabilities!");
    exit(EXIT_FAILURE);
  }

  // Determine kernel's max scheduling priority
  const int max_thread_priority = sched_get_priority_max(scheduling_policy);
  if (max_thread_priority == -1) {
    ROS_FATAL_STREAM_NAMED(node_name, "Kernel failed to provide kernel max priority information!");
    exit(EXIT_FAILURE);
  }

  // Get handle to this thread
  pthread_t this_thread = pthread_self();
  struct sched_param scheduling_params = {.sched_priority = max_thread_priority};

  // Set this thread to max scheduling priority and change scheduling policy
  int ret = pthread_setschedparam(this_thread, scheduling_policy, &scheduling_params);
  if (0 != ret) {
    ROS_FATAL_STREAM_NAMED(node_name, "Failed to set realtime thread priority to "
                                          << max_thread_priority
                                          << " using pthread_setschedparam. Return Code: " << ret);
    exit(EXIT_FAILURE);
  }

  // Double check that this thread's policy matches what was requested
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

/** preventMemoryPageFaults - prevent page faults caused by critical memory sections being swapped into swap space
 *
 * @param node_name This node's name used for rosconsole logger name
 */
void preventMemoryPageFaults(const std::string& node_name) {
  const int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (ret != 0) {
    ROS_FATAL_STREAM_NAMED(node_name, "Failed to lock memory pages for realtime execution using mlockall. Error: "
                                          << std::strerror(errno));
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char** argv) {
  const std::string node_name = "uwrt_mars_rover_control";
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  // Startup enough spinners to automatically match number of CPU cores
  ros::AsyncSpinner async_spinner(0);
  async_spinner.start();

  const bool default_realtime_mode = true;
  const bool use_realtime_kernel{
      uwrt_mars_rover_utils::getParam(nh, node_name, "use_realtime_kernel", default_realtime_mode)};

  if (use_realtime_kernel) {
    constexpr auto REALTIME_SCHEDULING_POLICY = SCHED_FIFO;

    enableRealtimeScheduling(node_name, REALTIME_SCHEDULING_POLICY);

    preventMemoryPageFaults(node_name);

  } else {
    ROS_WARN_STREAM_NAMED(node_name, "Not using kernel-level realtime scheduling. Deadlines may be missed!");
  }

  uwrt_mars_rover_control::MarsRoverHWControlLoopReal rover_hw_control_loop_real(nh);
  rover_hw_control_loop_real.runForeverBlocking();

  async_spinner.stop();

  return EXIT_SUCCESS;
}
