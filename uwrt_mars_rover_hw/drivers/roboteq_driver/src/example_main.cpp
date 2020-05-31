#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "CanopenInterface.hpp"
#include "CommunicationInterface.hpp"
#include "RoboteqController.hpp"

#include <linux/can.h>

static constexpr int32_t TEST_SPEED{300};
static bool TEST_SUCCESS{true};

int main() {

  static constexpr int NUM_DRIVE_JOINTS{2};
  std::array<double, NUM_DRIVE_JOINTS> velocities{};

  // successful command
  // cansend vcan0 581#4803210190010000

  // Passed execptions
  // cansend vcan0 581#2803210190010000
  // cansend vcan0 581#4303210190010000
  // cansend vcan0 581#4803210290010000

  // TODO:
  // 1: Note enough bytes in DLC
  // cansend vcan0 581#48032101

  try
  {  
  std::unique_ptr<roboteq::CanopenInterface> comm = std::make_unique<roboteq::CanopenInterface>(0x01, "vcan0");
  roboteq::RoboteqController motor_controller(std::move(comm));

  velocities[1] = motor_controller.readEncoderMotorSpeed(1);
  std::cout << "velocity: " << velocities[1] << std::endl;

  }
  catch(roboteq::CanopenInterface::StringException &exception)
  {
    TEST_SUCCESS = false;
		std::cerr << "Exception Thrown: (" << exception.getError() << ")\n";
	}

  //   std::unique_ptr<roboteq::CanopenInterface> comm = std::make_unique<roboteq::CanopenInterface>(0x01, "vcan0");
  // roboteq::RoboteqController motor_controller(std::move(comm));

  // static constexpr int NUM_DRIVE_JOINTS{2};
  // uint16_t status_flags;
  // uint16_t fault_flags;
  // std::array<double, NUM_DRIVE_JOINTS> positions{};
  // std::array<double, NUM_DRIVE_JOINTS> velocities{};
  // std::array<double, NUM_DRIVE_JOINTS> efforts{};

  // bool command_successful = motor_controller.releaseShutdown();
  // if (!command_successful) {
  //   std::cout << "Failed to release shutdown.";
  // }

  // using namespace std::chrono_literals;
  // static constexpr auto EXECUTION_PERIOD{1s / 10};  // 10 Hz
  // auto start_time = std::chrono::steady_clock::now();
  // // auto end_time = start_time + 5s;
  // auto next_execution_time = start_time + EXECUTION_PERIOD;

  // motor_controller.setMotorCommand(TEST_SPEED, 1);
  // motor_controller.setVelocity(TEST_SPEED, 1)
  // velocities[1] = motor_controller.readEncoderMotorSpeed(1);
  //     std::cout << "velocity: " << velocities[1] << std::endl;

  // motor_controller.setMotorCommand(-TEST_SPEED, 2);
  //###########################################33
  // while (true) {
  //   bool success = motor_controller.setVelocity(TEST_SPEED, 1);
  //   std::cout << "setvel success: " << success << std::endl;
  //   std::this_thread::sleep_for(1s);
  //   //    next_execution_time += EXECUTION_PERIOD;
  // }

  // velocities[2] = motor_controller.readEncoderMotorSpeed(1);
  // std::cout << "velocity: " << velocities[1] << std::endl;
  // std::this_thread::sleep_until(next_execution_time);
  //###################################################
  //  status_flags = motor_controller.readStatusFlags();
  //  std::cout << "Status Flags: " << status_flags << std::endl;
  //
  //  fault_flags = motor_controller.readFaultFlags();
  //  std::cout << "Fault Flags: " << fault_flags << std::endl;

  // std::this_thread::sleep_until(next_execution_time);

  // using namespace std::chrono_literals;
  // static constexpr auto EXECUTION_PERIOD{1s / 100};  // 100 Hz
  // auto start_time = std::chrono::steady_clock::now();
  // auto end_time = start_time + 5s;
  // auto next_execution_time = start_time;
  // while (std::chrono::steady_clock::now() < end_time) {
  //   // read to all joints
  //   for (int joint_num = 0; joint_num < NUM_DRIVE_JOINTS; joint_num++) {
  //     positions[joint_num] = motor_controller.readAbsoluteEncoderCount(joint_num);
  //     velocities[joint_num] = motor_controller.readEncoderMotorSpeed(joint_num);
  //     efforts[joint_num] = motor_controller.readMotorAmps(joint_num);

  //     static constexpr int NUMBER_FIELD_WIDTH{5};
  //     std::cout << std::left << "joint_num: " << std::setw(NUMBER_FIELD_WIDTH) << joint_num
  //               << "position: " << std::setw(NUMBER_FIELD_WIDTH) << positions[joint_num]
  //               << "velocity: " << std::setw(NUMBER_FIELD_WIDTH) << velocities[joint_num]
  //               << "effort: " << std::setw(NUMBER_FIELD_WIDTH) << efforts[joint_num] << std::endl;
  //   }

  //   // write to all joints
  //   for (int joint_num = 0; joint_num < NUM_DRIVE_JOINTS; joint_num++) {
  //     motor_controller.setMotorCommand(TEST_SPEED, joint_num);
  //     bool command_successful = motor_controller.setVelocity(TEST_SPEED, joint_num);
  //     if (!command_successful) {
  //       bool shutdown_successful{false};
  //       do {
  //         shutdown_successful = motor_controller.emergencyShutdown();
  //         std::cout << "Command write failed. Sending ESTOP!" << std::endl;
  //       } while (!shutdown_successful);
  //       std::cout << "Test FAILED. Aborting program early." << std::endl;
  //       return EXIT_FAILURE;
  //     }
  //   }

  //   next_execution_time += EXECUTION_PERIOD;
  //   std::this_thread::sleep_until(next_execution_time);
  // }

  // // write to all joints
  // for (int joint_num = 1; joint_num <= NUM_DRIVE_JOINTS; joint_num++) {
  //   std::cout << "set motor command 0 on joint#" << joint_num << std::endl;
  //   bool command_successful = motor_controller.setVelocity(0, joint_num);
  //   if (!command_successful) {
  //     bool shutdown_successful{false};
  //     //      do {
  //     std::cout << "Command write failed. Sending ESTOP!" << std::endl;
  //     shutdown_successful = motor_controller.emergencyShutdown();
  //     //      } while (!shutdown_successful);
  //     std::cout << "Test FAILED. Aborting program early." << std::endl;
  //     return EXIT_FAILURE;
  //   }
  // }

  // motor_controller.stopInAllModes(1);
  // motor_controller.stopInAllModes(2);

  // for (int joint_num = 0; joint_num < NUM_DRIVE_JOINTS; joint_num++) {
  //   bool success = motor_controller.stopInAllModes(joint_num);
  //   assert(success);
  //   success = motor_controller.stopInAllModes(joint_num);
  //   assert(success);
  //   (void)success;
  // }

  if (TEST_SUCCESS) {
    std::cout << "Test Success! Program Terminating." << std::endl;
    return EXIT_SUCCESS;
  }
}
