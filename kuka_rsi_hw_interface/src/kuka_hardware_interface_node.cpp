
#include <kuka_rsi_hw_interface/kuka_hardware_interface.h>

int main(int argc, char** argv)
{
  ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");

  ros::init(argc, argv, "kuka_rsi_hardware_interface");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle r_nh("rsi");

  kuka_rsi_hw_interface::KukaHardwareInterface kuka_rsi_hw_interface;
  kuka_rsi_hw_interface.init(nh, r_nh);
  controller_manager::ControllerManager controller_manager(&kuka_rsi_hw_interface, nh);

restart:
  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  kuka_rsi_hw_interface.start();

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  // Run as fast as possible
  while (ros::ok())
  // while (!g_quit)
  {
    // Receive current state from robot
    if (!kuka_rsi_hw_interface.read())
    {
      ROS_FATAL_NAMED("kuka_hardware_interface", "Failed to read state from robot. Shutting down!");
      // ros::shutdown();
      ros::Duration(0.5).sleep();
      goto restart;
    }

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // Update the controllers
    controller_manager.update(timestamp, period);

    // Send new setpoint to robot
    kuka_rsi_hw_interface.write(timestamp, period);
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  return 0;
}
