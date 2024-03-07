#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <csignal>
#include <aubo_ros_control/aubo_hardware_interface.h>

std::unique_ptr<AuboHardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";
  g_hw_interface.reset();
  exit(signum);
}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "aubo_hardware_interface");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;

  g_hw_interface.reset(new AuboHardwareInterface);

  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time = 1.0 / static_cast<double>(g_hw_interface->getControlFrequency());

  // Run as fast as possible
  while (ros::ok())
  {
    // Receive current state from robot
    g_hw_interface->read(timestamp, period);


    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period);

    g_hw_interface->write(timestamp, period);
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}