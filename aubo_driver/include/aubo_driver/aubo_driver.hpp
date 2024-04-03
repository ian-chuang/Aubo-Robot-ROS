#ifndef AUBO_DRIVER_HPP_
#define AUBO_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "servojinterface.h"
#include "serviceinterface.h"
#include <chrono>
#include <algorithm>
#include <array>

namespace aubo_driver
{

class AuboDriver
{
public:
  AuboDriver(
    const std::string &robot_ip,
    double servoj_time,
    double servoj_max_vel,
    double servoj_max_acc,
    double servoj_smooth_scale,
    double servoj_delay_scale);

  ~AuboDriver();
  
  // Connect to the robot (robot_client_)
  void connectRobot();

  // Disconnect from the robot (robot_client_)
  void disconnectRobot();

  // Connect to the servoj interface (servoj_client_)
  void connectServoJ();

  // Disconnect from the servoj interface (servoj_client_)
  void disconnectServoJ();

  // Send a joint position command to the robot with servoj
  double servoJ(std::array<double, 6UL>  values, bool is_last_point);

  // Send a joint position command to the robot with movej
  void moveJ(std::array<double, 6UL>  values);

  // Get the current joint positions of the robot
  void getJointPositions(std::array<double, 6UL>  &values);

private:
  // robot interfaces for communication
  std::unique_ptr<ServojInterface> servoj_client_;
  std::unique_ptr<ServiceInterface> robot_client_;

  // flags for connection status
  std::atomic<bool> servoj_connected_;
  std::atomic<bool> robot_connected_;

  // parameters for servoj
  double servoj_time_;
  double servoj_smooth_scale_;
  double servoj_delay_scale_;
  double servoj_max_buffer_duration_;

  // parameters for robot
  std::string robot_ip_;
  double max_joint_vel_;
  double max_joint_acc_;
  double max_joint_vel_arr_[6];
  double max_joint_acc_arr_[6];
};

}  // namespace aubo_driver

#endif // AUBO_DRIVER_HPP_