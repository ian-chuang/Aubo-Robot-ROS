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

  void connectRobot();

  void disconnectRobot();

  void connectServoJ();

  void disconnectServoJ();

  double servoJ(std::array<double, 6UL>  values, bool is_last_point);

  void moveJ(std::array<double, 6UL>  values);

  void getJointPositions(std::array<double, 6UL>  &values);

private:
  std::unique_ptr<ServojInterface> servoj_client_;
  std::unique_ptr<ServiceInterface> robot_client_;

  std::atomic<bool> servoj_connected_;
  std::atomic<bool> robot_connected_;

  double servoj_time_;
  double servoj_smooth_scale_;
  double servoj_delay_scale_;
  double servoj_max_buffer_duration_;

  std::string robot_ip_;
  double max_joint_vel_;
  double max_joint_acc_;
  double max_joint_vel_arr_[6];
  double max_joint_acc_arr_[6];
};

}  // namespace aubo_driver

#endif // AUBO_DRIVER_HPP_