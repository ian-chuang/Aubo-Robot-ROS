#ifndef AUBO_HARDWARE_INTERFACE_HPP_
#define AUBO_HARDWARE_INTERFACE_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// aubo_driver
#include "aubo_driver/aubo_driver.hpp"
#include <ruckig/ruckig.hpp>

namespace aubo_driver
{

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class AuboPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AuboPositionHardwareInterface)
  virtual ~AuboPositionHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

protected:
  std::array<double, 6UL> joint_position_command_;
  std::array<double, 6UL> prev_joint_position_command_;
  std::array<double, 6UL> joint_positions_;
  std::array<double, 6UL> joint_velocities_;
  std::array<double, 6UL> joint_efforts_;

  std::unique_ptr<AuboDriver> aubo_driver_;
  double servoj_time_;
  double servoj_buffer_time_;
  double servoj_total_delay_time_;

  std::unique_ptr<ruckig::Ruckig<6>> otg_;
  ruckig::InputParameter<6> otg_input_;
  ruckig::OutputParameter<6> otg_output_;
  std::array<double, 6UL> max_vel_limit_;
  std::array<double, 6UL> max_acc_limit_;
  std::array<double, 6UL> max_jerk_limit_;

  std::chrono::_V2::steady_clock::time_point stopwatch_last_;
  std::chrono::_V2::steady_clock::time_point stopwatch_now_;

};
}  // namespace aubo_driver

#endif  // AUBO_HARDWARE_INTERFACE_HPP_