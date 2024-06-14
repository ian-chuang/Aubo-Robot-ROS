#ifndef AUBO_HARDWARE_INTERFACE_HPP_
#define AUBO_HARDWARE_INTERFACE_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <mutex>

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

  // on hardware interface initialization
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& system_info) final;

  // export state and command interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  // lifecycle callbacks
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) final;

  // read and write methods
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) final;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) final;

protected:
  // manage joint commands and states
  std::array<double, 6UL> joint_position_command_;
  std::array<double, 6UL> joint_positions_;
  std::array<double, 6UL> joint_velocities_;
  std::array<double, 6UL> joint_efforts_;

  // We use a thread to read/write to the driver so that we dont block the hardware_interface read/write.
  std::thread communication_thread_;
  std::atomic<bool> communication_thread_is_running_;
  void background_task();
  // read and write with mutex
  std::array<double, 6UL> joint_position_write_command_;
  std::array<double, 6UL> joint_position_read_state_;
  std::array<double, 6UL> joint_velocity_read_state_;
  std::mutex joint_write_command_mutex_;
  std::mutex joint_read_state_mutex_;

  // Aubo Driver and parameters
  std::unique_ptr<AuboDriver> aubo_driver_;
  double servoj_time_;
  double servoj_buffer_time_;
  double servoj_total_delay_time_;

  // Ruckig Online Trajectory Generator
  std::unique_ptr<ruckig::Ruckig<6>> otg_;
  ruckig::InputParameter<6> otg_input_;
  ruckig::OutputParameter<6> otg_output_;
  std::mutex otg_mutex_;
  std::array<double, 6UL> max_vel_limit_;
  std::array<double, 6UL> max_acc_limit_;
  std::array<double, 6UL> max_jerk_limit_;

};
}  // namespace aubo_driver

#endif  // AUBO_HARDWARE_INTERFACE_HPP_