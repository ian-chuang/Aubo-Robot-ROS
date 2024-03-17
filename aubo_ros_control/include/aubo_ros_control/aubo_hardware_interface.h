#ifndef AUBO_HARDWARE_INTERFACE_H
#define AUBO_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <aubo_ros_control/otg_interface.h>
#include <aubo_ros_control/aubo_driver.h>
#include <chrono>
#include <mutex>

/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class AuboHardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  AuboHardwareInterface();
  virtual ~AuboHardwareInterface() = default;
  /*!
   * \brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns True, if the setup was performed successfully
   */
  virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;
  /*!
   * \brief Read method of the control loop. Reads data from the robot and handles and
   * publishes the information as needed.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time &time, const ros::Duration &period) override;
  /*!
   * \brief Write method of the control loop. Writes target joint positions to the robot.
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time &time, const ros::Duration &period) override;

  /*!
   * \brief Returns the control frequency of the control loop.
   *
   * \returns Control frequency in Hz
   */
  int getControlFrequency() const;

  /*!
   * \brief Returns bool if two joint arrays are very close to each other
   *
   * \returns bool
   */
   bool compareJoints(const std::array<double, 6UL> &a, const std::array<double, 6UL> &b);

protected:

  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  OTGInterface otg_interface_;

  std::array<double, 6UL> joint_position_command_;
  std::array<double, 6UL> joint_positions_;
  std::array<double, 6UL> joint_velocities_;
  std::array<double, 6UL> joint_efforts_;

  std::array<double, 6UL> prev_joint_position_command_;
  bool robot_stopped_ = true;

  std::vector<std::string> joint_names_;

  std::unique_ptr<AuboDriver> aubo_driver_;
  int control_frequency_; 
  double servoj_time_;
  double servoj_buffer_time_;
  double servoj_total_delay_time_;

  std::chrono::_V2::steady_clock::time_point stopwatch_last_;
  std::chrono::_V2::steady_clock::time_point stopwatch_now_;
};

#endif // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED