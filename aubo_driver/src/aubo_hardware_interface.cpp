#include "aubo_driver/aubo_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aubo_driver
{

AuboPositionHardwareInterface::~AuboPositionHardwareInterface()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn
AuboPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (
    hardware_interface::SystemInterface::on_init(system_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("AuboPositionHardwareInterface"),
    "AuboPositionHardwareInterface is initializing.");

  info_ = system_info;

  // initialize
  joint_position_command_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  prev_joint_position_command_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  servoj_time_ = 0.0;
  servoj_buffer_time_ = 0.0;
  servoj_total_delay_time_ = 0.0;
  stopwatch_last_ = std::chrono::steady_clock::now();
  stopwatch_now_ = std::chrono::steady_clock::now();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Aubo has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AuboPositionHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AuboPositionHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AuboPositionHardwareInterface"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("AuboPositionHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AuboPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AuboPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
AuboPositionHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's IP address.
  const std::string robot_ip = info_.hardware_parameters["robot_ip"];

  const int servoj_frequency = stoi(info_.hardware_parameters["servoj_frequency"]);
  const double servoj_time = 1.0 / servoj_frequency;
  const double servoj_smooth_scale = stod(info_.hardware_parameters["servoj_smooth_scale"]);
  const double servoj_delay_scale = stod(info_.hardware_parameters["servoj_delay_scale"]);
  const double servoj_buffer_time = stod(info_.hardware_parameters["servoj_buffer_time"]);
  servoj_time_ = servoj_time;
  servoj_buffer_time_ = servoj_buffer_time;

  const double max_vel_limit = stod(info_.hardware_parameters["max_vel_limit"]);
  const double max_acc_limit = stod(info_.hardware_parameters["max_acc_limit"]);
  const double max_jerk_limit = stod(info_.hardware_parameters["max_jerk_limit"]);  
  for (int i = 0; i < 6; i++)
  {
    max_vel_limit_[i] = max_vel_limit;
    max_acc_limit_[i] = max_acc_limit;
    max_jerk_limit_[i] = max_jerk_limit;
  }

  // init ruckig otg
  otg_ = std::make_unique<ruckig::Ruckig<6>>(servoj_time);

  // initialize driver
  aubo_driver_.reset(new AuboDriver(
      robot_ip,
      3.2, // make this as high as possible to avoid joint limits
      10.0, // make this as high as possible to avoid joint limits
      servoj_time,
      servoj_smooth_scale,
      servoj_delay_scale));

  // connect to robot 
  aubo_driver_->connectRobot();
  // connect servoj
  aubo_driver_->connectServoJ();

  // fill initial joint positions
  aubo_driver_->getJointPositions(joint_positions_);
  joint_position_command_ = joint_positions_;
  prev_joint_position_command_ = joint_positions_;

  RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AuboPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // fill initial joint positions
  aubo_driver_->getJointPositions(joint_positions_);
  joint_position_command_ = joint_positions_;
  prev_joint_position_command_ = joint_positions_;

  otg_input_.current_position = joint_positions_;
  otg_input_.current_velocity = {0,0,0,0,0,0};
  otg_input_.current_acceleration = {0,0,0,0,0,0};
  otg_input_.target_position = joint_positions_;
  otg_input_.target_velocity = {0,0,0,0,0,0};
  otg_input_.target_acceleration = {0,0,0,0,0,0};
  otg_input_.max_velocity = max_vel_limit_;
  otg_input_.max_acceleration = max_acc_limit_;
  otg_input_.max_jerk = max_jerk_limit_;

  // initialize control period stopwatch times
  stopwatch_last_ = std::chrono::steady_clock::now();
  stopwatch_now_ = stopwatch_last_;

  RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AuboPositionHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  aubo_driver_.reset();

  RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AuboPositionHardwareInterface::read(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  // HACK: just set joint positions to the command positions 
  // avoid reading from the robot to reduce latency
  // also avoids having another control loop running for position control
  joint_positions_ = joint_position_command_; 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboPositionHardwareInterface::write(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
{


  // update ruckig otg input
  otg_input_.target_position = joint_position_command_;
  otg_input_.current_position = joint_positions_;
  auto result = otg_->update(otg_input_, otg_output_);
  if (result < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("AuboPositionHardwareInterface"), "Ruckig OTG failed to update.");
  }
  otg_output_.pass_to_input(otg_input_);
  joint_position_command_ = otg_output_.new_position;

  stopwatch_now_ = std::chrono::steady_clock::now();
  double stopwatch_period = std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now_ - stopwatch_last_).count();

  
  // HACK: set the delay before sending the next command to the robot
  // to avoid sending commands too fast
  // delay is thresholded to be within 80% and 120% of the control period
  double delay = servoj_total_delay_time_ - servoj_buffer_time_ - stopwatch_period;
  delay = std::min(delay, (servoj_time_ * 1.2) - stopwatch_period);
  delay = std::max(delay, (servoj_time_ * 0.8) - stopwatch_period);
  delay = std::max(delay, 0.0);

  usleep(delay * 1000000);

  servoj_total_delay_time_ = aubo_driver_->servoJ(joint_position_command_, false);
  stopwatch_last_ = std::chrono::steady_clock::now();

  prev_joint_position_command_ = joint_position_command_;

  // RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "servoj time buffer (s): %f", servoj_total_delay_time_);

  return hardware_interface::return_type::OK;
}

}  // namespace aubo_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboPositionHardwareInterface, hardware_interface::SystemInterface)