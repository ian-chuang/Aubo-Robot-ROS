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
  AuboPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info)
  {
    // check if the system interface was initialized correctly
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
    joint_position_write_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_position_read_state_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_velocity_read_state_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_position_command_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_positions_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_velocities_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    joint_efforts_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    servoj_time_ = 0.0;
    servoj_buffer_time_ = 0.0;
    servoj_total_delay_time_ = 0.0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // Aubo has one position command interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("AuboPositionHardwareInterface"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // confirm that the command interface is of the correct type
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("AuboPositionHardwareInterface"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Aubo has two state interfaces, position and velocity
      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("AuboPositionHardwareInterface"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      // confirm that the first state interface is position
      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("AuboPositionHardwareInterface"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
      // confirm that the second state interface is velocity
      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(rclcpp::get_logger("AuboPositionHardwareInterface"),
                     "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                     joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
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
      // export position and velocity state interfaces, joint_positions_ and joint_velocities_ are references
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> AuboPositionHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      // export position command interface, joint_position_command_ is a reference
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn
  AuboPositionHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Starting ...please wait...");

    // The robot's IP address.
    const std::string robot_ip = info_.hardware_parameters["robot_ip"];

    // servoj parameters initialization
    const int servoj_frequency = stoi(info_.hardware_parameters["servoj_frequency"]);
    const double servoj_time = 1.0 / servoj_frequency;
    const double servoj_smooth_scale = stod(info_.hardware_parameters["servoj_smooth_scale"]);
    const double servoj_delay_scale = stod(info_.hardware_parameters["servoj_delay_scale"]);
    const double servoj_buffer_time = stod(info_.hardware_parameters["servoj_buffer_time"]);
    servoj_time_ = servoj_time;
    servoj_buffer_time_ = servoj_buffer_time;

    // max joint limits
    const double max_vel_limit = stod(info_.hardware_parameters["max_vel_limit"]);
    const double max_acc_limit = stod(info_.hardware_parameters["max_acc_limit"]);
    const double max_jerk_limit = stod(info_.hardware_parameters["max_jerk_limit"]);
    for (int i = 0; i < 6; i++)
    {
      // set max limits for each joint (same for all joints for now)
      max_vel_limit_[i] = max_vel_limit;
      max_acc_limit_[i] = max_acc_limit;
      max_jerk_limit_[i] = max_jerk_limit;
    }

    // init ruckig otg
    otg_ = std::make_unique<ruckig::Ruckig<6>>(servoj_time);

    // initialize driver
    aubo_driver_.reset(new AuboDriver(
        robot_ip,
        3.2,  // make this as high as possible to avoid joint limits
        10.0, // make this as high as possible to avoid joint limits
        servoj_time,
        servoj_smooth_scale,
        servoj_delay_scale));

    // connect to robot
    aubo_driver_->connectRobot();
    // connect servoj
    aubo_driver_->connectServoJ();

    RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  AuboPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    // fill initial joint positions
    aubo_driver_->getJointPositions(joint_positions_); // read current joint positions
    joint_position_command_ = joint_positions_;

    // init read/write
    joint_position_write_command_ = joint_positions_;
    joint_position_read_state_ = joint_positions_;
    joint_velocity_read_state_ = {0, 0, 0, 0, 0, 0};

    // initialize ruckig otg input
    otg_input_.current_position = joint_positions_;
    otg_input_.current_velocity = {0, 0, 0, 0, 0, 0};
    otg_input_.current_acceleration = {0, 0, 0, 0, 0, 0};
    otg_input_.target_position = joint_positions_;
    otg_input_.target_velocity = {0, 0, 0, 0, 0, 0};
    otg_input_.target_acceleration = {0, 0, 0, 0, 0, 0};
    otg_input_.max_velocity = max_vel_limit_;
    otg_input_.max_acceleration = max_acc_limit_;
    otg_input_.max_jerk = max_jerk_limit_;

    // start the communication thread
    communication_thread_is_running_.store(true);
    communication_thread_ = std::thread([this] { this->background_task(); });

    RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  AuboPositionHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    // end the communication thread
    communication_thread_is_running_.store(false);
    communication_thread_.join();
    if (communication_thread_.joinable())
    {
      communication_thread_.join();
    }

    // cleanup aubo_driver (will call destructor AuboDriver::~AuboDriver()
    aubo_driver_.reset();

    RCLCPP_INFO(rclcpp::get_logger("AuboPositionHardwareInterface"), "System successfully stopped!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type AuboPositionHardwareInterface::read(const rclcpp::Time &time,
                                                                      const rclcpp::Duration &period)
  {
    joint_read_state_mutex_.lock();
    joint_positions_ = joint_position_read_state_;
    joint_velocities_ = joint_velocity_read_state_;
    joint_read_state_mutex_.unlock();
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type AuboPositionHardwareInterface::write(const rclcpp::Time &time,
                                                                       const rclcpp::Duration &period)
  {
    joint_position_write_command_mutex_.lock();
    joint_position_command_ = joint_position_write_command_;
    joint_position_write_command_mutex_.unlock();
    return hardware_interface::return_type::OK;
  }

  void AuboPositionHardwareInterface::background_task()
  {    
    while (communication_thread_is_running_.load())
    {
      // start time
      auto start_time = std::chrono::steady_clock::now();

      // update ruckig otg input
      joint_position_write_command_mutex_.lock();
      otg_input_.target_position = joint_position_write_command_;
      joint_position_write_command_mutex_.unlock();

      // update ruckig otg output
      auto result = otg_->update(otg_input_, otg_output_);
      if (result < 0)
      {
        RCLCPP_FATAL(rclcpp::get_logger("AuboPositionHardwareInterface"), "Ruckig OTG failed to update.");
      }
      // pass the output to the input for the next update
      otg_output_.pass_to_input(otg_input_);

      // read joint positions and velocities
      // HACK: don't actually read from the robot, just use otg_input_ as the new state
      joint_read_state_mutex_.lock();
      joint_position_read_state_ = otg_input_.current_position;
      joint_velocity_read_state_ = otg_input_.current_velocity;
      joint_read_state_mutex_.unlock();

      // call servoj to command the robot to joint position command
      servoj_total_delay_time_ = aubo_driver_->servoJ(otg_output_.new_position, false);

      // calculate the delay before sending the next command to the robot
      // to avoid sending commands too fast
      // HACK: delay is thresholded to be within 80% and 120% of the control period
      auto period = std::chrono::duration_cast<std::chrono::duration<double>>(
                        std::chrono::steady_clock::now() - start_time).count();
      double delay = servoj_total_delay_time_ - servoj_buffer_time_ - period;
      delay = std::min(delay, (servoj_time_ * 1.2) - period);
      delay = std::max(delay, (servoj_time_ * 0.8) - period);
      delay = std::max(delay, 0.0);
      // sleep for the calculated delay
      usleep(delay * 1000000);
    }

  } // namespace aubo_driver

#include "pluginlib/class_list_macros.hpp"

  PLUGINLIB_EXPORT_CLASS(aubo_driver::AuboPositionHardwareInterface, hardware_interface::SystemInterface)