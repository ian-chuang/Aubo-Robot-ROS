#include "aubo_ros_control/aubo_hardware_interface.h"

AuboHardwareInterface::AuboHardwareInterface()
    : joint_positions_({0, 0, 0, 0, 0, 0}),
      joint_velocities_({0, 0, 0, 0, 0, 0}),
      joint_efforts_({0, 0, 0, 0, 0, 0}),
      joint_position_command_({0, 0, 0, 0, 0, 0}),
      prev_joint_position_command_({0, 0, 0, 0, 0, 0}),
      joint_names_(6),
      robot_stopped_(true),
      servoj_total_delay_time_(0)
{
}

bool AuboHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    // The robot's IP address.
    std::string robot_ip;
    if (!robot_hw_nh.getParam("robot_ip", robot_ip))
    {
        ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("robot_ip") << " not given.");
        return false;
    }

    // period at which hardware interface runs at
    int servoj_frequency = robot_hw_nh.param("servoj_frequency", 125);
    if ((servoj_frequency > 125) || (servoj_frequency < 3))
    {
        ROS_ERROR_STREAM("servoj_frequency is " << servoj_frequency << ", must be in range [3, 125]");
        return false;
    }

    // servoj_time is same as control period
    control_frequency_ = servoj_frequency;
    servoj_time_ = 1.0 / static_cast<double>(servoj_frequency);
    ROS_INFO_STREAM("servoj_time is " << servoj_time_);

    // Specify lookahead time for servoing to position in joint space.
    // A longer lookahead time can smooth the trajectory.
    double servoj_smooth_scale = robot_hw_nh.param("servoj_smooth_scale", 1.0);
    if ((servoj_smooth_scale > 1) || (servoj_smooth_scale <= 0))
    {
        ROS_ERROR_STREAM("servoj_smooth_scale is " << servoj_smooth_scale << ", must be in range (0, 1]");
        return false;
    }

    // Specify gain for servoing to position in joint space.
    // A higher gain can sharpen the trajectory.
    double servoj_delay_scale = robot_hw_nh.param("servoj_delay_scale", 1.0);
    if ((servoj_delay_scale > 1) || (servoj_delay_scale <= 0))
    {
        ROS_ERROR_STREAM("servoj_delay_scale is " << servoj_delay_scale << ", must be in range (0, 1]");
        return false;
    }

    // Specify a maxmimum buffer time that servoj can run at
    servoj_buffer_time_ = robot_hw_nh.param("servoj_buffer_time", 0.05);
    if (servoj_buffer_time_ < 0)
    {
        ROS_ERROR_STREAM("servoj_buffer_time is " << servoj_buffer_time_ << ", must be in range [0, inf)");
        return false;
    }

    // get joint limits
    joint_limits_interface::JointLimits joint_limits;
    joint_limits_interface::getJointLimits("joint", robot_hw_nh, joint_limits);

    // Names of the joints. Usually, this is given in the controller config file.
    if (!robot_hw_nh.getParam("joints", joint_names_))
    {
        ROS_ERROR_STREAM("Cannot find required parameter " << robot_hw_nh.resolveName("joints")
                                                           << " on the parameter server.");
        return false;
    }

    // Create ros_control interfaces
    for (std::size_t i = 0; i < joint_positions_.size(); ++i)
    {
        ROS_INFO_STREAM("Registering handles for joint " << joint_names_[i]);
        // Create joint state interface for all joints
        js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                          &joint_velocities_[i], &joint_efforts_[i]));

        // Create joint position control interface
        pj_interface_.registerHandle(
            hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));

        // connect and register the joint position saturation interface
        pjs_interface_.registerHandle(
            PositionJointSaturationHandle(pj_interface_.getHandle(joint_names_[i]), joint_limits));
    }

    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&pjs_interface_);

    // initialize driver
    aubo_driver_.reset(new AuboDriver(
        robot_ip,
        5.0, // make this as high as possible to avoid joint limits
        10.0, // make this as high as possible to avoid joint limits
        servoj_time_,
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

    // initialize control period stopwatch times
    stopwatch_last_ = std::chrono::steady_clock::now();
    stopwatch_now_ = stopwatch_last_;

    return true;
}

void AuboHardwareInterface::read(const ros::Time &time, const ros::Duration &period)
{
    // HACK: just set joint positions to the command positions 
    // avoid reading from the robot to reduce latency
    // also avoids having another control loop running for position control
    joint_positions_ = joint_position_command_; 
}

void AuboHardwareInterface::write(const ros::Time &time, const ros::Duration &period)
{


    // pjs_interface_.enforceLimits(period);


    stopwatch_now_ = std::chrono::steady_clock::now();
    double stopwatch_period = std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now_ - stopwatch_last_).count();

    
    // HACK: set the delay before sending the next command to the robot
    // to avoid sending commands too fast
    // delay is thresholded to be within 90% and 110% of the control period
    double delay = servoj_total_delay_time_ - servoj_buffer_time_ - stopwatch_period;
    delay = std::min(delay, (servoj_time_ * 1.2) - stopwatch_period);
    delay = std::max(delay, (servoj_time_ * 0.8) - stopwatch_period);
    delay = std::max(delay, 0.0);

    if (!compareJoints(joint_position_command_, prev_joint_position_command_)) {
        usleep(delay * 1000000);

        servoj_total_delay_time_ = aubo_driver_->servoJ(joint_position_command_, false);
        prev_joint_position_command_ = joint_position_command_;
        robot_stopped_ = false;

        ROS_INFO("servoj time buffer (s): %f", servoj_total_delay_time_);
    }
    else if (!robot_stopped_) {
        usleep(delay * 1000000);

        servoj_total_delay_time_ = aubo_driver_->servoJ(joint_position_command_, true);
        robot_stopped_ = true;

        ROS_INFO("servoj time buffer (s): %f", servoj_total_delay_time_);
    }

    

    stopwatch_last_ = std::chrono::steady_clock::now();
}

int AuboHardwareInterface::getControlFrequency() const
{
    return control_frequency_;
}

bool AuboHardwareInterface::compareJoints(const std::array<double, 6UL> &a, const std::array<double, 6UL> &b)
{
    for (int i = 0; i < 6; i++)
    {
        if (std::abs(a[i] - b[i]) > 1e-6)
        {
            return false;
        }
    }
    return true;
}