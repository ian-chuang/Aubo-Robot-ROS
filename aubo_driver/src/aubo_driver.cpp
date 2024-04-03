#include "aubo_driver/aubo_driver.hpp"

namespace aubo_driver
{

AuboDriver::AuboDriver(
    const std::string &robot_ip,
    double max_joint_vel,
    double max_joint_acc,
    double servoj_time,
    double servoj_smooth_scale,
    double servoj_delay_scale)
    : robot_ip_(robot_ip),
      max_joint_vel_(max_joint_vel),
      max_joint_acc_(max_joint_acc),
      servoj_time_(servoj_time),
      servoj_smooth_scale_(servoj_smooth_scale),
      servoj_delay_scale_(servoj_delay_scale),
      robot_connected_(false),
      servoj_connected_(false)
{
  for (int i = 0; i < 6; i++)
  {
    // HACK get weird bugs if we don't set these to 0
    max_joint_vel_arr_[i] =  0;
    max_joint_acc_arr_[i] =  0;
  }

  // create clients
  servoj_client_ = std::make_unique<ServojInterface>();
  robot_client_ = std::make_unique<ServiceInterface>();
}

AuboDriver::~AuboDriver() {
  // disconnect from robot and servoj interface if connected
  if (servoj_connected_) {
    RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Disconnecting from servoj interface.");
    disconnectServoJ();
  }
  if (robot_connected_) {
    RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Disconnecting from robot interface.");
    disconnectRobot();
  }
}

void AuboDriver::connectRobot()
{
  // login to robot
  auto status = robot_client_->robotServiceLogin(
      robot_ip_.c_str(),
      8899,
      "SERVOJ_DEMO",
      "SERVOJ_DEMO");
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceLogin failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServiceInterface::robotServiceLogin successful.");

  // get robot mode
  aubo_robot_namespace::RobotWorkMode mode;
  status = robot_client_->robotServiceGetRobotWorkMode(mode);
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceGetRobotWorkMode failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Robot mode: %d", mode);

  // Only call robotStartup if there is a real machine
  if (mode == aubo_robot_namespace::RobotModeReal)
  {
    RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Robot mode is real, starting up robot.");

    // startup robot
    aubo_robot_namespace::ToolDynamicsParam tool_dynamics = {0}; // Kinematic parameters of the tool
    uint8 colli_class = 6;                                       // Arm collision class
    bool read_pos = true;                                        // Whether to read the position of the robotic arm at startup (default enabled)
    bool static_colli_detect = true;                             // Robotic arm static collision detection (default enabled)
    int board_maxacc = 6000;                                     // Max Acceleration of Manipulator
    auto state = aubo_robot_namespace::ROBOT_SERVICE_READY;      // Robot arm service startup status
    status = robot_client_->rootServiceRobotStartup(
        tool_dynamics,
        colli_class,
        read_pos,
        static_colli_detect,
        board_maxacc,
        state);
    if (status != aubo_robot_namespace::ErrnoSucc)
    {
      throw std::runtime_error("[AuboDriver] ServiceInterface::rootServiceRobotStartup failed.");
    }
    RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServiceInterface::rootServiceRobotStartup successful, robot state: %d.", state);
  }
  else
  {
    // don't need to startup robot if in simulated mode
    RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Robot mode is simulated.");
  }

  // initialize global move profile
  status = robot_client_->robotServiceInitGlobalMoveProfile();
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceInitGlobalMoveProfile failed.");
  }

  // define velocity and acceleration limits
  aubo_robot_namespace::JointVelcAccParam maxvel, maxacc;
  for (int i = 0; i < 6; i++)
  {
    maxvel.jointPara[i] = max_joint_vel_;
    maxacc.jointPara[i] = max_joint_acc_;
  }
  // set velocity limits
  status = robot_client_->robotServiceSetGlobalMoveJointMaxVelc(maxvel);
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceSetGlobalMoveJointMaxVelc failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Joint velocity limit set to %f", max_joint_vel_);

  // set acceleration limits
  status = robot_client_->robotServiceSetGlobalMoveJointMaxAcc(maxacc);
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceSetGlobalMoveJointMaxAcc failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "Joint acceleration limit set to %f", max_joint_acc_);

  // startup successful
  robot_connected_ = true;
}

void AuboDriver::disconnectRobot()
{
  // logout robot client
  auto status = robot_client_->robotServiceLogout();
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceLogout failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServiceInterface::robotServiceLogout successful.");

  robot_connected_ = false;
}

void AuboDriver::connectServoJ()
{
  // login to servoj interface
  auto servoj_status = servoj_client_->login(robot_ip_.c_str(), 6000, "test", "1234");
  if (servoj_status != 0)
  {
    throw std::runtime_error("[AuboDriver] ServojInterface::login failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServojInterface::login successful.");

  // Start servoj
  // Note:
  // Entering the servoj mode will also enter the tcp2can transparent transmission mode.
  // At this time, the robot movement cannot be controlled by other methods, such as the teaching button on the teaching pendant
  // Once servojclient disconnects from the servoj server, it will automatically exit the servoj mode
  servoj_status = servoj_client_->startServoj();
  if (servoj_status != 0)
  {
    throw std::runtime_error("[AuboDriver] ServojInterface::startServoj failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServojInterface::startServoj successful.");

  servoj_connected_ = true;
}

void AuboDriver::disconnectServoJ()
{
  // stop servoj
  auto servoj_status = servoj_client_->stopServoj();
  if (servoj_status != 0)
  {
    throw std::runtime_error("[AuboDriver] ServojInterface::stopServoj failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServojInterface::stopServoj successful.");

  // logout servoj client
  servoj_status = servoj_client_->logout();
  if (servoj_status != 0)
  {
    throw std::runtime_error("[AuboDriver] ServojInterface::logout failed.");
  }
  RCLCPP_INFO(rclcpp::get_logger("AuboDriver"), "ServojInterface::logout successful.");

  servoj_connected_ = false;
}

double AuboDriver::servoJ(std::array<double, 6UL> values, bool is_last_point = false)
{  
  // call servoj interface
  double total_delay_time = 0;
  auto servoj_status = servoj_client_->servoj(
      values.data(),
      max_joint_vel_arr_,
      max_joint_acc_arr_,
      servoj_time_,
      servoj_smooth_scale_,
      servoj_delay_scale_,
      total_delay_time,
      is_last_point);

  // check for errors
  if (servoj_status != 0)
  {
    throw std::runtime_error("[AuboDriver] ServojInterface::servoj failed.");
  }

  return total_delay_time;
}

void AuboDriver::moveJ(std::array<double, 6UL> values)
{
  // call movej which is a blocking call
  auto status = robot_client_->robotServiceJointMove(values.data(), true);

  // check for errors
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceJointMove failed.");
  }
}

void AuboDriver::getJointPositions(std::array<double, 6UL> &values)
{
  // get waypoint information
  aubo_robot_namespace::wayPoint_S waypoint;
  auto status = robot_client_->robotServiceGetCurrentWaypointInfo(waypoint); /** this method upates the joint states more quickly **/
  if (status != aubo_robot_namespace::ErrnoSucc)
  {
    throw std::runtime_error("[AuboDriver] ServiceInterface::robotServiceGetCurrentWaypointInfo failed.");
  }

  // set joint values
  for (int i = 0; i < 6; i++)
  {
    values.at(i) = waypoint.jointpos[i];
  }
}

}  // namespace aubo_driver