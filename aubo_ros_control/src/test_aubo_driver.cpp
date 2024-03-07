#include "ros/ros.h"
#include "aubo_ros_control/aubo_driver.h"

const double PI = 3.141592653589793238463;

int main(int argc, char **argv)
{
    // servoj params
    std::string ip = "192.168.0.4";
    int servoj_frequency = 125;
    double servoj_buffer_time = 0.05;
    double servoj_smooth_scale = 1;
    double servoj_delay_scale = 1;
    double servoj_max_vel = 1;
    double servoj_max_acc = 1;
    double servoj_time = 1.0 / static_cast<double>(servoj_frequency);

    // movement test params
    double angle = 100; // degrees
    double duration = 10;
    int joint_index = 3;
    angle *= PI / 180;

    ros::init(argc, argv, "test_aubo_driver");
    ros::NodeHandle n;

    AuboDriver driver(
        ip,       // robot ip
        servoj_max_vel,                   // max vel
        servoj_max_acc,                 // max acc
        servoj_time,         // servoJ dt
        servoj_smooth_scale, // servoJ smooth scale
        servoj_delay_scale   // servoJ delay scale
    );

    driver.connectRobot();

    auto values = std::array<double, 6UL>({0, 0, 0, 0, 0, 0});
    driver.moveJ(values);

    driver.connectServoJ();

    int steps = duration / servoj_time;
    for (int step = 0; step < steps; step++)
    {
        values = std::array<double, 6UL>({0, 0, 0, 0, 0, 0});
        values.at(joint_index) = (angle / steps) * step;
        double total_delay_time = driver.servoJ(values, false);

        double delay = std::max(total_delay_time - servoj_buffer_time, servoj_time * 0.6875);

        ROS_INFO("DELAY %f, TOTAL_DELAY_TIME %f", delay, total_delay_time);

        usleep(delay * 1000000);
    }

    values = std::array<double, 6UL>({0, 0, 0, 0, 0, 0});
    values.at(joint_index) = angle;
    driver.servoJ(values, true);

    driver.disconnectServoJ();

    driver.disconnectRobot();

    return 0;
}
