#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>
#include <joint_limits_interface/joint_limits_interface.h>

template <typename T>
T saturate(const T val, const T min_val, const T max_val)
{
    return std::min(std::max(val, min_val), max_val);
}

class PositionJointSaturationHandle
{
public:
    PositionJointSaturationHandle(const hardware_interface::JointHandle &jh, const joint_limits_interface::JointLimits &limits)
        : jh_(jh),
          limits_(limits)
    {

        if (limits_.has_position_limits)
        {
            min_pos_limit_ = limits_.min_position;
            max_pos_limit_ = limits_.max_position;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Position limits not specified");
        }
        
        if (limits_.has_velocity_limits)
        {
            min_vel_limit_ = -limits_.max_velocity;
            max_vel_limit_ = limits_.max_velocity;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Velocity limits not specified");
        }

        if (limits_.has_acceleration_limits)
        {
            min_acc_limit_ = -limits_.max_acceleration;
            max_acc_limit_ = limits_.max_acceleration;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Acceleration limits not specified");
        }
    }

    std::string getName() const { return jh_.getName(); }

    void enforceLimits(const ros::Duration &period)
    {
        if (std::isnan(prev_cmd_)) {
            prev_cmd_ = jh_.getPosition();
            prev_vel_ = 0.0;
        }

        double des_cmd, des_vel, des_acc;

        des_cmd = jh_.getCommand();
        des_cmd = saturate(des_cmd, min_pos_limit_, max_pos_limit_);

        des_vel = (des_cmd - prev_cmd_) / period.toSec();
        des_vel = saturate(des_vel, min_vel_limit_, max_vel_limit_);

        des_acc = (des_vel - prev_vel_) / period.toSec();
        des_acc = saturate(des_acc, min_acc_limit_, max_acc_limit_);

        double new_cmd, new_vel;

        new_vel = prev_vel_ + des_acc * period.toSec();
        new_cmd = prev_cmd_ + new_vel * period.toSec();

        jh_.setCommand(new_cmd);
        prev_cmd_ = new_cmd;
        prev_vel_ = new_vel;
    }

    void reset()
    {
        prev_cmd_ = std::numeric_limits<double>::quiet_NaN();
        prev_vel_ = 0.0;
    }

private:
    hardware_interface::JointHandle jh_;
    joint_limits_interface::JointLimits limits_;
    double min_pos_limit_, max_pos_limit_;
    double min_vel_limit_, max_vel_limit_;
    double min_acc_limit_, max_acc_limit_;

    double prev_cmd_ = {std::numeric_limits<double>::quiet_NaN()};
    double prev_vel_ = 0.0;
};

class PositionJointSaturationInterface : public joint_limits_interface::JointLimitsInterface<PositionJointSaturationHandle> {
 public:
   void reset()
   {
     for (auto&& resource_name_and_handle : this->resource_map_)
     {
       resource_name_and_handle.second.reset();
     }
   }
   /*\}*/
 };