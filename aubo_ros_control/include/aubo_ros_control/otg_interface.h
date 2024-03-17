#include <hardware_interface/internal/resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface_exception.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <ruckig/ruckig.hpp>

template <typename T>
T saturate(const T val, const T min_val, const T max_val)
{
    return std::min(std::max(val, min_val), max_val);
}

class OTGHandle
{
public:
    OTGHandle(const hardware_interface::JointHandle &jh, const joint_limits_interface::JointLimits &limits, double control_period)
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
            max_vel_limit_ = limits_.max_velocity;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Velocity limits not specified");
        }

        if (limits_.has_acceleration_limits)
        {
            max_acc_limit_ = limits_.max_acceleration;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Acceleration limits not specified");
        }

        if (limits_.has_jerk_limits)
        {
            max_jerk_limit_ = limits_.max_jerk;
        }
        else
        {
            throw std::runtime_error("[PositionJointSaturationHandle] Jerk limits not specified");
        }

        otg_ = std::make_shared<ruckig::Ruckig<1>>(control_period);
        reset();
    }

    std::string getName() const { return jh_.getName(); }

    void enforceLimits(const ros::Duration &period)
    {
        otg_input_.target_position = {jh_.getCommand()};
        otg_input_.current_position = {jh_.getPosition()};
        auto result = otg_->update(otg_input_, otg_output_);
        if (result < 0)
        {
            ROS_ERROR_STREAM("Error in trajectory generator: " << result);
        }
        double new_position = otg_output_.new_position[0];
        new_position = saturate(new_position, min_pos_limit_, max_pos_limit_);
        jh_.setCommand(new_position);
        otg_output_.pass_to_input(otg_input_);
    }

    void reset()
    {
        otg_input_.current_position = {jh_.getPosition()};
        otg_input_.current_velocity = {0};
        otg_input_.current_acceleration = {0};
        otg_input_.target_position = {jh_.getPosition()};
        otg_input_.target_velocity = {0};
        otg_input_.target_acceleration = {0};
        otg_input_.max_velocity = {max_vel_limit_};
        otg_input_.max_acceleration = {max_acc_limit_};
        otg_input_.max_jerk = {max_jerk_limit_};
    }

private:
    hardware_interface::JointHandle jh_;
    joint_limits_interface::JointLimits limits_;
    double min_pos_limit_, max_pos_limit_;
    double max_vel_limit_;
    double max_acc_limit_;
    double max_jerk_limit_;

    std::shared_ptr<ruckig::Ruckig<1>> otg_;
    ruckig::InputParameter<1> otg_input_;
    ruckig::OutputParameter<1> otg_output_;
};

class OTGInterface : public joint_limits_interface::JointLimitsInterface<OTGHandle> {
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