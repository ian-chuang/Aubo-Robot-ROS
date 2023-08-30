import moveit_commander
from geometry_msgs.msg import Pose
from aubo_i5_planning_interface.utils import (
    euler2mat,
    mat2euler,
    mat2quat,
    quat2mat,
)
import numpy as np

_DEFAULT_VELOCITY_SCALE = 0.1
_DEFAULT_ACCELERATION_SCALE = 0.1

class PlanningInterface:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")

        self.space = np.array([
            [0.0, -0.5, 0.0, -1.57, -1.57, -3.14],
            [0.8, 0.5, 1.0, 1.57, 1.57, 3.14],
        ])

    def move_home(
            self,
            vel_scale=_DEFAULT_VELOCITY_SCALE,
            acc_scale=_DEFAULT_ACCELERATION_SCALE
        ) -> None:
        self.arm.set_max_velocity_scaling_factor(vel_scale)
        self.arm.set_max_acceleration_scaling_factor(acc_scale)
        self.arm.set_named_target("home")

        error_code_val, plan, planning_time, error_code = self.arm.plan()
        if not error_code_val:
            return False

        error_code_val = self.arm.execute(plan)
        if not error_code_val:
            return False

        return True
    
    def move_line(
            self, 
            pose: Pose, 
            vel_scale=_DEFAULT_VELOCITY_SCALE, 
            acc_scale=_DEFAULT_ACCELERATION_SCALE
        ) -> None:

        self.arm.set_max_velocity_scaling_factor(vel_scale)
        self.arm.set_max_acceleration_scaling_factor(acc_scale)

        self.arm.set_planner_id("LIN")

        self.arm.set_max_velocity_scaling_factor(vel_scale)
        self.arm.set_max_acceleration_scaling_factor(acc_scale)
        self.arm.set_pose_target(pose)

        error_code_val, plan, planning_time, error_code = self.arm.plan()
        if not error_code_val:
            return False

        error_code_val = self.arm.execute(plan)
        if not error_code_val:
            return False

        return True
    
    def move_offset(
            self,
            pos,
            rot,
            vel_scale=_DEFAULT_VELOCITY_SCALE,
            acc_scale=_DEFAULT_ACCELERATION_SCALE
        ) -> None:

        current_pose = self.arm.get_current_pose()

        current_pos = np.array([
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
        ])
        
        current_quat = np.array([
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        ])

        current_euler = mat2euler(quat2mat(current_quat))
        
        target_pos = current_pos + pos
        target_euler = current_euler + rot

        target_pos = np.clip(target_pos, self.space[0, :3], self.space[1, :3])
        target_euler = np.clip(target_euler, self.space[0, 3:], self.space[1, 3:])

        target_quat = mat2quat(euler2mat(target_euler))

        target_pose = Pose()
        target_pose.position.x = float(target_pos[0])
        target_pose.position.y = float(target_pos[1])
        target_pose.position.z = float(target_pos[2])
        target_pose.orientation.x = float(target_quat[0])
        target_pose.orientation.y = float(target_quat[1])
        target_pose.orientation.z = float(target_quat[2])
        target_pose.orientation.w = float(target_quat[3])

        self.move_line(target_pose, vel_scale, acc_scale)

    