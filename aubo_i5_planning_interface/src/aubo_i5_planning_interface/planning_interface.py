import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.srv import SetPlannerParamsRequest, SetPlannerParams
from aubo_i5_planning_interface.utils.control_utils import (
    offset_pose,
)
import numpy as np

_DEFAULT_VELOCITY_SCALE = 0.1
_DEFAULT_ACCELERATION_SCALE = 0.1

class PlanningInterface:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")

        self._position_limits = np.array([
            [0.1, -0.5, 0.1],
            [0.6, 0.5, 1],
        ])
        self._orientation_limits = np.array([
            [-1.57, -1.57, -3.14],
            [1.57, 1.57, 3.14],
        ])

        

    def move_home(
            self,
            vel_scale=_DEFAULT_VELOCITY_SCALE,
            acc_scale=_DEFAULT_ACCELERATION_SCALE
        ) -> None:
        self.arm.set_planner_id("PTP")

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
        self.set_pipeline("pilz_industrial_motion_planner")
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
            delta,
            vel_scale=_DEFAULT_VELOCITY_SCALE,
            acc_scale=_DEFAULT_ACCELERATION_SCALE
        ) -> None:

        current_pose = self.arm.get_current_pose()

        cur_arr = np.array([
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        ])

        target_arr = offset_pose(
            delta, 
            cur_arr, 
            self._position_limits, 
            self._orientation_limits
        )

        target_pose = Pose()
        target_pose.position.x = float(target_arr[0])
        target_pose.position.y = float(target_arr[1])
        target_pose.position.z = float(target_arr[2])
        target_pose.orientation.x = float(target_arr[3])
        target_pose.orientation.y = float(target_arr[4])
        target_pose.orientation.z = float(target_arr[5])
        target_pose.orientation.w = float(target_arr[6])

        self.move_line(target_pose, vel_scale, acc_scale)

    def set_pipeline(self, pipeline_id):
        # Create a service proxy to call the /set_planner_params service
        set_planner_params_service = rospy.ServiceProxy('/set_planner_params', SetPlannerParams)

        # Prepare the request data
        request = SetPlannerParamsRequest()
        request.pipeline_id = pipeline_id
        request.replace = False

        # Call the service and handle the response
        try:
            set_planner_params_service(request)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")