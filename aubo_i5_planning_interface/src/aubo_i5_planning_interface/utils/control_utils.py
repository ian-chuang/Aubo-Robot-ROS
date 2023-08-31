import numpy as np
from aubo_i5_planning_interface.utils.transform_utils import (
    quat_multiply,
    axisangle2quat,
    quat2axisangle
)

def clip_orientation(orientation, orientation_limit):
    # Create a rotation matrix from the provided quaternion
    axis_angle = quat2axisangle(orientation)

    axis_angle = np.clip(axis_angle, orientation_limit[0], orientation_limit[1])
    
    clipped_orientation = axisangle2quat(axis_angle)
    
    return clipped_orientation

def offset_pose(delta, current_pose, position_limit, orientation_limit):
    current_pos = current_pose[:3]
    current_ori = current_pose[3:]

    delta_pos = delta[:3]
    delta_rot = delta[3:]

    new_pos = current_pos + delta_pos
    new_ori = quat_multiply(current_ori, axisangle2quat(delta_rot))

    new_pos = np.clip(new_pos, position_limit[0], position_limit[1])
    new_ori = clip_orientation(new_ori, orientation_limit)

    return np.concatenate([new_pos, new_ori])