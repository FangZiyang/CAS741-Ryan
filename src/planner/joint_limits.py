# planner/joint_limits.py
import numpy as np


class JointLimits:
    """
    关节角度限制
    angles: [(min_angle1, max_angle1), (min_angle2, max_angle2)]
    """
    def __init__(self, angle_ranges):
        self.angle_ranges = angle_ranges

    def is_within_limits(self, joint_angles):
        # 检查角度数量是否匹配
        if len(joint_angles) != len(self.angle_ranges):
            return False
            
        joint_angles_deg = np.degrees(joint_angles)
        return all(self.angle_ranges[i][0] <= joint_angles_deg[i] <= self.angle_ranges[i][1]
               for i in range(len(joint_angles_deg)))
