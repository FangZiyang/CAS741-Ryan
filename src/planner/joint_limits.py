# planner/joint_limits.py
import numpy as np


class JointLimits:
    """
    Joint angle limits
    angles: [(min_angle1, max_angle1), (min_angle2, max_angle2)]
    """
    def __init__(self, angle_ranges):
        self.angle_ranges = angle_ranges

    def is_within_limits(self, joint_angles):
        # Check if the number of angles matches
        if len(joint_angles) != len(self.angle_ranges):
            return False

        joint_angles_deg = np.degrees(joint_angles)
        return all(
            (min_angle <= angle <= max_angle)
            for angle, (min_angle, max_angle) in zip(
                joint_angles_deg, self.angle_ranges
            )
        )
