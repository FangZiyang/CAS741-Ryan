# examples.py
import numpy as np

from src.planner.joint_limits import JointLimits


# examples.py

def get_all_examples():
    return {
        "Example 1": {
            "link_lengths": [1, 1],
            # "joint_angles": [0, 0],
            "joint_limits": JointLimits([(-180, 180), (-180, 180)]),
            # "joint_limits": JointLimits([(0, np.pi/9), (0, np.pi/9)]),
            "obstacles": [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.25]],
            "start": (10, 50),
            "goal": (58, 56),
        },
        "Example 2(with tight angle limit[0, pi/4])": {
            "link_lengths": [1, 1],
            "joint_angles": [0, 0],

            "joint_limits": JointLimits([(0, np.pi/4), (0, np.pi/4)]),
            "obstacles": [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.25]],
            "start": (10, 50),
            "goal": (58, 56),
        },
        "Example 3": {
            "link_lengths": [0.5, 1.5],
            "joint_angles": [0, 0],
            "obstacles": [[0.5, 0.5, 0.4], [-1.0, -0.5, 0.6]],
            "start": (20, 20),
            "goal": (75, 75),
        },
        "Example 4": {
            "link_lengths": [1, 1],
            "joint_angles": [0, 0],
            "obstacles": [[1.0, 1.0, 0.6], [-1.2, 0.3, 0.3], [0.0, -1.0, 0.5]],
            "start": (40, 40),
            "goal": (60, 60),
        },
        "Example 5 (From arm_obstacle_navigation.py)": {
            "link_lengths": [1, 1],
            "joint_angles": [0, 0],
            "obstacles": [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.25]],
            "start": (10, 50),
            "goal": (58, 56),
        },
        "Example 6 (From arm_obstacle_navigation_2.py)": {
            "link_lengths": [0.5, 1.5],
            "joint_angles": [0, 0],
            "obstacles": [[1.75, 0.75, 0.6], [0.55, 1.5, 0.5], [0, -1, 0.7]],
            "start": (10, 50),
            "goal": (58, 56),
        },
        "Example 7(3 rangle)": {
            "link_lengths": [1, 5, 1],
            "joint_angles": [0, 0, 0],
            # "joint_limits": JointLimits([(-np.pi, np.pi), (-np.pi, np.pi)]),
            # "joint_limits": JointLimits([(0, np.pi/9), (0, np.pi/9)]),
            "obstacles": [[1.75, 0.75, 0.6], [0, -1, 0.7]],
            "start": (10, 50, 0),
            "goal": (58, 56, 30),
        },
    }
