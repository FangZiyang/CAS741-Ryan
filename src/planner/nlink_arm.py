# planner/nlink_arm.py

import numpy as np
import matplotlib.pyplot as plt
from planner.joint_limits import JointLimits



class NLinkArm:
    def __init__(self, link_lengths, joint_angles, joint_limits=None):
        self.n_links = len(link_lengths)
        if self.n_links != len(joint_angles):
            raise ValueError("关节角度数量必须与连杆数量一致。")

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.points = [[0, 0] for _ in range(self.n_links + 1)]
        self.lim = sum(link_lengths)

        # ✅ 新增关节限制
        self.joint_limits = joint_limits
        if joint_limits and not joint_limits.is_within_limits(joint_angles):
            raise ValueError("初始关节角度超出限制！")

        self.update_points()

    def update_joints(self, joint_angles):
        if self.joint_limits and not self.joint_limits.is_within_limits(joint_angles):
            raise ValueError(f"更新的关节角度 {joint_angles} 超出限制范围。")
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        for i in range(1, self.n_links + 1):
            self.points[i][0] = self.points[i - 1][0] + \
                                self.link_lengths[i - 1] * np.cos(np.sum(self.joint_angles[:i]))
            self.points[i][1] = self.points[i - 1][1] + \
                                self.link_lengths[i - 1] * np.sin(np.sum(self.joint_angles[:i]))

        self.end_effector = np.array(self.points[self.n_links]).T

    def draw(self, ax, obstacles=[]):
        ax.clear()
        for obstacle in obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), radius=0.5 * obstacle[2], fc='gray')
            ax.add_patch(circle)

        for i in range(self.n_links):
            x = [self.points[i][0], self.points[i + 1][0]]
            y = [self.points[i][1], self.points[i + 1][1]]
            ax.plot(x, y, 'r-', linewidth=2)
            ax.plot(self.points[i][0], self.points[i][1], 'ko')

        ax.plot(self.points[-1][0], self.points[-1][1], 'bo')

        ax.set_xlim(-self.lim, self.lim)
        ax.set_ylim(-self.lim, self.lim)
        ax.set_aspect('equal')
        ax.grid(True)
