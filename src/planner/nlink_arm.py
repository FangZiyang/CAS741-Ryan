# planner/nlink_arm.py

import numpy as np
import matplotlib.pyplot as plt
from planner.joint_limits import JointLimits
from matplotlib.patches import Rectangle, RegularPolygon

class NLinkArm:
    def __init__(self, link_lengths, joint_angles, joint_limits=None):
        self.n_links = len(link_lengths)
        if self.n_links != len(joint_angles):
            raise ValueError("Input parameter error")

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.points = [[0, 0] for _ in range(self.n_links + 1)]
        self.lim = sum(link_lengths)

        self.joint_limits = joint_limits
        if joint_limits and not joint_limits.is_within_limits(joint_angles):
            raise ValueError("Initial joint angle out of limits！")

        self.update_points()

    def update_joints(self, joint_angles):
        if self.joint_limits and not self.joint_limits.is_within_limits(joint_angles):
            raise ValueError(f"Updated joint angles {joint_angles} Beyond the limits.")
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        for i in range(1, self.n_links + 1):
            self.points[i][0] = self.points[i - 1][0] + \
                                self.link_lengths[i - 1] * np.cos(np.sum(self.joint_angles[:i]))
            self.points[i][1] = self.points[i - 1][1] + \
                                self.link_lengths[i - 1] * np.sin(np.sum(self.joint_angles[:i]))

        self.end_effector = np.array(self.points[self.n_links]).T

    def draw(self, ax, obstacles=[],ee_x=None,ee_y=None, goal_angles=None):
        ax.clear()

        for obstacle in obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), radius=0.5 * obstacle[2], fc='gray')
            ax.add_patch(circle)

        # ✅ 起点（蓝色小正方形）
        if ee_x and ee_y:
            square_size = 0.08  # 控制正方形大小
            square = Rectangle(
                (ee_x[0], ee_y[0]),
                square_size,
                square_size,
                color='blue',
                label='Start'
            )
            ax.add_patch(square)
            ax.text(
                ee_x[0] + 0.05, ee_y[0] + 0.05,
                "Beginning",
                fontsize=10,
                color='blue'
            )

        # ✅ 终点（黄色五角星，用 RegularPolygon 来模拟五角星）
        if ee_x and ee_y:
            star_size = 0.14
            star = RegularPolygon(
                (ee_x[-1], ee_y[-1]),
                numVertices=6,
                radius=star_size,
                orientation=np.pi / 6,
                color='gold',
                label='Goal'
            )
            ax.add_patch(star)
            ax.text(
                ee_x[-1] + 0.05, ee_y[-1] + 0.05,
                "Ending",
                fontsize=10,
                color='goldenrod'
            )

        for i in range(self.n_links):
            x = [self.points[i][0], self.points[i + 1][0]]
            y = [self.points[i][1], self.points[i + 1][1]]
            ax.plot(x, y, 'r-', linewidth=2)
            ax.plot(self.points[i][0], self.points[i][1], 'ko')

        ax.plot(self.points[-1][0], self.points[-1][1], 'bo')

        if goal_angles:
            theta_list = [2 * np.pi * i / 100 - np.pi for i in range(100)]

            theta_goal = [theta_list[int(idx)] for idx in goal_angles]
            temp_arm = NLinkArm(self.link_lengths, theta_goal)
            points = temp_arm.points
            for i in range(self.n_links):
                x = [points[i][0], points[i + 1][0]]
                y = [points[i][1], points[i + 1][1]]
                ax.plot(x, y, linestyle='--', color='orange', linewidth=2)
                ax.plot(points[i][0], points[i][1], 'o', color='orange')

        ax.set_xlim(-self.lim, self.lim)
        ax.set_ylim(-self.lim, self.lim)
        ax.set_aspect('equal')
        ax.grid(True)