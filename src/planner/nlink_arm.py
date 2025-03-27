# planner/nlink_arm.py

import numpy as np
import matplotlib.pyplot as plt

class NLinkArm:
    """
    一个支持任意节数连杆的二维机械臂类。
    可根据关节角度计算末端点位置，并绘制当前姿态。
    """
    def __init__(self, link_lengths, joint_angles):
        # 连杆数量
        self.n_links = len(link_lengths)

        # 检查：连杆数量和关节角度数量必须一致
        if self.n_links != len(joint_angles):
            raise ValueError("关节角度数量必须与连杆数量一致。")

        self.link_lengths = np.array(link_lengths)  # 每节连杆的长度
        self.joint_angles = np.array(joint_angles)  # 每个关节的初始角度（单位：弧度）

        # 所有关节的坐标点（包括起点和末端）
        self.points = [[0, 0] for _ in range(self.n_links + 1)]

        # 绘图时使用的坐标范围（最大连杆总长度）
        self.lim = sum(link_lengths)

        # 根据初始角度，更新机械臂各点的位置
        self.update_points()

    def update_joints(self, joint_angles):
        """
        更新关节角度，并刷新各点坐标。
        """
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        """
        根据当前关节角度和连杆长度，计算每个关节和末端执行器的位置。
        """
        for i in range(1, self.n_links + 1):
            # 当前点的位置 = 上一个点 + 当前连杆的方向向量
            self.points[i][0] = self.points[i - 1][0] + \
                                self.link_lengths[i - 1] * np.cos(np.sum(self.joint_angles[:i]))
            self.points[i][1] = self.points[i - 1][1] + \
                                self.link_lengths[i - 1] * np.sin(np.sum(self.joint_angles[:i]))

        # 保存末端执行器的位置
        self.end_effector = np.array(self.points[self.n_links]).T

    def draw(self, ax, obstacles=[]):
        """
        在传入的 matplotlib 画布 ax 上绘制当前机械臂配置和障碍物。
        参数：
            ax - matplotlib 的子图对象
            obstacles - 圆形障碍物列表，每个元素为 [x, y, r]
        """
        ax.clear()

        # 绘制所有障碍物（灰色圆形）
        for obstacle in obstacles:
            circle = plt.Circle((obstacle[0], obstacle[1]), radius=0.5 * obstacle[2], fc='gray')
            ax.add_patch(circle)

        # 绘制连杆和关节点
        for i in range(self.n_links):
            x = [self.points[i][0], self.points[i + 1][0]]
            y = [self.points[i][1], self.points[i + 1][1]]
            ax.plot(x, y, 'r-', linewidth=2)             # 连杆（红线）
            ax.plot(self.points[i][0], self.points[i][1], 'ko')  # 关节点（黑点）

        # 绘制末端执行器（蓝点）
        ax.plot(self.points[-1][0], self.points[-1][1], 'bo')

        # 设置坐标轴范围与比例
        ax.set_xlim(-self.lim, self.lim)
        ax.set_ylim(-self.lim, self.lim)
        ax.set_aspect('equal')
        ax.grid(True)
