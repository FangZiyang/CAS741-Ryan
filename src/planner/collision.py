# planner/collision.py

import numpy as np

def detect_collision(line_seg, circle):
    """
    判断一条线段（连杆）是否与圆形障碍物发生碰撞。

    参数：
        line_seg: 线段的两个端点 [[x1, y1], [x2, y2]]
        circle: 圆形障碍物的中心和半径 [cx, cy, r]

    返回：
        True：碰撞；False：无碰撞
    """
    a = np.array(line_seg[0])  # 线段起点
    b = np.array(line_seg[1])  # 线段终点
    c = np.array(circle[:2])   # 圆心
    r = circle[2]              # 半径

    ab = b - a                 # 线段向量
    ab_norm = np.linalg.norm(ab)  # 线段长度

    if ab_norm == 0:
        # 起终点相同的点，直接判断是否在圆内
        return np.linalg.norm(a - c) <= r

    # 投影公式计算最近点
    t = np.dot(c - a, ab) / ab_norm**2
    t = np.clip(t, 0, 1)  # 限制在 [0, 1] 范围内（在线段内部）
    closest = a + t * ab

    # 判断最近点是否在圆内
    return np.linalg.norm(closest - c) <= r


def get_occupancy_grid(arm, obstacles, M=100):
    """
    生成一个 MxM 的二维网格，标记哪些关节角度组合下会发生碰撞。

    参数：
        arm: NLinkArm 机械臂对象
        obstacles: 障碍物列表，每个为 [x, y, r]
        M: 网格划分密度（越大越精细，计算也更慢）

    返回：
        一个大小为 MxM 的 numpy 数组，值为 0 表示可通行，1 表示碰撞区域
    """
    grid = np.zeros((M, M), dtype=int)

    # 将角度范围 [-pi, pi] 离散为 M 个值
    theta_list = [2 * np.pi * i / M - np.pi for i in range(M)]

    for i in range(M):
        for j in range(M):
            theta1 = theta_list[i]
            theta2 = theta_list[j]

            arm.update_joints([theta1, theta2])  # 更新当前关节角度
            points = arm.points

            # 检查每段连杆是否与任意障碍物碰撞
            collision = False
            for k in range(len(points) - 1):
                seg = [points[k], points[k + 1]]
                if any(detect_collision(seg, obs) for obs in obstacles):
                    collision = True
                    break

            grid[i][j] = int(collision)  # 1 表示有碰撞

    return grid
