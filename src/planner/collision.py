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


# def get_occupancy_grid(arm, obstacles, M=100):
#     grid = np.zeros((M, M), dtype=int)
#
#     theta_list = [2 * np.pi * i / M - np.pi for i in range(M)]
#
#     for i in range(M):
#         for j in range(M):
#             theta1 = theta_list[i]
#             theta2 = theta_list[j]
#
#             # 检查是否在角度限制范围内（不在范围内直接视作碰撞）
#             if arm.joint_limits and not arm.joint_limits.is_within_limits([theta1, theta2]):
#                 grid[i][j] = 1
#                 continue
#
#             arm.update_joints([theta1, theta2])
#             points = arm.points
#
#             collision = False
#             for k in range(len(points) - 1):
#                 seg = [points[k], points[k + 1]]
#                 if any(detect_collision(seg, obs) for obs in obstacles):
#                     collision = True
#                     break
#
#             grid[i][j] = int(collision)
#
#     return grid
def get_occupancy_grid(arm, obstacles, M=100):
    dims = arm.n_links  # N维
    grid_shape = tuple([M] * dims)
    grid = np.zeros(grid_shape, dtype=int)

    # 遍历所有关节组合：
    for idx in np.ndindex(grid.shape):
        # 转换为对应的关节角度：
        angles = [2 * np.pi * idx[i] / M - np.pi for i in range(dims)]

        # 检查角度限制
        if arm.joint_limits and not arm.joint_limits.is_within_limits(angles):
            grid[idx] = 1
            continue

        # 更新机械臂姿态
        arm.update_joints(angles)

        # 碰撞检查
        collision = False
        points = arm.points
        for k in range(len(points) - 1):
            seg = [points[k], points[k + 1]]
            if any(detect_collision(seg, obs) for obs in obstacles):
                collision = True
                break

        grid[idx] = int(collision)

    return grid