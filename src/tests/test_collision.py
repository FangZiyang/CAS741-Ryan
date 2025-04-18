import pytest
import numpy as np
from planner.collision import detect_collision, get_occupancy_grid

class MockArm:
    def __init__(self, link_lengths, joint_angles=None, joint_limits=None):
        self.n_links = len(link_lengths)
        self.link_lengths = link_lengths
        self.joint_angles = joint_angles if joint_angles is not None else [0] * self.n_links
        self.joint_limits = joint_limits
        self.points = [[0, 0] for _ in range(self.n_links + 1)]
        
    def update_joints(self, angles):
        self.joint_angles = angles
        # 简单更新点的位置
        for i in range(1, self.n_links + 1):
            angle_sum = sum(self.joint_angles[:i])
            self.points[i][0] = self.points[i-1][0] + self.link_lengths[i-1] * np.cos(angle_sum)
            self.points[i][1] = self.points[i-1][1] + self.link_lengths[i-1] * np.sin(angle_sum)

def test_detect_collision():
    # 测试无碰撞情况
    line_seg = [[0, 0], [1, 0]]  # 水平线段
    circle = [0, 2, 1]  # 圆心在(0,2)，半径为1
    assert not detect_collision(line_seg, circle)
    
    # 测试碰撞情况
    line_seg = [[0, 0], [2, 0]]
    circle = [1, 0, 0.5]  # 圆心在线段上
    assert detect_collision(line_seg, circle)
    
    # 测试边界情况
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 1, 1]  # 刚好相切
    assert not detect_collision(line_seg, circle)
    
    # 测试零半径情况
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 0, 0]
    assert not detect_collision(line_seg, circle)
    
    # 测试负半径情况
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 0, -1]
    assert not detect_collision(line_seg, circle)
    
    # 测试点线段情况
    line_seg = [[1, 1], [1, 1]]
    circle = [1, 1, 0.5]
    assert detect_collision(line_seg, circle)

def test_get_occupancy_grid():
    # 创建一个简单的2连杆机械臂
    arm = MockArm([1, 1])  # 两个连杆，长度都是1
    obstacles = [[0, 2, 0.5]]  # 一个圆形障碍物
    
    # 测试网格生成
    grid = get_occupancy_grid(arm, obstacles, M=10)
    
    # 验证网格形状
    assert grid.shape == (10, 10)
    
    # 验证网格值都是0或1
    assert np.all(np.logical_or(grid == 0, grid == 1))
    
    # 测试无障碍物情况
    grid = get_occupancy_grid(arm, [], M=10)
    assert grid.shape == (10, 10)

class MockJointLimits:
    def __init__(self, limits):
        self.limits = limits
        
    def is_within_limits(self, angles):
        return all(low <= angle <= high for angle, (low, high) in zip(angles, self.limits))

def test_get_occupancy_grid_with_limits():
    # 创建带关节限制的机械臂
    arm = MockArm([1, 1])
    arm.joint_limits = MockJointLimits([(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)])
    obstacles = [[0, 2, 0.5]]
    
    # 测试网格生成
    grid = get_occupancy_grid(arm, obstacles, M=10)
    
    # 验证网格形状
    assert grid.shape == (10, 10)
    
    # 验证网格值都是0或1
    assert np.all(np.logical_or(grid == 0, grid == 1))
