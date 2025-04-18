import pytest
import numpy as np
from planner.astar_planner import astar_torus, find_neighbors_nd, calc_heuristic_map

def test_find_neighbors_nd():
    # 测试2D网格中的邻居查找
    node = (5, 5)
    M = 10
    dims = 2
    neighbors = find_neighbors_nd(node, M, dims)
    
    # 验证邻居数量（每个维度2个方向）
    assert len(neighbors) == 4
    
    # 验证邻居位置是否正确
    expected_neighbors = [(4, 5), (6, 5), (5, 4), (5, 6)]
    for neighbor in expected_neighbors:
        assert neighbor in neighbors

def test_calc_heuristic_map():
    # 创建一个简单的2D网格
    grid = np.zeros((5, 5))
    goal_node = (2, 2)
    
    heuristic_map = calc_heuristic_map(grid, goal_node)
    
    # 验证启发式地图的形状
    assert heuristic_map.shape == (5, 5)
    
    # 验证目标点的启发式值为0
    assert heuristic_map[goal_node] == 0
    
    # 验证其他点的启发式值是否正确
    assert heuristic_map[0, 0] == 4  # 曼哈顿距离
    assert heuristic_map[4, 4] == 4  # 曼哈顿距离
    
    # 测试维度不匹配的情况
    with pytest.raises(ValueError) as exc_info:
        calc_heuristic_map(grid, (2, 2, 2))  # 3D目标节点，但网格是2D的
    assert "Goal node has 3 dimensions but grid has 2" in str(exc_info.value)

def test_astar_torus_simple():
    # 创建一个简单的无障碍物网格
    grid = np.zeros((5, 5))
    start_node = (0, 0)
    goal_node = (2, 2)
    
    path = astar_torus(grid, start_node, goal_node)
    
    # 验证路径存在
    assert path is not None
    assert len(path) > 0
    
    # 验证路径的起点和终点
    assert path[0] == start_node
    assert path[-1] == goal_node

def test_astar_torus_no_path():
    # 创建一个完全被障碍物占据的网格
    grid = np.ones((5, 5))
    start_node = (0, 0)
    goal_node = (2, 2)
    
    path = astar_torus(grid, start_node, goal_node)
    
    # 验证没有找到路径
    assert path == []

def test_astar_torus_wrap_around():
    # 测试环绕情况
    grid = np.zeros((5, 5))
    start_node = (0, 0)
    goal_node = (4, 4)
    
    path = astar_torus(grid, start_node, goal_node)
    
    # 验证路径存在
    assert path is not None
    assert len(path) > 0
    
    # 验证路径的起点和终点
    assert path[0] == start_node
    assert path[-1] == goal_node
