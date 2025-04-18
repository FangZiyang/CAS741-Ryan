import pytest
import numpy as np
from planner.astar_planner import astar_torus, find_neighbors_nd, calc_heuristic_map

def test_find_neighbors_nd():
    # Test neighbor finding in 2D grid
    node = (5, 5)
    M = 10
    dims = 2
    neighbors = find_neighbors_nd(node, M, dims)
    
    # Verify number of neighbors (2 directions per dimension)
    assert len(neighbors) == 4
    
    # Verify neighbor positions are correct
    expected_neighbors = [(4, 5), (6, 5), (5, 4), (5, 6)]
    for neighbor in expected_neighbors:
        assert neighbor in neighbors

def test_calc_heuristic_map():
    # Create a simple 2D grid
    grid = np.zeros((5, 5))
    goal_node = (2, 2)
    
    heuristic_map = calc_heuristic_map(grid, goal_node)
    
    # Verify heuristic map shape
    assert heuristic_map.shape == (5, 5)
    
    # Verify heuristic value at goal is 0
    assert heuristic_map[goal_node] == 0
    
    # Verify heuristic values at other points
    assert heuristic_map[0, 0] == 4  # Manhattan distance
    assert heuristic_map[4, 4] == 4  # Manhattan distance
    
    # Test dimension mismatch case
    with pytest.raises(ValueError) as exc_info:
        calc_heuristic_map(grid, (2, 2, 2))  # 3D goal node with 2D grid
    assert "Goal node has 3 dimensions but grid has 2" in str(exc_info.value)

def test_astar_torus_simple():
    # Create a simple grid without obstacles
    grid = np.zeros((5, 5))
    start_node = (0, 0)
    goal_node = (2, 2)
    
    path = astar_torus(grid, start_node, goal_node)
    
    # Verify path exists
    assert path is not None
    assert len(path) > 0
    assert path[0] == start_node
    assert path[-1] == goal_node

def test_astar_torus_no_path():
    # Create a grid completely blocked by obstacles
    grid = np.ones((5, 5))
    start_node = (0, 0)
    goal_node = (4, 4)
    
    path = astar_torus(grid, start_node, goal_node)
    assert path is None

def test_astar_torus_wrap_around():
    # Test wrap-around case
    grid = np.zeros((5, 5))
    start_node = (0, 0)
    goal_node = (4, 4)
    
    path = astar_torus(grid, start_node, goal_node)
    assert path is not None
    assert len(path) > 0
    assert path[0] == start_node
    assert path[-1] == goal_node
