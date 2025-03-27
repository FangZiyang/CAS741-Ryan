# planner/astar_planner.py

import numpy as np

def calc_heuristic_map(M, goal_node):
    """
    为每个网格单元预先计算启发式代价（曼哈顿距离 + 环绕最短路径）。

    参数：
        M: 网格尺寸（MxM）
        goal_node: 目标节点的索引坐标 (i, j)

    返回：
        MxM 的启发式代价矩阵
    """
    X, Y = np.meshgrid(range(M), range(M))
    heuristic_map = np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0])

    # 加入环绕优化（考虑角度连续性）
    for i in range(M):
        for j in range(M):
            heuristic_map[i, j] = min(
                heuristic_map[i, j],
                M - i - 1 + heuristic_map[M - 1, j],
                i + heuristic_map[0, j],
                M - j - 1 + heuristic_map[i, M - 1],
                j + heuristic_map[i, 0]
            )
    return heuristic_map


def find_neighbors(i, j, M):
    """
    获取当前点的 4 个环绕邻居（上下左右）。
    网格边界是环形（模 M）。

    返回：
        相邻节点的列表
    """
    return [((i - 1) % M, j), ((i + 1) % M, j),
            (i, (j - 1) % M), (i, (j + 1) % M)]


def astar_torus(grid, start_node, goal_node):
    """
    在二维角度空间网格上使用 A* 算法进行路径搜索。

    参数：
        grid: 网格矩阵（0 表示可通行，1 表示障碍）
        start_node: 起始位置 (i, j)
        goal_node: 目标位置 (i, j)

    返回：
        从 start 到 goal 的路径（坐标列表），若不可达则为空列表
    """
    M = grid.shape[0]
    parent_map = [[None for _ in range(M)] for _ in range(M)]  # 记录路径
    heuristic_map = calc_heuristic_map(M, goal_node)  # 启发式估计

    explored_heuristic_map = np.full((M, M), np.inf)
    distance_map = np.full((M, M), np.inf)

    explored_heuristic_map[start_node] = heuristic_map[start_node]
    distance_map[start_node] = 0

    while True:
        # 找出当前最小估价代价的位置
        current_node = np.unravel_index(np.argmin(explored_heuristic_map), explored_heuristic_map.shape)
        if current_node == goal_node or np.isinf(explored_heuristic_map[current_node]):
            break  # 到达终点 或 无路可走

        explored_heuristic_map[current_node] = np.inf  # 标记为已访问
        i, j = current_node

        for ni, nj in find_neighbors(i, j, M):
            if grid[ni][nj] == 0 or (ni, nj) == goal_node:
                new_dist = distance_map[i][j] + 1
                if new_dist < distance_map[ni][nj]:
                    distance_map[ni][nj] = new_dist
                    explored_heuristic_map[ni][nj] = new_dist + heuristic_map[ni][nj]
                    parent_map[ni][nj] = (i, j)

    # 构建路径
    if parent_map[goal_node[0]][goal_node[1]] is None:
        return []  # 不可达

    path = [goal_node]
    while path[0] != start_node:
        path.insert(0, parent_map[path[0][0]][path[0][1]])
    return path
