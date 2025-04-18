import numpy as np


def astar_torus(grid, start_node, goal_node):
    dims = len(grid.shape)
    M = grid.shape[0]

    parent_map = np.empty(grid.shape, dtype=object)
    heuristic_map = calc_heuristic_map(grid, goal_node)

    explored_heuristic_map = np.full(grid.shape, np.inf)
    distance_map = np.full(grid.shape, np.inf)

    explored_heuristic_map[start_node] = heuristic_map[start_node]
    distance_map[start_node] = 0

    while True:
        current_node = np.unravel_index(np.argmin(explored_heuristic_map), grid.shape)
        if current_node == goal_node or np.isinf(explored_heuristic_map[current_node]):
            break

        explored_heuristic_map[current_node] = np.inf
        neighbors = find_neighbors_nd(current_node, M, dims)

        for neighbor in neighbors:
            if grid[neighbor] == 0 or neighbor == goal_node:
                new_dist = distance_map[current_node] + 1
                if new_dist < distance_map[neighbor]:
                    distance_map[neighbor] = new_dist
                    explored_heuristic_map[neighbor] = (
                        new_dist + heuristic_map[neighbor]
                    )
                    parent_map[neighbor] = current_node

    if parent_map[goal_node] is None:
        return []

    path = [goal_node]
    while path[0] != start_node:
        path.insert(0, parent_map[path[0]])
    return path


def find_neighbors_nd(node, M, dims):
    neighbors = []
    for i in range(dims):
        for delta in [-1, 1]:
            neighbor = list(node)
            neighbor[i] = (neighbor[i] + delta) % M
            neighbors.append(tuple(neighbor))
    return neighbors


def calc_heuristic_map(grid, goal_node):
    dims = len(grid.shape)
    if len(goal_node) != dims:
        msg = f"Goal node has {len(goal_node)} dimensions but grid has {dims}."
        raise ValueError(msg)

    heuristic_map = np.zeros_like(grid, dtype=float)
    it = np.nditer(grid, flags=["multi_index"])
    while not it.finished:
        heuristic_map[it.multi_index] = sum(
            min(
                abs(it.multi_index[i] - goal_node[i]),
                grid.shape[i] - abs(it.multi_index[i] - goal_node[i]),
            )
            for i in range(dims)
        )
        it.iternext()
    return heuristic_map
