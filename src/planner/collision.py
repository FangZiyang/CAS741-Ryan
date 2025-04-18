# planner/collision.py

import numpy as np


def detect_collision(line_seg, circle):
    """
    Detect collision between a line segment and a circular obstacle.

    Parameters:
        line_seg: Two endpoints of the line segment [[x1, y1], [x2, y2]]
        circle: Center and radius of the circular obstacle [cx, cy, r]

    Returns:
        True: collision detected; False: no collision
    """
    a = np.array(line_seg[0])  # Start point
    b = np.array(line_seg[1])  # End point
    c = np.array(circle[:2])   # Circle center
    r = circle[2]              # Radius

    if r <= 0:  # Handle zero or negative radius
        return False

    ab = b - a                 # Line segment vector
    ab_norm = np.linalg.norm(ab)  # Line segment length

    if ab_norm == 0:
        # If start and end points are the same, check if point is inside circle
        return np.linalg.norm(a - c) < r  # Use strict inequality

    # Calculate closest point using projection
    t = np.dot(c - a, ab) / (ab_norm * ab_norm)
    t = np.clip(t, 0, 1)  # Restrict to [0, 1] range (within line segment)
    closest = a + t * ab

    # Check if closest point is inside circle
    dist = np.linalg.norm(closest - c)
    return dist < r  # Use strict inequality, ignore tangent points


def get_occupancy_grid(arm, obstacles, M=100):
    dims = arm.n_links  # N dimensions
    grid_shape = tuple([M] * dims)
    grid = np.zeros(grid_shape, dtype=int)

    # Iterate through all joint combinations
    for idx in np.ndindex(grid.shape):
        # Convert to corresponding joint angles
        angles = [2 * np.pi * idx[i] / M - np.pi for i in range(dims)]

        # Check angle limits
        if arm.joint_limits and not arm.joint_limits.is_within_limits(angles):
            grid[idx] = 1
            continue

        # Update arm pose
        arm.update_joints(angles)

        # Collision check
        collision = False
        points = arm.points
        for k in range(len(points) - 1):
            seg = [points[k], points[k + 1]]
            if any(detect_collision(seg, obs) for obs in obstacles):
                collision = True
                break

        grid[idx] = int(collision)

    return grid