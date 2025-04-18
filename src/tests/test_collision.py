import numpy as np
from planner.collision import detect_collision, get_occupancy_grid


class MockArm:
    def __init__(self, link_lengths, joint_angles=None, joint_limits=None):
        self.n_links = len(link_lengths)
        self.link_lengths = link_lengths
        self.joint_angles = (
            joint_angles if joint_angles is not None else [0] * self.n_links
        )
        self.joint_limits = joint_limits
        self.points = [[0, 0] for _ in range(self.n_links + 1)]

    def update_joints(self, angles):
        self.joint_angles = angles
        # Simple update of point positions
        for i in range(1, self.n_links + 1):
            angle_sum = sum(self.joint_angles[:i])
            self.points[i][0] = self.points[i - 1][0] + self.link_lengths[
                i - 1
            ] * np.cos(angle_sum)
            self.points[i][1] = self.points[i - 1][1] + self.link_lengths[
                i - 1
            ] * np.sin(angle_sum)


def test_detect_collision():
    # Test case: no collision
    line_seg = [[0, 0], [1, 0]]  # Horizontal line segment
    circle = [0, 2, 1]  # Circle center at (0,2) with radius 1
    assert not detect_collision(line_seg, circle)

    # Test case: collision occurs
    line_seg = [[0, 0], [2, 0]]
    circle = [1, 0, 0.5]  # Circle center on the line segment
    assert detect_collision(line_seg, circle)

    # Test case: boundary condition
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 1, 1]  # Circle just touching the line
    assert not detect_collision(line_seg, circle)

    # Test case: zero radius
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 0, 0]
    assert not detect_collision(line_seg, circle)

    # Test case: negative radius
    line_seg = [[0, 0], [1, 0]]
    circle = [0.5, 0, -1]
    assert not detect_collision(line_seg, circle)

    # Test case: point segment
    line_seg = [[1, 1], [1, 1]]
    circle = [1, 1, 0.5]
    assert detect_collision(line_seg, circle)


def test_get_occupancy_grid():
    # Create a simple 2-link arm
    arm = MockArm([1, 1])
    obstacles = [[1, 1, 1]]  # Single obstacle at (1,1) with radius 1
    grid_size = 10
    resolution = 0.1

    grid = get_occupancy_grid(arm, obstacles, grid_size, resolution)
    assert grid is not None
    assert grid.shape == (grid_size, grid_size)


class MockJointLimits:
    def __init__(self, limits):
        self.angle_ranges = limits

    def is_within_limits(self, angles):
        return True


def test_get_occupancy_grid_with_limits():
    # Create an arm with joint limits
    arm = MockArm(
        [1, 1],
        joint_limits=MockJointLimits(
            [(-np.pi / 2, np.pi / 2), (-np.pi / 2, np.pi / 2)]
        ),
    )
    obstacles = [[1, 1, 1]]
    grid_size = 10
    resolution = 0.1

    grid = get_occupancy_grid(arm, obstacles, grid_size, resolution)
    assert grid is not None
    assert grid.shape == (grid_size, grid_size)
