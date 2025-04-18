import numpy as np
from planner.joint_limits import JointLimits


def test_joint_limits_initialization():
    # 测试基本初始化
    angle_ranges = [(-90, 90), (-180, 180)]
    limits = JointLimits(angle_ranges)
    assert limits.angle_ranges == angle_ranges

    # 测试空列表
    limits = JointLimits([])
    assert limits.angle_ranges == []

    # 测试单个关节限制
    limits = JointLimits([(-45, 45)])
    assert limits.angle_ranges == [(-45, 45)]


def test_is_within_limits():
    # 创建一个两关节的限制对象
    angle_ranges = [(-90, 90), (-180, 180)]
    limits = JointLimits(angle_ranges)

    # 测试在范围内的角度
    angles = [np.pi / 4, np.pi / 2]  # 45度, 90度
    assert limits.is_within_limits(angles)

    # 测试在边界上的角度
    angles = [np.pi / 2, np.pi]  # 90度, 180度
    assert limits.is_within_limits(angles)

    # 测试超出范围的角度
    angles = [np.pi, 2 * np.pi]  # 180度, 360度
    assert not limits.is_within_limits(angles)

    # 测试负角度
    angles = [-np.pi / 4, -np.pi / 2]  # -45度, -90度
    assert limits.is_within_limits(angles)


def test_is_within_limits_edge_cases():
    # 创建一个单关节的限制对象
    angle_ranges = [(-90, 90)]
    limits = JointLimits(angle_ranges)

    # 测试零角度
    angles = [0]
    assert limits.is_within_limits(angles)

    # 测试接近边界但仍在范围内的角度
    angles = [89.9 * np.pi / 180]  # 89.9度
    assert limits.is_within_limits(angles)

    # 测试接近边界但超出范围的角度
    angles = [90.1 * np.pi / 180]  # 90.1度
    assert not limits.is_within_limits(angles)


def test_is_within_limits_array_length():
    # 创建一个两关节的限制对象
    angle_ranges = [(-90, 90), (-180, 180)]
    limits = JointLimits(angle_ranges)

    # 测试输入角度数量不匹配的情况
    angles = [np.pi / 4]  # 只有一个角度
    assert not limits.is_within_limits(angles)  # 应该返回False而不是抛出异常

    angles = [np.pi / 4, np.pi / 2, np.pi / 4]  # 三个角度
    assert not limits.is_within_limits(angles)  # 应该返回False而不是抛出异常
