import pytest
import numpy as np
from planner.nlink_arm import NLinkArm
from planner.joint_limits import JointLimits

import matplotlib.pyplot as plt


def test_draw_method_runs():
    # 创建一个简单的 2-link arm
    arm = NLinkArm([1.0, 1.0], [0.0, 0.0])

    fig, ax = plt.subplots()

    # 起点和终点测试
    ee_x = [0.0, 2.0]
    ee_y = [0.0, 0.0]

    # 障碍物（x, y, radius）测试
    obstacles = [(1.0, 1.0, 1.0)]

    # 虚线目标角度（index 空间）
    goal_angles = [45, 75]

    # 调用 draw 方法
    arm.draw(ax, obstacles=obstacles, ee_x=ee_x, ee_y=ee_y, goal_angles=goal_angles)

    plt.close(fig)  # 不弹出窗口


def test_joint_limits_passed_in_constructor():
    link_lengths = [1.0, 1.0]
    joint_angles = [0.0, 0.0]
    limits = JointLimits([(-1.0, -0.9), (-1.0, -0.9)])
    arm = NLinkArm(link_lengths, joint_angles, limits)
    with pytest.raises(ValueError):
        NLinkArm(link_lengths, joint_angles, limits)


def test_joint_limits_passed_in_constructor():
    link_lengths = [1.0, 1.0]
    joint_angles = [0.0, 0.0]
    limits = JointLimits([(-1.0, -0.9), (-1.0, -0.9)])  # 故意超出范围

    # 断言：构造函数应当抛出 ValueError
    with pytest.raises(ValueError, match="Initial joint angle out of limits"):
        NLinkArm(link_lengths, joint_angles, limits)


def test_nlink_arm_initialization():
    # 测试正常初始化
    link_lengths = [1.0, 1.0]
    joint_angles = [0.0, 0.0]
    arm = NLinkArm(link_lengths, joint_angles)
    assert arm.n_links == 2
    assert np.array_equal(arm.link_lengths, np.array(link_lengths))
    assert np.array_equal(arm.joint_angles, np.array(joint_angles))

    # 测试带关节限制的初始化
    limits = JointLimits([(-np.pi, np.pi), (-np.pi / 2, np.pi / 2)])
    arm = NLinkArm(link_lengths, joint_angles, limits)
    assert arm.joint_limits == limits

    # 测试错误的输入参数
    with pytest.raises(ValueError):
        NLinkArm([1.0], [0.0, 0.0])  # 连杆数量与关节角度数量不匹配


def test_update_joints():
    arm = NLinkArm([1.0, 1.0], [0.0, 0.0])

    # 测试正常更新
    new_angles = [np.pi / 4, np.pi / 4]
    arm.update_joints(new_angles)
    assert np.allclose(arm.joint_angles, new_angles)

    # 测试带限制的更新
    limits = JointLimits([(-np.pi / 2, np.pi / 2), (-np.pi / 2, np.pi / 2)])
    arm = NLinkArm([1.0, 1.0], [0.0, 0.0], limits)

    # 测试超出限制的更新
    with pytest.raises(ValueError):
        arm.update_joints([np.pi, np.pi])


def test_update_points():
    arm = NLinkArm([1.0, 1.0], [0.0, 0.0])

    # 测试初始位置（水平伸展）
    assert np.allclose(arm.points[0], [0, 0])
    assert np.allclose(arm.points[1], [1, 0])
    assert np.allclose(arm.points[2], [2, 0])

    # 测试45度角位置
    arm.update_joints([np.pi / 4, np.pi / 4])
    assert np.allclose(arm.points[0], [0, 0])
    assert np.allclose(arm.points[1], [np.cos(np.pi / 4), np.sin(np.pi / 4)], rtol=1e-7)
    expected_x = np.cos(np.pi / 4) + np.cos(np.pi / 2)
    expected_y = np.sin(np.pi / 4) + np.sin(np.pi / 2)
    assert np.allclose(arm.points[2], [expected_x, expected_y], rtol=1e-7)


def test_end_effector_position():
    arm = NLinkArm([1.0, 1.0], [0.0, 0.0])
    assert np.allclose(arm.end_effector, [2, 0])

    arm.update_joints([np.pi / 2, 0])
    assert np.allclose(arm.end_effector, [0, 1 + 1])  # 垂直向上

    arm.update_joints([np.pi / 4, -np.pi / 4])
    expected_x = np.cos(np.pi / 4) + np.cos(0)
    expected_y = np.sin(np.pi / 4) + np.sin(0)
    assert np.allclose(arm.end_effector, [expected_x, expected_y], rtol=1e-7)
