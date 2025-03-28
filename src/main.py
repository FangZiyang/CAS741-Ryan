# main.py

import sys
import time

from math import pi
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout,
    QWidget, QComboBox, QMessageBox
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from examples import get_all_examples
from planner.nlink_arm import NLinkArm
from planner.collision import get_occupancy_grid
from planner.astar_planner import astar_torus
from src.planner.smooth_angle_pairs import smooth_angle_pairs
from ui.trajectory_plot import TrajectoryPlotWindow





class ArmPlannerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2D Robotic Arm Path Planning Demo")
        self.setGeometry(100, 100, 900, 700)
        self.M = 100
        self.examples = get_all_examples()
        self.current_example_name = "Example 1"
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        self.canvas = FigureCanvas(Figure(figsize=(6, 6)))
        layout.addWidget(self.canvas)
        self.ax = self.canvas.figure.subplots()

        # 示例选择下拉框
        self.example_selector = QComboBox()
        self.example_selector.addItems(self.examples.keys())
        self.example_selector.currentTextChanged.connect(self.select_example)
        layout.addWidget(self.example_selector)

        # 运行按钮
        self.run_button = QPushButton("Run Path Planning")
        self.run_button.clicked.connect(self.run_example)
        layout.addWidget(self.run_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.select_example(self.current_example_name)

    def select_example(self, name):
        self.current_example_name = name
        self.visualize_static()

    def visualize_static(self):
        """
        显示当前机械臂初始姿态和障碍物
        """
        self.ax.clear()
        ex = self.examples[self.current_example_name]
        arm = NLinkArm(ex["link_lengths"], ex["joint_angles"])
        arm.draw(self.ax, obstacles=ex["obstacles"])
        self.canvas.draw()

    def run_example(self):
        """
        执行路径规划：先弹出轨迹图窗口，再播放动画
        """
        ex = self.examples[self.current_example_name]
        arm = NLinkArm(ex["link_lengths"], ex["joint_angles"])
        grid = get_occupancy_grid(arm, ex["obstacles"], self.M)
        route = astar_torus(grid, ex["start"], ex["goal"])

        if not route:
            QMessageBox.warning(self, "End-effector trajectory", "Path inaccessible")
            return

        # 提取并整体平滑角度序列
        theta1_list = [2 * pi * node[0] / self.M - pi for node in route]
        theta2_list = [2 * pi * node[1] / self.M - pi for node in route]
        theta1_smooth, theta2_smooth = smooth_angle_pairs(theta1_list, theta2_list, sigma=1.2)

        # 提前计算末端轨迹，并存储轨迹点（供轨迹图和动画共用）
        ee_x, ee_y = [], []
        trajectory = list(zip(theta1_smooth, theta2_smooth))  # 所有平滑后的角度对
        for t1, t2 in trajectory:
            arm.update_joints([t1, t2])
            arm.update_points()
            ee = arm.end_effector
            ee_x.append(ee[0])
            ee_y.append(ee[1])

        # # 弹出轨迹窗口，使用真实平滑角度路径计算结果
        # self.traj_window = TrajectoryPlotWindow(ee_x, ee_y, ex["obstacles"])
        # self.traj_window.show()

        # 播放动画（用同一组角度）
        self.ax.clear()
        for t1, t2 in trajectory:
            arm.update_joints([t1, t2])
            arm.draw(self.ax, ex["obstacles"])
            self.canvas.draw()
            QApplication.processEvents()

        # # 弹出轨迹窗口，使用真实平滑角度路径计算结果
        time.sleep(5)
        self.traj_window = TrajectoryPlotWindow(ee_x, ee_y, ex["obstacles"])
        self.traj_window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ArmPlannerGUI()
    gui.show()
    sys.exit(app.exec_())
