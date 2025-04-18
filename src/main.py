# main.py
from ui.example_info_window import ExampleInfoWindow
import sys
import time

import numpy as np
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
from ui.trajectory_plot import TrajectoryPlotWindow


class ArmPlannerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("2D Robotic Arm Path Planning Demo")
        self.setGeometry(100, 100, 900, 700)
        self.M = 100
        self.examples = get_all_examples()
        self.current_example_name = list(self.examples.keys())[0]
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        self.canvas = FigureCanvas(Figure(figsize=(6, 6)))
        layout.addWidget(self.canvas)
        self.ax = self.canvas.figure.subplots()

        # Example selection dropdown
        self.example_selector = QComboBox()
        self.example_selector.addItems(self.examples.keys())
        self.example_selector.currentTextChanged.connect(self.select_example)
        layout.addWidget(self.example_selector)

        # Run button
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
        Display current robotic arm initial pose and obstacles
        """
        self.ax.clear()
        ex = self.examples[self.current_example_name]
        try:
            theta_list = [2 * np.pi * i / self.M - pi for i in range(self.M)]
            theta_start = [theta_list[idx] for idx in ex["start"]]

            arm = NLinkArm(
                ex["link_lengths"],
                theta_start,
                joint_limits=ex.get("joint_limits")
            )
        except ValueError as e:
            # Key: Show error in popup dialog
            QMessageBox.critical(self, "Initialization Error", str(e))
            return
        arm.draw(self.ax, obstacles=ex["obstacles"], goal_angles=ex["goal"])
        self.canvas.draw()

    def run_example(self):
        """
        Execute path planning: show trajectory plot window first, then play animation
        """
        ex = self.examples[self.current_example_name]
        self.info_window = ExampleInfoWindow(self.current_example_name, ex)

        # Position: Place info window 20 pixels to the right of main window
        main_x = self.x()
        main_y = self.y()
        main_width = self.width()
        self.info_window.move(main_x + main_width + 20, main_y)
        self.info_window.show()

        theta_start = np.radians(ex["start"])  # Convert to radians
        arm = NLinkArm(
            ex["link_lengths"],
            theta_start,
            joint_limits=ex.get("joint_limits")
        )

        grid = get_occupancy_grid(arm, ex["obstacles"], self.M)
        route = astar_torus(grid, tuple(ex["start"]), tuple(ex["goal"]))

        if not route:
            QMessageBox.warning(self, "End-effector trajectory", "Path inaccessible")
            return

        # Calculate end-effector trajectory in advance
        ee_x, ee_y = [], []
        for node in route:
            theta1 = 2 * pi * node[0] / self.M - pi
            theta2 = 2 * pi * node[1] / self.M - pi
            arm.update_joints([theta1, theta2])
            arm.update_points()
            ee = arm.end_effector
            ee_x.append(ee[0])
            ee_y.append(ee[1])

        # Play animation
        self.ax.clear()
        for node in route:
            theta1 = 2 * pi * node[0] / self.M - pi
            theta2 = 2 * pi * node[1] / self.M - pi
            arm.update_joints([theta1, theta2])
            arm.draw(self.ax, ex["obstacles"], ee_x, ee_y)
            self.canvas.draw()
            QApplication.processEvents()

        # Show trajectory window (before animation)
        time.sleep(2)
        self.traj_window = TrajectoryPlotWindow(ee_x, ee_y, ex["obstacles"])
        self.traj_window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = ArmPlannerGUI()
    gui.show()
    sys.exit(app.exec_())
