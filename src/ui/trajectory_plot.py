# ui/trajectory_plot.py

from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt


class TrajectoryPlotWindow(QWidget):
    def __init__(self, ee_x, ee_y, obstacles=None):
        super().__init__()
        self.setWindowTitle("End Trajectory Visualization")
        self.setGeometry(200, 200, 600, 600)

        # ✅ 复制数据（防止受外部影响）
        self.ee_x = list(ee_x)
        self.ee_y = list(ee_y)
        self.obstacles = obstacles or []

        layout = QVBoxLayout()
        self.canvas = FigureCanvas(Figure(figsize=(5, 5)))
        layout.addWidget(self.canvas)
        self.ax = self.canvas.figure.subplots()
        self.setLayout(layout)

        self.draw_plot()

    def draw_plot(self):
        # ✅ 障碍物（灰色圆）
        for obs in self.obstacles:
            circle = plt.Circle((obs[0], obs[1]), radius=0.5 * obs[2], fc='gray', alpha=0.5)
            self.ax.add_patch(circle)

        # ✅ 绘制轨迹线
        self.ax.plot(self.ee_x, self.ee_y, 'r-', linewidth=2)

        # ✅ 起点
        self.ax.plot(self.ee_x[0], self.ee_y[0], 'go', markersize=8)
        self.ax.text(self.ee_x[0] - 0.3, self.ee_y[0] - 0.3, "beginning", color='green', fontsize=12)

        # ✅ 终点
        self.ax.plot(self.ee_x[-1], self.ee_y[-1], 'bo', markersize=8)
        self.ax.text(self.ee_x[-1] + 0.3, self.ee_y[-1], "Ending", color='blue', fontsize=12)

        self.ax.set_title("End-effector trajectory")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.canvas.draw()
