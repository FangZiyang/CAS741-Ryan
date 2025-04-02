# ui/example_info_window.py

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QTextEdit
import json

class ExampleInfoWindow(QWidget):
    def __init__(self, example_name, example_data):
        super().__init__()
        self.setWindowTitle(f"Info: {example_name}")
        self.setGeometry(300, 100, 500, 400)

        layout = QVBoxLayout()
        self.text_box = QTextEdit()
        self.text_box.setReadOnly(True)

        # 处理不能序列化的对象，比如 JointLimits
        serializable_data = {}
        for k, v in example_data.items():
            if hasattr(v, "__class__") and v.__class__.__name__ == "JointLimits":
                # 以可读格式显示关节限制
                serializable_data[k] = v.angle_ranges
            else:
                serializable_data[k] = v

        formatted_json = json.dumps(serializable_data, indent=4)
        self.text_box.setText(formatted_json)
        layout.addWidget(self.text_box)
        self.setLayout(layout)
