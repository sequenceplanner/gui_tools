import sys
import json
import rclpy
import random
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import Transform
from viz_tools_msgs.srv import ManipulateDynamicMarker

import threading
import yaml

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks:
    type_ = ""
    parent_id = ""
    child_id = ""
    mesh_path = ""
    scale = "1.0"
    frames = []

    trigger_refresh = None
    trigger_update_marker = None
    trigger_remove_marker = None
    trigger_clear = None

class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "marker_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_refresh = self.trigger_refresh
        Callbacks.trigger_update_marker = self.trigger_update_marker
        Callbacks.trigger_remove_marker = self.trigger_remove_marker
        Callbacks.trigger_clear = self.trigger_clear

        self.tf_buffer = tf2_ros.Buffer()
        self.lf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = self.create_client(ManipulateDynamicMarker, "manipulate_dynamic_marker")
        self.request = ManipulateDynamicMarker.Request()
        self.response = ManipulateDynamicMarker.Response()

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Manipulate Dynamic Marker Service not available, waiting again...")
        
        self.get_logger().info("Marker GUI up and running.")

    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        for i in yaml_file.keys():
            Callbacks.frames.append(i)
        Callbacks.frames.sort()

    def trigger_update_marker(self):
        self.request.command = "update"
        self.request.child_id = Callbacks.child_id
        self.request.parent_id = Callbacks.parent_id
        if Callbacks.type_ == "mesh":
            self.request.use_primitive = False
        else:
            self.request.use_primitive = True
        if Callbacks.type_ == "cube":
            self.request.primitive_type = 1
        elif Callbacks.type_ == "sphere":
            self.request.primitive_type = 2
        elif Callbacks.type_ == "cylinder":
            self.request.primitive_type = 3
        else:
            self.request.primitive_type = 0
        self.request.transformation = Transform()
        self.request.color.a = 1.0
        self.request.color.r = random.uniform(0.3, 0.9)
        self.request.color.g = random.uniform(0.3, 0.9)
        self.request.color.b = random.uniform(0.3, 0.9)
        self.request.scale.x = float(Callbacks.scale)
        self.request.scale.y = float(Callbacks.scale)
        self.request.scale.z = float(Callbacks.scale)
        self.request.absolute_mesh_path = Callbacks.mesh_path
        self.manipulate(self.request)

    def trigger_remove_marker(self):
        self.request.command = "remove"
        self.request.child_id = Callbacks.child_id
        self.manipulate(self.request)

    def trigger_clear(self):
        self.request.command = "clear"
        self.request.child_id = Callbacks.child_id
        self.manipulate(self.request)

    def manipulate(self, request):
        
        future = self.client.call_async(request)
        self.get_logger().info("Manipulate Dynamic Marker Request sent: %s" % request)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(
                        "Manipulate Dynamic Marker Result: %s" % response
                    )
                    return response.success
                except Exception as e:
                    self.get_logger().info("Manipulate Dynamic Marker Service call failed with: %r" % (e,))
                    response = future.result()
                    return response.success


class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_manipulate_marker_box())
        self.setLayout(grid)
        self.setWindowTitle("Marker GUI")
        self.resize(600, 250)

    def make_manipulate_marker_box(self):
        combo_box = QGroupBox("manipulate_marker")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(200)

        combo_box_layout = QGridLayout()

        line_edit_1 = QLineEdit("")
        line_edit_label_1 = QLabel("name")

        combo_2_box_label = QLabel("parent_id")
        combo_3_box_label = QLabel("type")

        combo_2 = QComboBox()
        combo_3 = QComboBox()
        combo_3.addItems(["cube", "cylinder", "sphere", "mesh"])

        line_edit_2 = QLineEdit("")

        line_edit_label_2 = QLabel("mesh_path")

        line_edit_3 = QLineEdit("1.0")

        line_edit_label_3 = QLabel("scale")

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2_box_button = QPushButton("update")
        combo_2_box_button.setMaximumWidth(280)
        combo_3_box_button = QPushButton("remove")
        combo_3_box_button.setMaximumWidth(280)
        combo_4_box_button = QPushButton("clear")
        combo_4_box_button.setMaximumWidth(280)

        combo_box_layout.addWidget(line_edit_label_1, 0, 0)
        combo_box_layout.addWidget(line_edit_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box_layout.addWidget(combo_3_box_label, 2, 0)
        combo_box_layout.addWidget(combo_3, 2, 1)
        combo_box_layout.addWidget(combo_3_box_button, 2, 2)
        combo_box_layout.addWidget(combo_4_box_button, 3, 2)
        combo_box_layout.addWidget(line_edit_label_2, 3, 0)
        combo_box_layout.addWidget(line_edit_2, 3, 1)
        combo_box_layout.addWidget(line_edit_label_3, 4, 0)
        combo_box_layout.addWidget(line_edit_3, 4, 1)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.child_id = line_edit_1.text()
            Callbacks.parent_id = combo_2.currentText()
            Callbacks.type_ = combo_3.currentText()
            Callbacks.mesh_path = line_edit_2.text()
            Callbacks.scale = line_edit_3.text()
            Callbacks.trigger_update_marker()

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        def combo_3_box_button_clicked():
            Callbacks.child_id = line_edit_1.text()
            Callbacks.parent_id = combo_2.currentText()
            Callbacks.type_ = combo_3.currentText()
            Callbacks.mesh_path = line_edit_2.text()
            Callbacks.scale = line_edit_3.text()
            Callbacks.trigger_remove_marker()

        combo_3_box_button.clicked.connect(combo_3_box_button_clicked)

        def combo_4_box_button_clicked():
            Callbacks.child_id = line_edit_1.text()
            Callbacks.parent_id = combo_2.currentText()
            Callbacks.type_ = combo_3.currentText()
            Callbacks.mesh_path = line_edit_2.text()
            Callbacks.scale = line_edit_3.text()
            Callbacks.trigger_clear()

        combo_4_box_button.clicked.connect(combo_4_box_button_clicked)

        return combo_box


def main(args=None):
    def launch_node():
        def launch_node_callback_local():
            rclpy.init(args=args)
            node = Ros2Node()
            rclpy.spin(node)
            node.destroy_node()
            rclpy.shutdown()

        t = threading.Thread(target=launch_node_callback_local)
        t.daemon = True
        t.start()

    # Window has to be in the main thread
    def launch_window():
        app = QApplication(sys.argv)
        clock = Window()
        clock.show()
        sys.exit(app.exec_())

    launch_node()
    launch_window()


if __name__ == "__main__":
    main()
