import sys
import rclpy
from rclpy.client import Client
from rclpy.service import Service
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from ur_tools_msgs.action import URScriptControl
import threading
import yaml
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from ur_script_msgs.srv import DashboardCommand
from rclpy.executors import MultiThreadedExecutor

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks:
    transform = TransformStamped()

    command = ""
    acceleration = ""
    velocity = ""
    with_tcp = ""
    to_target = ""

    frames = []

    trigger_refresh = None
    trigger_move_robot = None
    trigger_stop_robot = None
    trigger_stop = None

class Ros2ActionNode(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "control_gui_action")
        Callbacks.__init__(self)

        Callbacks.trigger_move_robot = self.trigger_move_robot
        # Callbacks.trigger_stop = self.trigger_stop

        self._ur_control_client = ActionClient(self, URScriptControl, "ur_script_controller")
        self.get_logger().warn("UR Script Controller Server not available, wait...")
        self._ur_control_client.wait_for_server()
        self.get_logger().info("UR Script Controller Server online.")

    def trigger_move_robot(self):
        request = URScriptControl.Goal()
        request.command = Callbacks.command
        request.acceleration = float(Callbacks.acceleration)
        request.velocity = float(Callbacks.velocity)
        request.tcp_name = Callbacks.with_tcp
        request.goal_feature_name = Callbacks.to_target
        self.generate_and_send_ur_script(request)

    def generate_and_send_ur_script(self, request):
        self._send_goal_future = self._ur_control_client.send_goal_async(request)

    # def trigger_stop(self):
    #     self._send_goal_future.cancel()

class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "control_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_refresh = self.trigger_refresh
        Callbacks.trigger_stop_robot = self.trigger_stop_robot
        Callbacks.trigger_stop = self.trigger_stop

        self.tf_buffer = tf2_ros.Buffer()
        self.lf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.scene_parameters_path = self.declare_parameter(
            "scene_parameters_path", "default value"
        )
        self.scene_parameters_path = (
            self.get_parameter("scene_parameters_path")
            .get_parameter_value()
            .string_value
        )

        self.scenario = self.declare_parameter("scenario", "default value")
        self.scenario = (
            self.get_parameter("scenario").get_parameter_value().string_value
        )

        self._cancel_goal_client = self.create_client(DashboardCommand, "dashboard_command")
        self.get_logger().warn("Dashboard Control Server not available, wait...")
        self._cancel_goal_client.wait_for_service()
        self.get_logger().info("Dashboard Control Server online.")

        self.get_logger().info("Control GUI node started.")

    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        for i in yaml_file.keys():
            Callbacks.frames.append(i)
        Callbacks.frames.sort()

    def trigger_stop(self):
        request = DashboardCommand.Request()
        request.cmd = "stop"
        future = self._cancel_goal_client.call_async(request)
        self.get_logger().info(f"Request to stop program execution sent.")
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().error(f"Stop program execution call failed with: {(e,)}")
                else:
                    self.get_logger().info(f"Stop program execution call succeeded with: {response}")
                finally:
                    self.get_logger().info(f"Stop program execution call completed.")
                break


class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_move_robot_box())
        self.setLayout(grid)
        self.setWindowTitle("Transformations GUI")
        self.resize(600, 250)

    def make_move_robot_box(self):
        combo_box = QGroupBox("move")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(200)

        combo_box_layout = QGridLayout()

        combo_1_box_label = QLabel("with tcp")
        combo_2_box_label = QLabel("to target")
        combo_3_box_label = QLabel("command")

        combo_1 = QComboBox()
        combo_2 = QComboBox()
        combo_3 = QComboBox()
        combo_3.addItems(["move_j", "move_l", "safe_move_j", "start_vacuum", "stop_vacuum", "pick", "place"])

        line_edit_1 = QLineEdit("0.1")
        line_edit_2 = QLineEdit("0.1")

        line_edit_1_label = QLabel("velocity")
        line_edit_2_label = QLabel("acceleration")

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2_box_button = QPushButton("execute")
        combo_2_box_button.setMaximumWidth(280)
        combo_3_box_button = QPushButton("stop")
        combo_3_box_button.setMaximumWidth(280)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box_layout.addWidget(combo_3_box_label, 2, 0)
        combo_box_layout.addWidget(combo_3, 2, 1)
        combo_box_layout.addWidget(combo_3_box_button, 2, 2)
        combo_box_layout.addWidget(line_edit_1_label, 3, 0)
        combo_box_layout.addWidget(line_edit_1, 3, 1)
        combo_box_layout.addWidget(line_edit_2_label, 4, 0)
        combo_box_layout.addWidget(line_edit_2, 4, 1)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.with_tcp = combo_1.currentText()
            Callbacks.to_target = combo_2.currentText()
            Callbacks.command = combo_3.currentText()
            Callbacks.velocity = line_edit_1.text()
            Callbacks.acceleration = line_edit_2.text()
            Callbacks.trigger_move_robot()
            # self.output.append(Callbacks.information)

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

        def combo_3_box_button_clicked():
            Callbacks.trigger_stop()
            # self.output.append(Callbacks.information)

        combo_3_box_button.clicked.connect(combo_3_box_button_clicked)

        return combo_box


def main(args=None):
    rclpy.init(args=args)
    def launch_node():
        def launch_node_callback_local():
            try:
                n1 = Ros2Node()
                n2 = Ros2ActionNode()

                executor = MultiThreadedExecutor()
                executor.add_node(n1)
                executor.add_node(n2)

                try:
                    executor.spin()
                finally:
                    executor.shutdown()
                    n1.destroy_node()
                    n2.destroy_node()
            finally:
                rclpy.shutdown()


            # try:
            #     rclpy.init(args=args)
            #     node = Ros2Node()
            #     rclpy.spin(node)
            #     node.destroy_node()
            #     rclpy.shutdown()

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