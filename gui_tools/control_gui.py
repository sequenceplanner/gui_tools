import sys
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from ur_script_generator_msgs.srv import GenerateURScript
import threading
import yaml
from rclpy.action import ActionClient
from ur_script_msgs.action import ExecuteScript

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


class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "control_gui")
        Callbacks.__init__(self)

        self.action_future = None

        Callbacks.trigger_refresh = self.trigger_refresh
        Callbacks.trigger_move_robot = self.trigger_move_robot
        Callbacks.trigger_stop_robot = self.trigger_stop_robot
        Callbacks.trigger_lock_scene = self.trigger_lock_scene

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

        self.ursg_client = self.create_client(GenerateURScript, "ur_script_generator")
        self.ursg_request = GenerateURScript.Request()

        while not self.ursg_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("UR Script Generator Service not available, waiting again...")

    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        for i in yaml_file.keys():
            Callbacks.frames.append(i)
        Callbacks.frames.sort()

    def trigger_move_robot(self):
        self.ursg_request.command = Callbacks.command
        self.ursg_request.acceleration = float(Callbacks.acceleration)
        self.ursg_request.velocity = float(Callbacks.velocity)
        self.ursg_request.tcp_name = Callbacks.with_tcp
        self.ursg_request.goal_feature_name = Callbacks.to_target
        self.generate_and_send_ur_script(self.ursg_request)

    def generate_and_send_ur_script(self, request):
        ursg_future = self.ursg_client.call_async(request)
        while 1:
            if ursg_future.done():
                try:
                    response = ursg_future.result()
                except Exception as e:
                    self.get_logger().error(
                        "UR Script Generator service call failed %r" % (e,)
                    )
                else:
                    ursg_response = response.script
                    self.get_logger().info(
                        "Result of generate_ur_script: %s" % ursg_response
                    )
                    if response.success:
                        self.action_client = ActionClient(self, ExecuteScript, "ur_script")

                        def send_goal(script):
                            goal_msg = ExecuteScript.Goal()
                            goal_msg.script = script
                            self.action_client.wait_for_server()
                            self.action_future = self.action_client.send_goal_async(
                                goal_msg
                            )
                        send_goal(ursg_response)
                    else:
                        self.get_logger().error(
                        "Gui did not send a move command command, UR script did not generate."
                    )
                break

    def trigger_lock_scene(self):
        self.lock_client.call_async(self.lock_request)

    def trigger_stop_robot(self):
        if self.action_future != None:
            self.get_logger().info("future is not none")
            if self.action_future.done():
                self.get_logger().info("future is done")
                self.action_future.result().cancel_goal()
            else:
                pass
        else:
            pass


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
        combo_3.addItems(["move_j", "move_l"])

        line_edit_1 = QLineEdit("0.1")
        line_edit_2 = QLineEdit("0.1")

        line_edit_1_label = QLabel("velocity")
        line_edit_2_label = QLabel("acceleration")

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2_box_button = QPushButton("move")
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
            Callbacks.trigger_stop_robot()
            self.output.append(Callbacks.information)

        combo_3_box_button.clicked.connect(combo_3_box_button_clicked)

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
