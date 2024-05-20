import sys
import rclpy
from rclpy.client import Client
from rclpy.service import Service
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from ur_controller_msgs.action import URControl
import threading
import yaml
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from ur_script_msgs.srv import DashboardCommand
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

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
    trigger_ghost = None
    ghost_joints = None

class Ros2ActionNode(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "control_gui_action")
        Callbacks.__init__(self)

        Callbacks.trigger_move_robot = self.trigger_move_robot
        Callbacks.trigger_ghost = self.trigger_ghost
        Callbacks.trigger_stop = self.trigger_stop

        self._ur_control_client = ActionClient(self, URControl, "ur_control")
        self.get_logger().warn("UR Script Controller Server not available, wait...")
        self._ur_control_client.wait_for_server()
        self.get_logger().info("UR Script Controller Server online.")


    def trigger_move_robot(self):
        request = URControl.Goal()
        request.command = Callbacks.command
        request.acceleration = float(Callbacks.acceleration)
        request.velocity = float(Callbacks.velocity)
        request.tcp_id = Callbacks.with_tcp
        request.goal_feature_id = Callbacks.to_target
        self.generate_and_send_ur_script(request)

    def trigger_ghost(self):
        if Callbacks.ghost_joints is None:
            return

        request = URControl.Goal()
        request.command = Callbacks.command
        request.acceleration = float(Callbacks.acceleration)
        request.velocity = float(Callbacks.velocity)
        request.use_joint_positions = True
        request.joint_positions = JointState()
        request.joint_positions.position = Callbacks.ghost_joints

        self.generate_and_send_ur_script(request)

    def generate_and_send_ur_script(self, request):
        Callbacks.status.setText("Making request.")
        Callbacks.feedback.setText("")
        self.goal_handle = None
        self._send_goal_future = self._ur_control_client.send_goal_async(request,
                                                                         feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            Callbacks.status.setText("Goal rejected")
            return

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        Callbacks.status.setText("Goal accepted")

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            Callbacks.status.setText("Completed successfully")
        else:
            Callbacks.status.setText("Failed.")

        self.goal_handle = None

    def feedback_callback(self, feedback_msg):
        Callbacks.feedback.setText(feedback_msg.feedback.current_state)

    def trigger_stop(self):
        if not self.goal_handle is None:
            self.get_logger().warn("Cancelling goal.")
            Callbacks.status.setText("Cancelling.")
            self._cancel_goal_future = self.goal_handle.cancel_goal_async()
            self._cancel_goal_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        result = future.result().result
        self.get_logger().info('Cancel request done: {0}'.format(result))


class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "control_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_refresh = self.trigger_refresh

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

        self.ghost_sub = self.create_subscription(
            JointState,
            '/ghost/joint_states',
            self.ghost_sub_callback,
            10)

        self.get_logger().info("Query gui up and running.")

        self._cancel_goal_client = self.create_client(DashboardCommand, "dashboard_command")
        self.get_logger().warn("Dashboard Control Server not available, wait...")
        # self._cancel_goal_client.wait_for_service()
        self.get_logger().info("Dashboard Control Server online.")

        self.get_logger().warn("Control GUI node started.")

    def ghost_sub_callback(self, msg):
        Callbacks.ghost_joints = msg.position

    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        for i in yaml_file.keys():
            Callbacks.frames.append(i)
        Callbacks.frames.sort()


class Window(QWidget, Callbacks):
    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_move_robot_box())
        self.setLayout(grid)
        self.setWindowTitle("Robot control")
        self.resize(600, 280)

    def make_move_robot_box(self):
        combo_box = QGroupBox("move")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(250)

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
        combo_4_box_button = QPushButton("match ghost")
        combo_4_box_button.setMaximumWidth(280)


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
        combo_box_layout.addWidget(combo_4_box_button, 3, 2)
        combo_box_layout.addWidget(line_edit_2_label, 4, 0)
        combo_box_layout.addWidget(line_edit_2, 4, 1)

        status = QLabel("Status:")
        feedback = QLabel("Feedback:")
        Callbacks.status = QLabel("[no goal]")
        Callbacks.feedback = QLabel("[no goal]")
        combo_box_layout.addWidget(status, 5, 0)
        combo_box_layout.addWidget(Callbacks.status, 5, 1)
        combo_box_layout.addWidget(feedback, 6, 0)
        combo_box_layout.addWidget(Callbacks.feedback, 6, 1)

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

        def combo_4_box_button_clicked():
            Callbacks.command = combo_3.currentText()
            Callbacks.velocity = line_edit_1.text()
            Callbacks.acceleration = line_edit_2.text()
            Callbacks.trigger_ghost()
            # self.output.append(Callbacks.information)

        combo_4_box_button.clicked.connect(combo_4_box_button_clicked)

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
