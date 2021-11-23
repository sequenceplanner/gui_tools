import sys
import json
import rclpy
import tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_tools_msgs.srv import LookupTransform
from sensor_msgs.msg import JointState

import threading
import yaml

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class Callbacks:
    transform = TransformStamped()
    information = ""
    parent = ""
    child = ""
    frames = []
    joints = JointState()

    trigger_refresh = None
    trigger_query = None

class Ros2Node(Node, Callbacks):
    def __init__(self):
        Node.__init__(self, "query_gui")
        Callbacks.__init__(self)

        Callbacks.trigger_query = self.trigger_query
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
        
        self.scenario = self.declare_parameter(
            "scenario", "default value"
        )
        self.scenario = (
            self.get_parameter("scenario")
            .get_parameter_value()
            .string_value
        )

        self.tf_lookup_client = self.create_client(LookupTransform, "tf_lookup")
        self.tf_lookup_request = LookupTransform.Request()
        self.tf_lookup_response = LookupTransform.Response()

        while not self.tf_lookup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("TF Lookup Service not available, waiting again...")

        self.jsub = self.create_subscription(
            JointState,
            '/joint_states',
            self.jsub_callback,
            10)
    
        self.get_logger().info("Query gui up and running.")

    def jsub_callback(self, msg):
        Callbacks.joints = msg
        
    def trigger_refresh(self):
        yaml_file = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
        Callbacks.frames.clear()
        for i in yaml_file.keys():
            Callbacks.frames.append(i)
        Callbacks.frames.sort()

    def trigger_query(self):
        Callbacks.transform = self.tf_lookup(Callbacks.parent, Callbacks.child)

    def tf_lookup(self, parent, child):
        self.tf_lookup_request.parent_id = parent
        self.tf_lookup_request.child_id = child
        self.tf_lookup_request.deadline = 3000
        
        future = self.tf_lookup_client.call_async(self.tf_lookup_request)
        self.get_logger().info("tf lookup request sent: %s" % self.tf_lookup_request)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info(
                        "lookup tf result: %s" % response
                    )
                    return response.transform
                except Exception as e:
                    self.get_logger().info("lookup tf service call failed with: %r" % (e,))
                    response = future.result()
                    return response.transform


class Window(QWidget, Callbacks):

    def __init__(self):
        Callbacks.__init__(self)
        QWidget.__init__(self, None)

        grid = QGridLayout(self)
        grid.addWidget(self.make_query_tf_box())
        grid.addWidget(self.make_information_box())
        self.setLayout(grid)
        self.setWindowTitle("Transformations GUI")
        self.resize(600, 900)

    def make_information_box(self):
        information_box = QGroupBox("information")
        information_box_layout = QGridLayout()
        self.output = QTextBrowser(information_box)
        self.output.setGeometry(QRect(10, 90, 690, 200))
        self.output.setObjectName("information")
        information_button = QPushButton("clear")
        information_button.setMaximumWidth(80)
        information_box_layout.addWidget(self.output, 0, 0)
        information_box_layout.addWidget(
            information_button, 1, 0, alignment=Qt.AlignRight
        )
        information_box.setLayout(information_box_layout)

        def information_button_clicked():
            self.output.clear()

        information_button.clicked.connect(information_button_clicked)

        return information_box

    def make_query_tf_box(self):
        combo_box = QGroupBox("query")
        combo_box.setMinimumWidth(300)
        combo_box.setMaximumHeight(100)

        combo_box_layout = QGridLayout()

        combo_1_box_label = QLabel("parent")
        combo_2_box_label = QLabel("child")

        combo_1 = QComboBox()

        combo_1_box_button = QPushButton("refresh")
        combo_1_box_button.setMaximumWidth(280)
        combo_2 = QComboBox()
        combo_2.setMinimumWidth(400)
        combo_2_box_button = QPushButton("query")
        combo_2_box_button.setMaximumWidth(80)

        combo_box_layout.addWidget(combo_1_box_label, 0, 0)
        combo_box_layout.addWidget(combo_1, 0, 1)
        combo_box_layout.addWidget(combo_1_box_button, 0, 2)
        combo_box_layout.addWidget(combo_2_box_label, 1, 0)
        combo_box_layout.addWidget(combo_2, 1, 1)
        combo_box_layout.addWidget(combo_2_box_button, 1, 2)
        combo_box.setLayout(combo_box_layout)

        def combo_1_box_button_clicked():
            Callbacks.trigger_refresh()
            combo_1.clear()
            combo_1.addItems(Callbacks.frames)
            combo_2.clear()
            combo_2.addItems(Callbacks.frames)

        combo_1_box_button.clicked.connect(combo_1_box_button_clicked)

        def combo_2_box_button_clicked():
            Callbacks.parent = combo_1.currentText()
            Callbacks.child = combo_2.currentText()
            Callbacks.trigger_query()
            self.output.append("{")
            self.output.append("    \"show\": true,")
            self.output.append("    \"active\": false,")
            self.output.append(f"    \"child_frame\": \"{Callbacks.child}\",")
            self.output.append(f"    \"parent_frame\": \"{Callbacks.parent}\",")
            self.output.append("    \"transform\": {")
            self.output.append("        \"translation\": {")
            self.output.append(f"            \"x\": {Callbacks.transform.transform.translation.x},")
            self.output.append(f"            \"y\": {Callbacks.transform.transform.translation.y},")
            self.output.append(f"            \"z\": {Callbacks.transform.transform.translation.z}")
            self.output.append("        },")
            self.output.append("        \"rotation\": {")
            self.output.append(f"            \"x\": {Callbacks.transform.transform.rotation.x},")
            self.output.append(f"            \"y\": {Callbacks.transform.transform.rotation.y},")
            self.output.append(f"            \"z\": {Callbacks.transform.transform.rotation.z},")
            self.output.append(f"            \"w\": {Callbacks.transform.transform.rotation.w}")
            self.output.append("        }")
            self.output.append("    },")
            self.output.append("    \"preferred_joint_configuration\": {")
            self.output.append(f"        \"j0\": {Callbacks.joints.position[0]},")
            self.output.append(f"        \"j1\": {Callbacks.joints.position[1]},")
            self.output.append(f"        \"j2\": {Callbacks.joints.position[2]},")
            self.output.append(f"        \"j3\": {Callbacks.joints.position[3]},")
            self.output.append(f"        \"j4\": {Callbacks.joints.position[4]},")
            self.output.append(f"        \"j5\": {Callbacks.joints.position[5]}")
            self.output.append("    }")
            self.output.append("}")
            # self.output.append(json.dumps(str(Callbacks.joints)))

            # self.output.append(json.dumps(str(Callbacks.transform)))

        combo_2_box_button.clicked.connect(combo_2_box_button_clicked)

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
