import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("set_initial_pose")

        self.declare_parameter("namespace", "robot1")
        self.declare_parameter("x", 0.)
        self.declare_parameter("y", 0.)

        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.x = self.get_parameter("x").get_parameter_value().double_value
        self.y = self.get_parameter("y").get_parameter_value().double_value

    def set_initialpose(self):
        navigator = BasicNavigator(namespace=self.namespace)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.x
        initial_pose.pose.position.y = self.y

        navigator.waitUntilNav2Active()
        navigator.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose published")


def main():
    rclpy.init()
    pub = InitialPosePublisher()
    pub.set_initialpose()

