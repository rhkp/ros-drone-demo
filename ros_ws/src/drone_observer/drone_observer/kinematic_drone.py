import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class KinematicDrone(Node):
    def __init__(self):
        super().__init__('kinematic_drone')
        self.speed = self.declare_parameter('speed_mps', 4.0).value
        self.pose = [0.0, 0.0, 12.0]
        self.target = list(self.pose)
        self.last_time = self.get_clock().now()
        self.create_subscription(PoseStamped, '/drone/setpoint', self.setpoint, 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.create_timer(0.05, self.tick)

    def setpoint(self, message):
        self.target = [message.pose.position.x, message.pose.position.y, message.pose.position.z]

    def tick(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds / 1e9)
        self.last_time = now
        distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(self.pose, self.target)))
        if distance > 0.001:
            step = min(distance, self.speed * dt)
            self.pose = [a + (b - a) * step / distance for a, b in zip(self.pose, self.target)]
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'farm_map'
        message.child_frame_id = 'drone/base_link'
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.pose
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
