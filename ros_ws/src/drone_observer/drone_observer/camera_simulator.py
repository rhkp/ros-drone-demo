import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .image_utils import synthetic_rgb


class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.width = 320
        self.height = 240
        self.pose = (0.0, 0.0, 12.0)
        self.image_pub = self.create_publisher(Image, '/drone/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/drone/camera/camera_info', 10)
        self.create_subscription(Odometry, '/drone/odom', self.odom, 10)
        self.create_timer(0.2, self.publish_frame)

    def odom(self, message):
        p = message.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = now
        image.header.frame_id = 'drone/camera_link'
        image.height, image.width = self.height, self.width
        image.encoding, image.step = 'rgb8', self.width * 3
        image.data = synthetic_rgb(self.width, self.height)
        self.image_pub.publish(image)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height = self.width, self.height
        info.k = [250.0, 0.0, self.width / 2, 0.0, 250.0, self.height / 2, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
