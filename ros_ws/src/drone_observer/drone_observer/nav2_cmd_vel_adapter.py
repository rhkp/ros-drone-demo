import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class Nav2CmdVelAdapter(Node):
    """Translate Nav2 planar velocity commands into smooth drone setpoints."""

    def __init__(self):
        super().__init__('nav2_cmd_vel_adapter')
        self.active = False
        self.pose = None
        self.altitude = 0.6
        self.target = None
        self.command = (0.0, 0.0)
        self.last_time = self.get_clock().now()
        self.create_subscription(Bool, '/drone/nav2_active', self.on_active, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/drone/nav_odom', self.on_nav_odom, 10)
        self.create_subscription(Odometry, '/drone/odom', self.on_drone_odom, 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/drone/setpoint', 10)
        self.create_timer(0.05, self.tick)

    def on_active(self, message):
        self.active = bool(message.data)
        if self.active and self.pose is not None:
            self.target = [self.pose[0], self.pose[1]]
            self.command = (0.0, 0.0)

    def on_cmd_vel(self, message):
        self.command = (float(message.linear.x), float(message.linear.y))

    def on_nav_odom(self, message):
        self.pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if not self.active:
            self.target = [self.pose[0], self.pose[1]]

    def on_drone_odom(self, message):
        self.altitude = float(message.pose.pose.position.z)

    def tick(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds / 1e9)
        self.last_time = now
        if not self.active or self.target is None:
            return
        self.target[0] += self.command[0] * dt
        self.target[1] += self.command[1] * dt
        message = PoseStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'farm_map'
        message.pose.position.x = self.target[0]
        message.pose.position.y = self.target[1]
        message.pose.position.z = self.altitude
        message.pose.orientation.w = 1.0
        self.setpoint_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
