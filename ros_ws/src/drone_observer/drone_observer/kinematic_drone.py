import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class KinematicDrone(Node):
    def __init__(self):
        super().__init__('kinematic_drone')
        self.speed = self.declare_parameter('speed_mps', 4.0).value
        self.home_x = self.declare_parameter('home_x', float(os.environ.get('DRONE_HOME_X', '10.0'))).value
        self.home_y = self.declare_parameter('home_y', float(os.environ.get('DRONE_HOME_Y', '16.0'))).value
        self.landing_altitude = float(
            self.declare_parameter('landing_altitude', float(os.environ.get('DRONE_LANDING_ALTITUDE', '0.6'))).value
        )
        self.gazebo_pose_service = self.declare_parameter(
            'gazebo_pose_service', '/world/farm_survey/set_pose').value
        self.gazebo_entity_name = self.declare_parameter(
            'gazebo_entity_name', 'observer_drone').value
        self.pose = [self.home_x, self.home_y, self.landing_altitude]
        self.target = list(self.pose)
        self._requested_target = None
        self.pose_client = self.create_client(SetEntityPose, self.gazebo_pose_service)
        self.last_time = self.get_clock().now()
        self.create_subscription(PoseStamped, '/drone/setpoint', self.setpoint, 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.create_timer(0.05, self.tick)

    def setpoint(self, message):
        self.target = [message.pose.position.x, message.pose.position.y, message.pose.position.z]

    def sync_gazebo_pose(self):
        target = tuple(self.target)
        if target == self._requested_target or not self.pose_client.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name = self.gazebo_entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x, request.pose.position.y, request.pose.position.z = target
        request.pose.orientation.w = 1.0
        future = self.pose_client.call_async(request)
        self._requested_target = target

        def check_result(done):
            try:
                if not done.result().success:
                    self._requested_target = None
            except Exception:
                self._requested_target = None

        future.add_done_callback(check_result)

    def tick(self):
        self.sync_gazebo_pose()
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
