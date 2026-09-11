import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class KinematicDrone(Node):
    def __init__(self):
        super().__init__('kinematic_drone')
        self.speed = float(self.declare_parameter('speed_mps', 4.0).value)
        self.acceleration = float(self.declare_parameter('acceleration_mps2', 2.5).value)
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
        self.current_speed = 0.0
        self._pose_request_in_flight = False
        self._last_synced_pose = None
        self._last_pose_warning = self.get_clock().now()
        self.pose_client = self.create_client(SetEntityPose, self.gazebo_pose_service)
        self.last_time = self.get_clock().now()
        self.create_subscription(PoseStamped, '/drone/setpoint', self.setpoint, 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.nav_odom_pub = self.create_publisher(Odometry, '/drone/nav_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.tick)

    def setpoint(self, message):
        self.target = [message.pose.position.x, message.pose.position.y, message.pose.position.z]

    def sync_gazebo_pose(self):
        if self._pose_request_in_flight:
            return
        if not self.pose_client.service_is_ready():
            now = self.get_clock().now()
            if (now - self._last_pose_warning).nanoseconds >= 5e9:
                self.get_logger().warning(
                    f'Gazebo pose service is not ready: {self.gazebo_pose_service}'
                )
                self._last_pose_warning = now
            return
        current_pose = tuple(round(value, 3) for value in self.pose)
        if self._last_synced_pose is not None and math.dist(current_pose, self._last_synced_pose) < 0.02:
            return
        request = SetEntityPose.Request()
        request.entity.name = self.gazebo_entity_name
        request.entity.type = Entity.MODEL
        request.pose.position.x, request.pose.position.y, request.pose.position.z = current_pose
        request.pose.orientation.w = 1.0
        self._pose_request_in_flight = True
        future = self.pose_client.call_async(request)

        def check_result(done):
            try:
                response = done.result()
                if response.success:
                    self._last_synced_pose = current_pose
                else:
                    self.get_logger().warning(
                        f'Gazebo rejected pose update for {self.gazebo_entity_name}'
                    )
            except Exception as error:
                self.get_logger().warning(f'Gazebo pose update failed: {error}')
            finally:
                self._pose_request_in_flight = False

        future.add_done_callback(check_result)

    def tick(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds / 1e9)
        self.last_time = now
        previous_pose = tuple(self.pose)
        distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(self.pose, self.target)))
        if distance > 0.001:
            stopping_speed = math.sqrt(2.0 * self.acceleration * distance)
            desired_speed = min(self.speed, stopping_speed)
            speed_delta = self.acceleration * dt
            if self.current_speed < desired_speed:
                self.current_speed = min(desired_speed, self.current_speed + speed_delta)
            else:
                self.current_speed = max(desired_speed, self.current_speed - speed_delta)
            step = min(distance, self.current_speed * dt)
            self.pose = [a + (b - a) * step / distance for a, b in zip(self.pose, self.target)]
        else:
            self.current_speed = max(0.0, self.current_speed - self.acceleration * dt)
        self.sync_gazebo_pose()
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'farm_map'
        message.child_frame_id = 'drone/base_link'
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = self.pose
        message.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(message)

        nav_odom = Odometry()
        nav_odom.header.stamp = now.to_msg()
        nav_odom.header.frame_id = 'odom'
        nav_odom.child_frame_id = 'base_link'
        nav_odom.pose.pose.position.x = self.pose[0]
        nav_odom.pose.pose.position.y = self.pose[1]
        nav_odom.pose.pose.orientation.w = 1.0
        nav_odom.twist.twist.linear.x = (self.pose[0] - previous_pose[0]) / dt
        nav_odom.twist.twist.linear.y = (self.pose[1] - previous_pose[1]) / dt
        self.nav_odom_pub.publish(nav_odom)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.pose[0]
        transform.transform.translation.y = self.pose[1]
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicDrone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
