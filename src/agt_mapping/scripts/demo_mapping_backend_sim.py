#!/usr/bin/env python3
"""Small operator-facing mapping backend demo.

This node does not replace Gazebo or the real backends. It publishes the same
AGT mapping contract as FAST-LIVO2/RTAB-Map so a laptop operator can practice
terminal/RViz checks before touching the robot.
"""

import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class DemoMappingBackend(Node):
    def __init__(self):
        super().__init__("agt_demo_mapping_backend")
        self.declare_parameter("backend", "fast_livo2")
        self.declare_parameter("odom_topic", "/agt/mapping/odometry")
        self.declare_parameter("cloud_topic", "/agt/mapping/registered_points")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.backend = self.get_parameter("backend").value
        if self.backend not in {"fast_livo2", "rtabmap"}:
            raise RuntimeError("backend must be fast_livo2 or rtabmap")

        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter("odom_topic").value, 20
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.get_parameter("cloud_topic").value, 5
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.on_cmd_vel, 20
        )
        self.tf = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.linear = 0.25
        self.angular = 0.18
        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.step)
        self.get_logger().info(
            "demo mapping backend '%s' publishing AGT mapping contract", self.backend
        )

    def on_cmd_vel(self, msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

    def step(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        drift = 1.0 if self.backend == "fast_livo2" else 0.55
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.yaw += self.angular * drift * dt
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.linear
        odom.twist.twist.angular.z = self.angular * drift
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(self.make_cloud(stamp))

    def make_cloud(self, stamp):
        points = []
        radius = 3.0 if self.backend == "fast_livo2" else 3.6
        for i in range(96):
            angle = i * math.tau / 96.0
            wall = radius + 0.12 * math.sin(4.0 * angle)
            px = self.x + wall * math.cos(angle)
            py = self.y + wall * math.sin(angle)
            pz = 0.15 + 0.04 * math.sin(i * 0.3)
            points.append((px, py, pz, 80.0 + i % 40))

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "odom"
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b"".join(struct.pack("<ffff", *point) for point in points)
        return cloud


def main():
    rclpy.init()
    node = DemoMappingBackend()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
