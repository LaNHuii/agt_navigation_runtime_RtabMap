#!/usr/bin/env python3
"""AGT runtime adapter for the RTAB-Map 3D-LiDAR odometry backend.

Consumes the RTAB-Map backend output and re-publishes it under the same public
AGT mapping contract that the FAST-LIVO2 backend exposes:

  - `/agt/mapping/odometry`
        nav_msgs/Odometry, header.frame_id = odom, child_frame_id = base_footprint
        plus the `odom -> base_footprint` TF (sole continuous-odometry publisher).
  - `/agt/mapping/registered_points`
        sensor_msgs/PointCloud2 of the *current* scan transformed into `odom`,
        preserving the original scan timestamp (same semantics as FAST-LIVO2).

The backend body frame is parameterized (`backend_body_frame`, default
`lidar_link`): the RTAB-Map ICP odometry is expressed in the LiDAR frame, so the
adapter converts the pose/twist into `base_footprint` using the static
`base_footprint -> backend_body_frame` transform (same math as the FAST-LIVO2
adapter).

TF ownership is respected: this node is the *only* `odom -> base_footprint`
publisher; the RTAB-Map nodes are launched with `publish_tf=false` and never
publish any TF.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def quaternion_multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def rotate_vector(vector, quaternion):
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)), inverse
    )
    return rotated[:3]


def sensor_pose_to_base_pose(position, orientation, base_to_sensor):
    norm = math.sqrt(sum(value * value for value in orientation))
    if norm == 0.0:
        raise ValueError("orientation quaternion must not be zero")
    q_odom_sensor = tuple(value / norm for value in orientation)
    translation, q_base_sensor = base_to_sensor
    q_sensor_base = (
        -q_base_sensor[0], -q_base_sensor[1], -q_base_sensor[2], q_base_sensor[3]
    )
    q_odom_base = quaternion_multiply(q_odom_sensor, q_sensor_base)
    sensor_to_base = rotate_vector(tuple(-value for value in translation), q_sensor_base)
    offset = rotate_vector(sensor_to_base, q_odom_sensor)
    return tuple(position[i] + offset[i] for i in range(3)), q_odom_base


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def sensor_twist_to_base_twist(linear, angular, base_to_sensor):
    translation, q_base_sensor = base_to_sensor
    linear_base_at_sensor = rotate_vector(linear, q_base_sensor)
    angular_base = rotate_vector(angular, q_base_sensor)
    lever_velocity = cross(angular_base, translation)
    linear_base = tuple(
        linear_base_at_sensor[i] - lever_velocity[i] for i in range(3)
    )
    return linear_base, angular_base


def identity_transform():
    """In-process identity base->backend transform for equal frames."""
    return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


class RtabmapAdapter(Node):
    def __init__(self):
        super().__init__("agt_mapping_rtabmap_adapter")
        defaults = {
            "input_odometry": "/rtabmap/odom",
            "output_odometry": "/agt/mapping/odometry",
            "input_scan_cloud": "/agt/sensors/lidar/points",
            "output_registered_points": "/agt/mapping/registered_points",
            "registered_points_frame": "odom",
            "odom_frame": "odom",
            "backend_body_frame": "lidar_link",
            "base_frame": "base_footprint",
            "publish_tf": True,
            "transform_timeout": 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.sensor_frame = self.get_parameter("backend_body_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.transform_timeout = self.get_parameter("transform_timeout").value
        self.registered_points_frame = self.get_parameter(
            "registered_points_frame"
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_odometry").value, 10
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_odometry").value,
            self.convert,
            10,
        )
        # registered cloud: reliable, deep queue (compatible with OctoMap
        # reliable consumers and the SensorDataQoS consumer in agt_localization).
        registered_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.points_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("output_registered_points").value,
            registered_qos,
        )
        # scan input uses sensor-data QoS to tolerate best-effort producers.
        self.points_subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_scan_cloud").value,
            self.publish_registered_points,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.warned_missing_tf = False
        self.warned_missing_cloud_tf = False

    def publish_registered_points(self, message):
        if message.header.frame_id == self.registered_points_frame:
            # Already in the target frame: forward unchanged.
            self.points_publisher.publish(message)
            return
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.registered_points_frame,
                    message.header.frame_id,
                    message.header.stamp,
                    timeout=rclpy.duration.Duration(seconds=self.transform_timeout),
                )
            except Exception:
                transform = self.tf_buffer.lookup_transform(
                    self.registered_points_frame,
                    message.header.frame_id,
                    rclpy.time.Time(),
                )
        except Exception as error:
            if not self.warned_missing_cloud_tf:
                self.get_logger().warning(
                    f"Waiting for {self.registered_points_frame} -> "
                    f"{message.header.frame_id}: {error}"
                )
                self.warned_missing_cloud_tf = True
            return
        self.warned_missing_cloud_tf = False
        cloud = do_transform_cloud(message, transform)
        cloud.header.frame_id = self.registered_points_frame
        self.points_publisher.publish(cloud)

    def convert(self, message):
        if self.base_frame == self.sensor_frame:
            base_to_sensor = identity_transform()
        else:
            try:
                static_tf = self.tf_buffer.lookup_transform(
                    self.base_frame, self.sensor_frame, rclpy.time.Time()
                ).transform
            except Exception as error:
                if not self.warned_missing_tf:
                    self.get_logger().warning(
                        f"Waiting for {self.base_frame} -> {self.sensor_frame}: {error}"
                    )
                    self.warned_missing_tf = True
                return
            self.warned_missing_tf = False
            base_to_sensor = (
                (
                    static_tf.translation.x,
                    static_tf.translation.y,
                    static_tf.translation.z,
                ),
                (
                    static_tf.rotation.x,
                    static_tf.rotation.y,
                    static_tf.rotation.z,
                    static_tf.rotation.w,
                ),
            )
        pose = message.pose.pose
        position, orientation = sensor_pose_to_base_pose(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            base_to_sensor,
        )
        twist = message.twist.twist
        linear, angular = sensor_twist_to_base_twist(
            (twist.linear.x, twist.linear.y, twist.linear.z),
            (twist.angular.x, twist.angular.y, twist.angular.z),
            base_to_sensor,
        )
        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose = message.pose
        output.twist = message.twist
        (
            output.pose.pose.position.x,
            output.pose.pose.position.y,
            output.pose.pose.position.z,
        ) = position
        output.pose.pose.orientation.x, output.pose.pose.orientation.y = orientation[:2]
        output.pose.pose.orientation.z, output.pose.pose.orientation.w = orientation[2:]
        output.twist.twist.linear.x, output.twist.twist.linear.y = linear[:2]
        output.twist.twist.linear.z = linear[2]
        output.twist.twist.angular.x, output.twist.twist.angular.y = angular[:2]
        output.twist.twist.angular.z = angular[2]
        self.publisher.publish(output)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
