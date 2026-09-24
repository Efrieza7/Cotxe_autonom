"""Kinematic bicycle-model simulator for the car ground-truth pose.

Publishes the same message formats the real stack uses (`/pose` and
`/bicycle_mode/pose`, both `Float32MultiArray` = [x, y, yaw, v, steer]) and
broadcasts the `map -> base_link` TF so the car model shows up in RViz.

Steering and speed are NOT decided here: `codi_principal`'s `path_follower`
node (pure pursuit over `/path_planning/waypoints`) publishes `/target_angle`
and `/target_speed`, which this node just integrates kinematically. The
simulator stays a physics/sensor stand-in only; all driving logic lives in
the main code, same as it would with the real car.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster


class CarSimulatorNode(Node):

    def __init__(self) -> None:
        super().__init__('car_simulator_node')

        self.declare_parameter('wheelbase', 0.18)
        self.declare_parameter('track', 0.13)
        self.declare_parameter('speed_mps', 0.0)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('bicycle_pose_topic', '/bicycle_mode/pose')
        self.declare_parameter('steering_topic', '/target_angle')
        self.declare_parameter('speed_topic', '/target_speed')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.speed = float(self.get_parameter('speed_mps').value)
        update_rate = float(self.get_parameter('update_rate_hz').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # start_x/start_y and the published pose are the FRONT axle, where the
        # LiDAR is mounted (same convention as codi_principal's bycicle_mode).
        # The kinematic model is integrated at the rear axle (self.x, self.y).
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.x = float(self.get_parameter('start_x').value) - self.wheelbase * math.cos(self.yaw)
        self.y = float(self.get_parameter('start_y').value) - self.wheelbase * math.sin(self.yaw)
        self.steer = 0.0

        pose_topic = str(self.get_parameter('pose_topic').value)
        bicycle_pose_topic = str(self.get_parameter('bicycle_pose_topic').value)
        steering_topic = str(self.get_parameter('steering_topic').value)
        speed_topic = str(self.get_parameter('speed_topic').value)

        self.pose_pub = self.create_publisher(Float32MultiArray, pose_topic, 10)
        self.bicycle_pose_pub = self.create_publisher(Float32MultiArray, bicycle_pose_topic, 10)

        self.create_subscription(Float32, steering_topic, self._steering_cb, 10)
        self.create_subscription(Float32, speed_topic, self._speed_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        # front wheel steering angle for robot_state_publisher (urdf/car.urdf)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        period = 1.0 / max(1.0, update_rate)
        self.last_time = self.get_clock().now()
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Car simulator started: wheelbase={self.wheelbase} speed={self.speed} '
            f'steering_topic={steering_topic} speed_topic={speed_topic}'
        )

    def _steering_cb(self, msg: Float32) -> None:
        self.steer = float(msg.data)

    def _speed_cb(self, msg: Float32) -> None:
        self.speed = float(msg.data)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        yaw_rate = (self.speed / self.wheelbase) * math.tan(self.steer)
        self.yaw += yaw_rate * dt
        self.x += self.speed * dt * math.cos(self.yaw)
        self.y += self.speed * dt * math.sin(self.yaw)

        front_x = self.x + self.wheelbase * math.cos(self.yaw)
        front_y = self.y + self.wheelbase * math.sin(self.yaw)

        pose_msg = Float32MultiArray()
        pose_msg.data = [front_x, front_y, self.yaw, self.speed, self.steer]
        self.pose_pub.publish(pose_msg)
        self.bicycle_pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = front_x
        tf_msg.transform.translation.y = front_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)

        joints = JointState()
        joints.header.stamp = tf_msg.header.stamp
        joints.name = ['front_left_steer_joint', 'front_right_steer_joint']
        joints.position = [float(self.steer), float(self.steer)]
        self.joint_pub.publish(joints)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CarSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
