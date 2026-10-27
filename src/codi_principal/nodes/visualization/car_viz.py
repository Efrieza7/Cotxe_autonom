import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CarViz(Node):
    """Passa els topics del cotxe a tipus estàndard per veure'ls a Foxglove / RViz.

    Entrades:
      - `/pose` (Float32MultiArray) = [x, y, yaw, v, steering] de l'eix davanter.
      - `/path_planning/waypoints` (Float32MultiArray) = [x0, y0, x1, y1, ...].
      - `steering_angle` i `target_angle` (Float32, rad): gir aplicat i demanat.

    Sortides (marc `map`):
      - TF `map -> base_link` (base_link = eix davanter, com a car.urdf).
      - `/joint_states`: angle de les rodes davanteres per al model (car.urdf).
      - `/viz/path` (nav_msgs/Path): camí planificat.
      - `/viz/trajectory` (nav_msgs/Path): recorregut del cotxe.
      - `/viz/steering` (MarkerArray): fletxa verda = gir demanat, lila = aplicat.
    """

    def __init__(self):
        super().__init__('car_viz')

        self.map_frame = str(self.declare_parameter('map_frame', 'map').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        # Distància mínima (m) entre punts del recorregut i nombre màxim de punts.
        self.trajectory_step = float(self.declare_parameter('trajectory_step', 0.02).value)
        self.trajectory_max = int(self.declare_parameter('trajectory_max', 3000).value)

        self.steering = 0.0
        self.target = 0.0
        self.trajectory = Path()
        self.trajectory.header.frame_id = self.map_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.path_pub = self.create_publisher(Path, '/viz/path', 10)
        self.trajectory_pub = self.create_publisher(Path, '/viz/trajectory', 10)
        self.steering_pub = self.create_publisher(MarkerArray, '/viz/steering', 10)

        self.create_subscription(Float32MultiArray, '/pose', self.pose_callback, 10)
        self.create_subscription(Float32MultiArray, '/path_planning/waypoints', self.path_callback, 10)
        self.create_subscription(Float32, 'steering_angle', self.steering_callback, 10)
        self.create_subscription(Float32, 'target_angle', self.target_callback, 10)

        # Rodes i fletxes a 20 Hz, encara que /pose només arribi amb cada escaneig.
        self.create_timer(0.05, self.publish_steering)

        self.get_logger().info('car_viz started: TF map -> base_link, /viz/path, /viz/trajectory, /viz/steering')

    def steering_callback(self, msg):
        if math.isfinite(msg.data):
            self.steering = float(msg.data)

    def target_callback(self, msg):
        if math.isfinite(msg.data):
            self.target = float(msg.data)

    def pose_callback(self, msg):
        if len(msg.data) < 3:
            return
        x, y, yaw = (float(v) for v in msg.data[:3])
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(tf)

        poses = self.trajectory.poses
        if not poses or math.hypot(
            x - poses[-1].pose.position.x, y - poses[-1].pose.position.y
        ) >= self.trajectory_step:
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = self.map_frame
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.z, p.pose.orientation.w = qz, qw
            poses.append(p)
            del poses[:-self.trajectory_max]
            self.trajectory.header.stamp = stamp
            self.trajectory_pub.publish(self.trajectory)

    def path_callback(self, msg):
        data = list(msg.data)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        for i in range(0, len(data) - 1, 2):
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(data[i])
            p.pose.position.y = float(data[i + 1])
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)

    def publish_steering(self):
        stamp = self.get_clock().now().to_msg()

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['front_left_steer_joint', 'front_right_steer_joint']
        joints.position = [self.steering, self.steering]
        self.joint_pub.publish(joints)

        markers = MarkerArray()
        for i, (angle, color) in enumerate(
            [(self.target, (0.1, 0.9, 0.1)), (self.steering, (0.8, 0.2, 0.9))]
        ):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.base_frame
            m.ns = 'steering'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.z = 0.12 + 0.01 * i
            _, _, m.pose.orientation.z, m.pose.orientation.w = yaw_to_quaternion(angle)
            m.scale.x, m.scale.y, m.scale.z = 0.3, 0.015, 0.015
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            markers.markers.append(m)
        self.steering_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CarViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
