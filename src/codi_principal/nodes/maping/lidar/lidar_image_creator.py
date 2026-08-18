import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import math


class LidarAngleDistancePublisher(Node):
    def __init__(self):
        super().__init__('lidar_angle_distance_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/bicycle_mode/pose',
            self.pose_callback,
            10,
        )
        self.pub = self.create_publisher(Float32MultiArray, '/ldlidar_node/scan_xy', 10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.robot_speed = 0.0
        self.robot_steering = 0.0
        self.wheelbase = 0.18
        self.get_logger().info('Cartesian publisher started, listening on /ldlidar_node/scan and publishing Cartesian XY on /ldlidar_node/scan_xy')

    def pose_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            self.get_logger().warning('Expected [x, y, direccio, v_motor, direccio_rodes] on /bicycle_mode/pose')
            return

        self.robot_x = float(msg.data[0])
        self.robot_y = float(msg.data[1])
        self.robot_heading = float(msg.data[2])
        self.robot_speed = float(msg.data[3])
        self.robot_steering = float(msg.data[4])

    def pose_at_offset(self, delta_t: float):
        yaw_rate = 0.0
        if abs(self.wheelbase) > 1e-6:
            yaw_rate = (self.robot_speed / self.wheelbase) * math.tan(self.robot_steering)

        heading = self.robot_heading + yaw_rate * delta_t
        x = self.robot_x + self.robot_speed * delta_t * math.cos(heading)
        y = self.robot_y + self.robot_speed * delta_t * math.sin(heading)
        return x, y, heading

    def listener_callback(self, msg: LaserScan):
        """Convert LaserScan to global Cartesian coordinates compensating vehicle motion during the scan."""
        if not msg.ranges:
            return

        time_increment = float(msg.time_increment)
        if time_increment <= 0.0 and len(msg.ranges) > 1 and msg.scan_time > 0.0:
            time_increment = float(msg.scan_time) / float(len(msg.ranges) - 1)

        last_index = len(msg.ranges) - 1
        angle = msg.angle_min
        cartesian_coords = []
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance < msg.range_min or distance > msg.range_max:
                angle += msg.angle_increment
                continue

            delta_t = (i - last_index) * time_increment
            pose_x, pose_y, pose_heading = self.pose_at_offset(delta_t)
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            x = pose_x + local_x * math.cos(pose_heading) - local_y * math.sin(pose_heading)
            y = pose_y + local_x * math.sin(pose_heading) + local_y * math.cos(pose_heading)
            cartesian_coords.append(x)
            cartesian_coords.append(y)
            angle += msg.angle_increment
        

        out = Float32MultiArray()
        out.data = cartesian_coords
        self.pub.publish(out)
        self.get_logger().debug(f'Published {len(cartesian_coords)//2} pairs from current scan')


def main(args=None):
    rclpy.init(args=args)
    node = LidarAngleDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
