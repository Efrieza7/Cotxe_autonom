import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import cos, sin, tan


class BicycleLocation(Node):
    def __init__(self):
        super().__init__('bicycle_location')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'my_topic',
            self.listener_callback,
            10
        )
        self.pose_publisher = self.create_publisher(
            Float32MultiArray,
            '/bicycle_mode/pose',
            10,
        )
        self.x = 0.0
        self.y = 0.0
        self.direccio_actual = 0.0
        self.v_motor = 0.0
        self.direccio_rodes = 0.0
        self.distancia_rodes = 0.18
        self.dt = 0.1

    def listener_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning(
                'Expected [v_motor, direccio_rodes] on my_topic'
            )
            return

        self.v_motor = float(msg.data[0])
        self.direccio_rodes = float(msg.data[1])

        self.direccio_actual += (self.v_motor / self.distancia_rodes) * tan(self.direccio_rodes) * self.dt
        self.x += self.v_motor * self.dt * cos(self.direccio_actual)
        self.y += self.v_motor * self.dt * sin(self.direccio_actual)

        pose_msg = Float32MultiArray()
        pose_msg.data = [
            self.x,
            self.y,
            self.direccio_actual,
            self.v_motor,
            self.direccio_rodes,
        ]
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    try:

        rclpy.init(args=args)
        bicycle_location = BicycleLocation()
        rclpy.spin(bicycle_location)
    except KeyboardInterrupt:
        print("exit node")
    except Exception as e:
        print(e)

        
if __name__ == '__main__':
    main()