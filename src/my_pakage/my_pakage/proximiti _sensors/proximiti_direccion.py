import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ProximitiDireccion(Node):
    def __init__(self):
        super().__init__('proximiti_direccion')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'proximity_values',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warning('Missatge incorrecte: calen dos valors')
            return

        proximity = msg.data[0]
        opposite = msg.data[1]

        if proximity > opposite and proximity > 0.8:
            self.get_logger().info('direccio: "Dreta"')
        elif proximity < opposite and proximity < 0.2:
            self.get_logger().info('direccio: "Esquerra"')
        else:
            self.get_logger().info('direccio: "Davant"')


def main(args=None):
    try:
        rclpy.init(args=args)
        proximiti_direccion = ProximitiDireccion()
        rclpy.spin(proximiti_direccion)
    except KeyboardInterrupt:
        print("exit node")
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
