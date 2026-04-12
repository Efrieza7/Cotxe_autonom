#node de suscrtipcio de un IMU per a trobar la posicio de un cotxe mitjançant Float 32MultiArray, el node es subscriu a un topic on el IMU publica les dades 

import rclpy
from rclpy.node import Node
from my_pakage.msg import IMU


class IMUSuscriber(Node):
    def __init__(self):
        super().__init__('imu_suscriber')
        #falta el publixher i el test
        self.imu_subscriver = self.create_subscription(
            IMU,
            'IMU_data',
            self.listener_callback,
            10
        )
        self.imu_publisher = self.create_publisher(IMU,'IMU_data',10)

    def listener_callback(self, msg):
        self.get_logger().info(f'received: ax={msg.ax}, ay={msg.ay}, vx={msg.vx}, vy={msg.vy}, gz={msg.gz}')
        msg.vx = msg.vx + msg.ax * 0.1
        msg.vy = msg.vy + msg.ay * 0.1
        msg.x = msg.vx * 0.1
        msg.y = msg.vy * 0.1

        self.imu_publisher.publish(msg)
                

def main(args=None):
    try:

        rclpy.init(args=args)
        imu_suscriber = IMUSuscriber()
        rclpy.spin(imu_suscriber)
    except KeyboardInterrupt:
        print("exit node")
    except Exception as e:
        print(e)

        
if __name__ == '__main__':
    main()
