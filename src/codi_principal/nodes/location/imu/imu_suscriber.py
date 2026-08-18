# Subscripció IMU traslladada a nodes/imu
# ubicació original: EKF_slam/IMU/imu_suscriber.py
import rclpy
import math
from rclpy.node import Node
from my_pakage.msg import IMU_reader, IMU_transformed


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_suscriber')
        self.ap = 0.0
        self.gzx = 0.0
        self.gzy = 0.0
        self.gxy = 0.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0

        self.sensor = mpu6050(0x68)  # Adreça I2C de la IMU
        self.imu_publisher = self.create_publisher(Float32, 'imu_transformed', 10)
        self.timer = self.create_timer(0.1, self.read_imu_data)  # Cada 0.1 segons

    def read_imu_data(self):
        data = self.sensor.get_all_data()
        accel = data[0]  # Acceleròmetre
        gyro = data[1]   # Giroscopi

        # Publica les dades (exemple amb l'acceleració en X)
        msg = Float32()
        msg.data = accel['x']
        self.imu_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
