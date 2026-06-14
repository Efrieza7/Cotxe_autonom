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

        self.imu_subscriber = self.create_subscription(
            IMU_reader,
            'imu_readers',
            self.listener_callback,
            10
        )
        self.imu_publisher = self.create_publisher(IMU_transformed, 'imu_transformed', 10)

    def start_position(self, msg):
        """Calcula els angles inicials a partir de les dades de l'IMU"""
        try:
            self.gzx = (math.acos(msg.ax / msg.az)) * 180 / math.pi
        except Exception:
            self.gzx = 0.0
        try:
            self.gzy = (math.acos(msg.ay / msg.az)) * 180 / math.pi
        except Exception:
            self.gzy = 0.0
        try:
            self.gxy = (math.acos(msg.ax / msg.ay)) * 180 / math.pi
        except Exception:
            self.gxy = 0.0

        self.ap = (msg.ax**2 + msg.ay**2 + msg.az**2)**0.5

    def listener_callback(self, msg):
        axp = 0.0
        ayp = 0.0
        azp = 0.0
        """Processa dades de l'IMU i calcula posicio"""
        if self.gzx == 0.0 and self.gzy == 0.0:
            self.start_position(msg)

        # Actualitza els angles segons velocitats angulars
        if self.gxy < 90:
            self.gzx = (msg.vgx*0.1 + self.gzx + msg.vgz*0.1*(self.gzx+self.gzy)/360) % 360
            self.gzy = (msg.vgy*0.1 + self.gzy + msg.vgz*0.1*(self.gzy+self.gzx)/360) % 360
            self.gxy = (msg.vgz*0.1 + self.gxy) % 360
        elif self.gxy > 90 and self.gxy < 270:
            # TODO: gestionar aquest cas correctament
            pass

        # Calcula acceleracio per gravetat
        axp = math.cos(self.gzx*2*math.pi/360)*self.ap
        ayp = math.cos(self.gzy*2*math.pi/360)*self.ap
        azp = math.cos(self.gxy*2*math.pi/360)*self.ap

        # Calcula velocitat i posicio
        self.v = (msg.ax - axp)*0.1 + self.v
        self.x = self.v*math.cos(self.gzx*2*math.pi/360)*0.1 + self.x
        self.y = self.v*math.sin(self.gzx*2*math.pi/360)*0.1 + self.y

        # Publica resultat
        output_msg = IMU_transformed()
        output_msg.ax = msg.ax
        output_msg.ay = msg.ay
        output_msg.az = msg.az
        output_msg.v = self.v
        output_msg.x = self.x
        output_msg.y = self.y
        output_msg.gzx = self.gzx
        output_msg.gzy = self.gzy
        output_msg.gxy = self.gxy

        self.imu_publisher.publish(output_msg)


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
