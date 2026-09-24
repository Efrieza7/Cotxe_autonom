# Subscripció IMU traslladada a nodes/imu
# ubicació original: EKF_slam/IMU/imu_suscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from sense_hat import SenseHat
except ImportError:
    SenseHat = None


class IMUSubscriber(Node):
    """Llegeix el giroscopi de la IMU de la Sense HAT (LSM9DS1).

    Publica `imu/yaw_rate` (Float32, rad/s, positiu = gir a l'esquerra), que
    `bycicle_mode` fa servir per integrar l'orientació del cotxe.

    En engegar, fa la mitjana del giroscopi durant `calibration_sec` per
    treure'n el biaix: el cotxe ha d'estar QUIET aquests segons.
    Si la Sense HAT no està disponible, no publica res i `bycicle_mode`
    calcula el gir només amb el model bicicleta.
    """

    def __init__(self):
        super().__init__('imu_suscriber')
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        self.calibration_sec = float(self.declare_parameter('calibration_sec', 2.0).value)
        # -1.0 si la Sense HAT està muntada de cap per avall
        self.yaw_sign = float(self.declare_parameter('yaw_sign', 1.0).value)

        self.imu_publisher = self.create_publisher(Float32, 'imu/yaw_rate', 10)

        self.sensor = None
        if SenseHat is None:
            self.get_logger().warning(
                'Llibreria sense_hat no disponible: no es publica imu/yaw_rate'
            )
            return
        try:
            self.sensor = SenseHat()
            self.sensor.set_imu_config(False, True, False)  # només giroscopi
        except Exception as e:
            self.sensor = None
            self.get_logger().error(f'No es pot inicialitzar la Sense HAT: {e}')
            return

        self.bias = 0.0
        self.calibration_samples = []
        self.calibration_target = max(1, int(self.calibration_sec * self.rate_hz))
        self.get_logger().info(
            f'Calibrant el giroscopi {self.calibration_sec:.1f} s: no moguis el cotxe'
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self.read_imu_data)

    def read_imu_data(self):
        try:
            gyro = self.sensor.get_gyroscope_raw()  # rad/s
        except Exception as e:
            self.get_logger().error(f'Error llegint la IMU: {e}')
            return
        z = float(gyro['z'])

        if len(self.calibration_samples) < self.calibration_target:
            self.calibration_samples.append(z)
            if len(self.calibration_samples) == self.calibration_target:
                self.bias = sum(self.calibration_samples) / len(self.calibration_samples)
                self.get_logger().info(f'Giroscopi calibrat: biaix z = {self.bias:.4f} rad/s')
            return

        msg = Float32()
        msg.data = self.yaw_sign * (z - self.bias)
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
