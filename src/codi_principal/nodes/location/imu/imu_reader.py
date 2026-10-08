# Subscripció IMU traslladada a nodes/imu
# ubicació original: EKF_slam/IMU/imu_reader.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from sense_hat import SenseHat
except ImportError:
    SenseHat = None


class IMUReader(Node):
    """Llegeix el giroscopi i l'acceleròmetre de la IMU de la Sense HAT (LSM9DS1).

    Publica:
      - `imu/yaw_rate` (Float32, rad/s, positiu = gir a l'esquerra), que
        `bicycle_model` fa servir per integrar l'orientació del cotxe.
      - `imu/accel` (Float32, m/s², positiu = accelerant cap endavant), que
        `bicycle_model` combina amb `wheel_speed` per estimar la velocitat.

    En engegar, fa la mitjana de les dues mesures durant `calibration_sec` per
    treure'n el biaix (en el cas de l'acceleròmetre, també la part de la
    gravetat si la placa no està del tot plana): el cotxe ha d'estar QUIET
    aquests segons.
    Si la Sense HAT no està disponible, no publica res i `bicycle_model`
    calcula el gir només amb el model bicicleta.
    """

    GRAVITY = 9.81  # m/s²

    def __init__(self):
        super().__init__('imu_reader')
        self.rate_hz = float(self.declare_parameter('rate_hz', 50.0).value)
        self.calibration_sec = float(self.declare_parameter('calibration_sec', 2.0).value)
        # -1.0 si la Sense HAT està muntada de cap per avall
        self.yaw_sign = float(self.declare_parameter('yaw_sign', 1.0).value)
        # Eix de l'acceleròmetre que apunta cap endavant del cotxe ('x', 'y' o 'z')
        # i el seu signe (-1.0 si apunta cap enrere). Depèn de com estigui muntada la placa.
        self.accel_axis = str(self.declare_parameter('accel_axis', 'x').value)
        self.accel_sign = float(self.declare_parameter('accel_sign', 1.0).value)
        if self.accel_axis not in ('x', 'y', 'z'):
            raise ValueError("accel_axis ha de ser 'x', 'y' o 'z'")

        self.imu_publisher = self.create_publisher(Float32, 'imu/yaw_rate', 10)
        self.accel_publisher = self.create_publisher(Float32, 'imu/accel', 10)

        self.sensor = None
        if SenseHat is None:
            self.get_logger().warning(
                'Llibreria sense_hat no disponible: no es publica imu/yaw_rate'
            )
            return
        try:
            self.sensor = SenseHat()
            self.sensor.set_imu_config(False, True, True)  # giroscopi i acceleròmetre
        except Exception as e:
            self.sensor = None
            self.get_logger().error(f'No es pot inicialitzar la Sense HAT: {e}')
            return

        self.bias = 0.0
        self.accel_bias = 0.0
        self.calibration_samples = []
        self.calibration_target = max(1, int(self.calibration_sec * self.rate_hz))
        self.get_logger().info(
            f'Calibrant la IMU {self.calibration_sec:.1f} s: no moguis el cotxe'
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self.read_imu_data)

    def read_imu_data(self):
        try:
            gyro = self.sensor.get_gyroscope_raw()  # rad/s
            accel = self.sensor.get_accelerometer_raw()  # g
        except Exception as e:
            self.get_logger().error(f'Error llegint la IMU: {e}')
            return
        z = float(gyro['z'])
        a = float(accel[self.accel_axis]) * self.GRAVITY  # m/s²

        if len(self.calibration_samples) < self.calibration_target:
            self.calibration_samples.append((z, a))
            if len(self.calibration_samples) == self.calibration_target:
                n = len(self.calibration_samples)
                self.bias = sum(s[0] for s in self.calibration_samples) / n
                self.accel_bias = sum(s[1] for s in self.calibration_samples) / n
                self.get_logger().info(
                    f'IMU calibrada: biaix giroscopi z = {self.bias:.4f} rad/s, '
                    f'biaix acceleròmetre {self.accel_axis} = {self.accel_bias:.3f} m/s²'
                )
            return

        self.imu_publisher.publish(Float32(data=self.yaw_sign * (z - self.bias)))
        self.accel_publisher.publish(Float32(data=self.accel_sign * (a - self.accel_bias)))


def main(args=None):
    rclpy.init(args=args)
    node = IMUReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
