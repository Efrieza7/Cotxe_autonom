# Subscripció IMU traslladada a nodes/imu
# ubicació original: EKF_slam/IMU/imu_reader.py
import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


class LSM9DS1:
    """Giroscopi i acceleròmetre de la Sense HAT (LSM9DS1, I2C 0x6a), llegits
    directament per I2C.

    No es fa servir la llibreria sense_hat: necessita RTIMULib, que a Ubuntu
    no detecta la IMU perquè el kernel ja té ocupat el magnetòmetre (0x1c).
    """

    ADDRESS = 0x6A
    WHO_AM_I = 0x0F  # ha de valer 0x68
    CTRL_REG1_G = 0x10
    CTRL_REG6_XL = 0x20
    OUT_X_G = 0x18
    OUT_X_XL = 0x28
    GYRO_SCALE = math.radians(0.00875)  # 245 dps: 8.75 mdps/LSB -> rad/s
    ACCEL_SCALE = 0.000061  # ±2 g: 0.061 mg/LSB -> g

    def __init__(self, bus=1):
        self.bus = SMBus(bus)
        who = self.bus.read_byte_data(self.ADDRESS, self.WHO_AM_I)
        if who != 0x68:
            raise RuntimeError(f'WHO_AM_I = 0x{who:02x} (esperat 0x68): no és la IMU de la Sense HAT')
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG1_G, 0x60)   # giroscopi 119 Hz, 245 dps
        self.bus.write_byte_data(self.ADDRESS, self.CTRL_REG6_XL, 0x60)  # acceleròmetre 119 Hz, ±2 g

    def _read_xyz(self, register, scale):
        x, y, z = struct.unpack('<hhh', bytes(self.bus.read_i2c_block_data(self.ADDRESS, register, 6)))
        return {'x': x * scale, 'y': y * scale, 'z': z * scale}

    def get_gyroscope_raw(self):
        """rad/s"""
        return self._read_xyz(self.OUT_X_G, self.GYRO_SCALE)

    def get_accelerometer_raw(self):
        """g"""
        return self._read_xyz(self.OUT_X_XL, self.ACCEL_SCALE)


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
        self.calibration_sec = float(self.declare_parameter('calibration_sec', 5.0).value)
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
        if SMBus is None:
            self.get_logger().warning(
                'Llibreria smbus2 no disponible (sudo apt install python3-smbus2): '
                'no es publica imu/yaw_rate'
            )
            return
        try:
            self.sensor = LSM9DS1()
        except Exception as e:
            self.sensor = None
            self.get_logger().error(f'No es pot inicialitzar la IMU de la Sense HAT: {e}')
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
