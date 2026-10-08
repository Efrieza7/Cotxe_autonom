import math
from math import cos, sin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray


class BicycleLocation(Node):
    """Odometria del cotxe real amb el model bicicleta.

    La posició s'actualitza cada vegada que es publica un LaserScan
    (`scan_topic`), amb el dt entre escanejos. Fins que arriba el primer
    escaneig es publica la posició inicial un cop per segon.

    Entrades:
      - `wheel_speed` (Float32, m/s): velocitat mesurada per `speed_control`.
      - `imu/accel` (Float32, m/s², opcional): acceleració cap endavant de
        `imu_reader`. Si arriba, la velocitat es calcula amb un filtre
        complementari: v = α·(v + a·dt) + (1 − α)·wheel_speed, amb
        α = τ / (τ + dt) i τ = `speed_filter_tau`. L'acceleròmetre segueix
        els canvis ràpids de velocitat i l'encoder evita que l'error
        d'integrar l'acceleració s'acumuli. Si l'encoder marca 0 (rodes
        aturades), v = 0, perquè el biaix de l'acceleròmetre no faci avançar
        el cotxe quan està quiet. Sense IMU (o si fa més de `imu_timeout_sec`
        que no arriba), v = wheel_speed.
      - `steering_angle` (Float32, rad): angle ordenat al servo per `steering`.
      - `imu/yaw_rate` (Float32, rad/s, opcional): giroscopi de `imu_reader`.
        Si arriba, el gir és la mitjana del giroscopi i del model
        v / wheelbase * tan(steering); si fa més de `imu_timeout_sec` que no
        arriba, el gir es calcula només amb el model.
      - `/lidar_node/location_solved` (opcional, `use_lidar_correction`):
        correcció de posició de `lidar_processing`.

    Sortida: `/pose` i `/bicycle_mode/pose` (Float32MultiArray)
    = [x, y, yaw, v, steering] de l'EIX DAVANTER (rodes de gir, on hi ha el
    LIDAR). L'origen (0, 0) és la posició inicial del LIDAR, x cap endavant.
    El model s'integra internament a l'eix posterior (on les rodes no
    llisquen de costat) i es passa a l'eix davanter en publicar.
    """

    def __init__(self):
        super().__init__('bicycle_location')

        self.distancia_rodes = float(self.declare_parameter('wheelbase', 0.18).value)
        scan_topic = str(self.declare_parameter('scan_topic', '/ldlidar_node/scan').value)
        self.imu_timeout = float(self.declare_parameter('imu_timeout_sec', 0.2).value)
        # Constant de temps (s) del filtre de velocitat: com més gran, més pes té l'acceleròmetre.
        self.speed_tau = float(self.declare_parameter('speed_filter_tau', 0.2).value)
        self.use_lidar_correction = bool(
            self.declare_parameter('use_lidar_correction', False).value
        )
        start_x = float(self.declare_parameter('start_x', 0.0).value)
        start_y = float(self.declare_parameter('start_y', 0.0).value)
        self.direccio_actual = float(self.declare_parameter('start_yaw', 0.0).value)

        if self.distancia_rodes <= 0.0:
            raise ValueError('wheelbase must be > 0')
        if self.speed_tau < 0.0:
            raise ValueError('speed_filter_tau must be >= 0')

        # estat intern: eix posterior
        self.x = start_x - self.distancia_rodes * cos(self.direccio_actual)
        self.y = start_y - self.distancia_rodes * sin(self.direccio_actual)

        self.v_motor = 0.0
        self.v = 0.0  # velocitat combinada (encoder + acceleròmetre)
        self.accel_time = None
        self.direccio_rodes = 0.0
        self.imu_yaw_rate = None
        self.imu_time = None

        self.create_subscription(Float32, 'wheel_speed', self.speed_callback, 10)
        self.create_subscription(Float32, 'steering_angle', self.steering_callback, 10)
        self.create_subscription(Float32, 'imu/yaw_rate', self.imu_callback, 10)
        self.create_subscription(Float32, 'imu/accel', self.accel_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/lidar_node/location_solved', self.lidar_callback, 10
        )

        self.pose_publisher = self.create_publisher(Float32MultiArray, '/pose', 10)
        self.bicycle_pose_publisher = self.create_publisher(
            Float32MultiArray, '/bicycle_mode/pose', 10
        )

        self.create_subscription(LaserScan, scan_topic, self.laser_callback, 10)

        self.temps_anterior_lidar = None
        # Posició inicial fins al primer escaneig (lidar_image_creator la necessita)
        self.initial_timer = self.create_timer(1.0, self.publish_pose)

        self.get_logger().info(
            f'bicycle_model started: wheelbase={self.distancia_rodes} m, '
            f'updating /pose on every {scan_topic}'
        )

    def speed_callback(self, msg):
        if math.isfinite(msg.data):
            self.v_motor = float(msg.data)
            if not self._accel_fresh(self.get_clock().now()):
                self.v = self.v_motor

    def accel_callback(self, msg):
        if not math.isfinite(msg.data):
            return
        now = self.get_clock().now()
        if not self._accel_fresh(now) or self.v_motor == 0.0:
            # primera lectura, la IMU s'havia aturat o rodes aturades: es parteix de l'encoder
            self.v = self.v_motor
        else:
            dt = (now - self.accel_time).nanoseconds * 1e-9
            if dt > 0.0:
                # filtre complementari: acceleròmetre a curt termini, encoder a llarg termini
                alpha = self.speed_tau / (self.speed_tau + dt)
                self.v = alpha * (self.v + float(msg.data) * dt) + (1.0 - alpha) * self.v_motor
        self.accel_time = now

    def _accel_fresh(self, now):
        return (
            self.accel_time is not None
            and (now - self.accel_time).nanoseconds * 1e-9 < self.imu_timeout
        )

    def steering_callback(self, msg):
        if math.isfinite(msg.data):
            self.direccio_rodes = float(msg.data)

    def imu_callback(self, msg):
        if math.isfinite(msg.data):
            self.imu_yaw_rate = float(msg.data)
            self.imu_time = self.get_clock().now()

    def lidar_callback(self, msg):
        if not self.use_lidar_correction or len(msg.data) < 2:
            return
        x, y = float(msg.data[0]), float(msg.data[1])
        if math.isfinite(x) and math.isfinite(y):
            # la correcció és de l'eix davanter
            self.x = x - self.distancia_rodes * cos(self.direccio_actual)
            self.y = y - self.distancia_rodes * sin(self.direccio_actual)

    def laser_callback(self, msg):
        temps_actual = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if temps_actual <= 0.0:
            temps_actual = self.get_clock().now().nanoseconds * 1e-9

        if self.temps_anterior_lidar is None:
            self.temps_anterior_lidar = temps_actual
            self.initial_timer.cancel()
            self.publish_pose()
            return

        dt = temps_actual - self.temps_anterior_lidar
        self.temps_anterior_lidar = temps_actual
        if dt <= 0.0:
            return

        now = self.get_clock().now()

        imu_fresh = (
            self.imu_time is not None
            and (now - self.imu_time).nanoseconds * 1e-9 < self.imu_timeout
        )
        v = self.v if self._accel_fresh(now) else self.v_motor
        if imu_fresh:
            yaw_rate = (self.imu_yaw_rate + (v / self.distancia_rodes) * math.tan(self.direccio_rodes)) / 2.0
        else:
            yaw_rate = (v / self.distancia_rodes) * math.tan(self.direccio_rodes)

        # Actualitzar la direcció, x i y: arc exacte (dt entre escanejos és gran)
        heading0 = self.direccio_actual
        self.direccio_actual += yaw_rate * dt
        if abs(yaw_rate) > 1e-6:
            radi = v / yaw_rate
            self.x += radi * (sin(self.direccio_actual) - sin(heading0))
            self.y -= radi * (cos(self.direccio_actual) - cos(heading0))
        else:
            self.x += v * dt * cos(heading0)
            self.y += v * dt * sin(heading0)

        self.publish_pose()

    def publish_pose(self):
        # Crear i publicar el missatge (eix davanter)
        pose_msg = Float32MultiArray()
        pose_msg.data = [
            float(self.x + self.distancia_rodes * cos(self.direccio_actual)),
            float(self.y + self.distancia_rodes * sin(self.direccio_actual)),
            float(self.direccio_actual),
            float(self.v),
            float(self.direccio_rodes),
        ]
        self.pose_publisher.publish(pose_msg)
        self.bicycle_pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    bicycle_location = BicycleLocation()
    try:
        rclpy.spin(bicycle_location)
    except KeyboardInterrupt:
        pass
    finally:
        bicycle_location.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
