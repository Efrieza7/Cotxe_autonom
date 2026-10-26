import math
from math import cos, sin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray


class BicycleLocation(Node):
    """Odometria del cotxe real amb el model bicicleta.

    La posició s'actualitza cada vegada que es publica un LaserScan
    (`scan_topic`), amb el dt entre escanejos. Fins que arriba el primer
    escaneig es publica la posició inicial un cop per segon.

    Entrades:
      - `wheel_speed` (Float32, m/s): velocitat mesurada per `speed_control`.
      - `imu/accel` (Float32, m/s², opcional): acceleració cap endavant de
        `imu_reader`. Només es fa servir per detectar derrapades: normalment
        v = wheel_speed. Si l'acceleració de la IMU (filtrada amb
        `slip_filter_sec`) i la de les rodes (derivada de wheel_speed) es
        diferencien més de `slip_accel_threshold`, les rodes han derrapat
        (patinen en accelerar o es bloquegen en frenar) i v s'integra amb
        l'acceleròmetre: v += a·dt. La derrapada s'acaba quan les
        acceleracions tornen a coincidir i |v − wheel_speed| <
        `slip_exit_speed`, o com a molt després de `slip_max_sec` (perquè
        l'error d'integrar no creixi sense límit). Sense IMU (o si fa més de
        `imu_timeout_sec` que no arriba), v = wheel_speed.
      - `steering_angle` (Float32, rad): angle ordenat al servo per `steering`.
      - `imu/yaw_rate` (Float32, rad/s, opcional): giroscopi de `imu_reader`.
        Si arriba, el gir surt del giroscopi (integrant totes les lectures
        entre dos escanejos), barrejat amb el model v / wheelbase *
        tan(steering) segons `gyro_weight` (1 = només giroscopi). El model
        es queda curt: a les proves el cotxe girava més del doble del que
        calculava. Si fa més de `imu_timeout_sec` que no arriba, el gir es
        calcula només amb el model.
      - `/lidar_node/location_solved` (opcional, `use_lidar_correction`):
        correcció de posició de `lidar_processing`.

    Sortida: `/pose` i `/bicycle_mode/pose` (Float32MultiArray)
    = [x, y, yaw, v, steering] de l'EIX DAVANTER (rodes de gir, on hi ha el
    LIDAR). L'origen (0, 0) és la posició inicial del LIDAR, x cap endavant.
    El model s'integra internament a l'eix posterior (on les rodes no
    llisquen de costat) i es passa a l'eix davanter en publicar.

    Sortida: `slip_detected` (Bool), a cada lectura de `imu/accel`: True
    mentre les rodes derrapen. `speed_control` talla la potència del motor.
    """

    def __init__(self):
        super().__init__('bicycle_location')

        self.distancia_rodes = float(self.declare_parameter('wheelbase', 0.18).value)
        scan_topic = str(self.declare_parameter('scan_topic', '/ldlidar_node/scan').value)
        self.imu_timeout = float(self.declare_parameter('imu_timeout_sec', 0.2).value)
        # Pes del giroscopi en el gir (0 = només model, 1 = només giroscopi).
        self.gyro_weight = float(self.declare_parameter('gyro_weight', 1.0).value)
        # Detecció de derrapada (l'acceleròmetre només es fa servir mentre les rodes derrapen)
        self.slip_accel_threshold = float(
            self.declare_parameter('slip_accel_threshold', 2.0).value
        )  # m/s²
        self.slip_exit_speed = float(self.declare_parameter('slip_exit_speed', 0.1).value)  # m/s
        self.slip_max_sec = float(self.declare_parameter('slip_max_sec', 1.0).value)
        # Filtre pas-baix de la IMU per comparar-la amb wheel_speed, que speed_control ja filtra
        self.slip_filter_sec = float(self.declare_parameter('slip_filter_sec', 0.3).value)
        self.use_lidar_correction = bool(
            self.declare_parameter('use_lidar_correction', False).value
        )
        start_x = float(self.declare_parameter('start_x', 0.0).value)
        start_y = float(self.declare_parameter('start_y', 0.0).value)
        self.direccio_actual = float(self.declare_parameter('start_yaw', 0.0).value)

        if self.distancia_rodes <= 0.0:
            raise ValueError('wheelbase must be > 0')
        if self.slip_accel_threshold <= 0.0:
            raise ValueError('slip_accel_threshold must be > 0')
        if self.slip_filter_sec < 0.0:
            raise ValueError('slip_filter_sec must be >= 0')

        # estat intern: eix posterior
        self.x = start_x - self.distancia_rodes * cos(self.direccio_actual)
        self.y = start_y - self.distancia_rodes * sin(self.direccio_actual)

        self.v_motor = 0.0
        self.v = 0.0  # wheel_speed, o integrada amb l'acceleròmetre durant una derrapada
        self.accel_time = None
        self.wheel_accel = 0.0  # derivada de wheel_speed (m/s²)
        self.wheel_time = None
        self.imu_accel_filtered = 0.0
        self.slipping = False
        self.slip_start = None
        self.direccio_rodes = 0.0
        self.imu_yaw_rate = None
        self.imu_yaw_delta = 0.0  # gir integrat del giroscopi des de l'últim escaneig (rad)
        self.imu_delta_time = 0.0  # temps que cobreix imu_yaw_delta (s)
        self.imu_time = None

        self.create_subscription(Float32, 'wheel_speed', self.speed_callback, 10)
        self.create_subscription(Float32, 'steering_angle', self.steering_callback, 10)
        self.create_subscription(Float32, 'imu/yaw_rate', self.imu_callback, 10)
        self.create_subscription(Float32, 'imu/accel', self.accel_callback, 10)
        self.create_subscription(
            Float32MultiArray, '/lidar_node/location_solved', self.lidar_callback, 10
        )

        self.slip_publisher = self.create_publisher(Bool, 'slip_detected', 10)
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
        if not math.isfinite(msg.data):
            return
        now = self.get_clock().now()
        v_motor = float(msg.data)
        if self.wheel_time is not None:
            dt = (now - self.wheel_time).nanoseconds * 1e-9
            if dt > 0.0:
                self.wheel_accel = (v_motor - self.v_motor) / dt
        self.v_motor = v_motor
        self.wheel_time = now
        if not self._accel_fresh(now):
            self.v = self.v_motor
        elif not self.slipping and not self._check_slip(now):
            self.v = self.v_motor

    def _check_slip(self, now):
        """Comença una derrapada si les acceleracions de la IMU i de les rodes no coincideixen."""
        if abs(self.imu_accel_filtered - self.wheel_accel) <= self.slip_accel_threshold:
            return False
        self.slipping = True
        self.slip_start = now
        self.get_logger().warn(
            f'Derrapada: IMU {self.imu_accel_filtered:.2f} m/s², '
            f'rodes {self.wheel_accel:.2f} m/s²'
        )
        return True

    def accel_callback(self, msg):
        if not math.isfinite(msg.data):
            return
        self._update_slip(float(msg.data), self.get_clock().now())
        self.slip_publisher.publish(Bool(data=self.slipping))

    def _update_slip(self, a, now):
        if not self._accel_fresh(now):
            # primera lectura o la IMU s'havia aturat: es parteix de l'encoder
            self.imu_accel_filtered = a
            self._end_slip(None)
            self.accel_time = now
            return

        dt = (now - self.accel_time).nanoseconds * 1e-9
        self.accel_time = now
        if dt <= 0.0:
            return
        if self.slip_filter_sec > 0.0:
            self.imu_accel_filtered += (a - self.imu_accel_filtered) * dt / (self.slip_filter_sec + dt)
        else:
            self.imu_accel_filtered = a

        if not self.slipping and not self._check_slip(now):
            self.v = self.v_motor
            return
        accel_diff = abs(self.imu_accel_filtered - self.wheel_accel)

        # derrapant: la velocitat surt de l'acceleròmetre
        self.v += a * dt
        slip_time = (now - self.slip_start).nanoseconds * 1e-9
        if slip_time > self.slip_max_sec:
            self._end_slip(f'derrapada massa llarga ({slip_time:.1f} s), es torna a l\'encoder')
        elif (
            accel_diff <= self.slip_accel_threshold
            and abs(self.v - self.v_motor) < self.slip_exit_speed
        ):
            self._end_slip(f'fi de la derrapada ({slip_time:.2f} s)')

    def _end_slip(self, reason):
        if self.slipping and reason:
            self.get_logger().info(reason)
        self.slipping = False
        self.slip_start = None
        self.v = self.v_motor

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
            now = self.get_clock().now()
            if self.imu_time is not None:
                dt = (now - self.imu_time).nanoseconds * 1e-9
                if 0.0 < dt < self.imu_timeout:
                    # trapezi entre aquesta lectura i l'anterior
                    self.imu_yaw_delta += 0.5 * (self.imu_yaw_rate + float(msg.data)) * dt
                    self.imu_delta_time += dt
            self.imu_yaw_rate = float(msg.data)
            self.imu_time = now

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
        model_yaw_rate = (v / self.distancia_rodes) * math.tan(self.direccio_rodes)
        if imu_fresh:
            # gir mitjà del giroscopi des de l'últim escaneig
            gyro_yaw_rate = (
                self.imu_yaw_delta / self.imu_delta_time
                if self.imu_delta_time > 0.0 else self.imu_yaw_rate
            )
            yaw_rate = self.gyro_weight * gyro_yaw_rate + (1.0 - self.gyro_weight) * model_yaw_rate
        else:
            yaw_rate = model_yaw_rate
        self.imu_yaw_delta = 0.0
        self.imu_delta_time = 0.0

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
