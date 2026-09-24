
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import bisect
import math
from collections import deque


class LidarAngleDistancePublisher(Node):

    def __init__(self):
        super().__init__('lidar_angle_distance_publisher')

        # --------------------------------------------------
        # Subscripció al LIDAR
        # --------------------------------------------------

        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',
            self.listener_callback,
            10
        )

        # --------------------------------------------------
        # Subscripció a la localització
        # --------------------------------------------------

        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/pose',
            self.pose_callback,
            10
        )

        # --------------------------------------------------
        # Publicador
        # --------------------------------------------------

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/ldlidar_node/scan_xy',
            10
        )

        # --------------------------------------------------
        # Estat del robot
        # --------------------------------------------------

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.robot_speed = 0.0
        self.robot_steering = 0.0

        self.wheelbase = float(self.declare_parameter('wheelbase', 0.18).value)

        # El LIDAR és sobre l'eix davanter (rodes de gir), que és l'origen de
        # /pose. Gir del zero del LIDAR respecte de l'eix del cotxe
        # (rad, positiu = cap a l'esquerra).
        self.lidar_yaw_offset = float(self.declare_parameter('lidar_yaw_offset', 0.0).value)

        # Rectangle (marc del cotxe, m) on els punts són el propi cotxe i
        # s'ignoren. Per defecte buit (desactivat).
        self.body_min_x = float(self.declare_parameter('body_min_x', 0.0).value)
        self.body_max_x = float(self.declare_parameter('body_max_x', 0.0).value)
        self.body_half_width = float(self.declare_parameter('body_half_width', 0.0).value)

        # Historial (temps_ns, x, y, heading, speed, steering) de /pose per poder
        # fer servir la posició de l'instant exacte de cada raig del LIDAR.
        self.pose_history = deque(maxlen=200)

        # Escaneig rebut que espera la pose calculada per bycicle_mode amb
        # aquest mateix escaneig (bycicle_mode s'actualitza a cada LaserScan).
        self.pending_scan = None

        # Si la pose més propera al final de l'escaneig és més lluny que això
        # (s), l'escaneig es descarta: situar-lo amb una pose vella afegiria
        # cons falsos al mapa.
        self.max_pose_gap = float(self.declare_parameter('max_pose_gap_sec', 0.05).value)

        # Indica si ja hem rebut una localització
        self.pose_received = False

        self.get_logger().info(
            'Cartesian publisher started. '
            'Waiting for /bicycle_mode/pose'
        )

    # ======================================================
    # CALLBACK DE LOCALITZACIÓ
    # ======================================================

    def pose_callback(self, msg: Float32MultiArray):

        if len(msg.data) < 5:
            self.get_logger().warning(
                'Expected [x, y, direccio, v_motor, direccio_rodes] '
                'on /bicycle_mode/pose'
            )
            return

        # Actualitzar estat del robot
        self.robot_x = float(msg.data[0])
        self.robot_y = float(msg.data[1])
        self.robot_heading = float(msg.data[2])
        self.robot_speed = float(msg.data[3])
        self.robot_steering = float(msg.data[4])

        self.pose_history.append((
            self.get_clock().now().nanoseconds,
            self.robot_x,
            self.robot_y,
            self.robot_heading,
            self.robot_speed,
            self.robot_steering,
        ))

        # Ja tenim una localització vàlida
        if not self.pose_received:
            self.pose_received = True

            self.get_logger().info(
                'Localització rebuda. '
                'Lidar processing activat.'
            )

        # La primera pose rebuda després d'un escaneig és la que bycicle_mode
        # ha calculat per a aquest escaneig: ara ja el podem situar.
        if self.pending_scan is not None:
            scan = self.pending_scan
            self.pending_scan = None
            self.process_scan(scan)

    # ======================================================
    # CALCULAR POSICIÓ EN UN INSTANT DETERMINAT
    # ======================================================

    def pose_at_time(self, t_ns: int):
        """Posició de l'eix davanter a l'instant t_ns: la pose rebuda més
        propera en el temps, extrapolada amb el model bicicleta fins a t_ns.

        El model s'integra a l'eix posterior (on no hi ha lliscament lateral) i
        el resultat es torna a passar a l'eix davanter."""

        # pose més propera en el temps (abans o després), extrapolada
        times = [p[0] for p in self.pose_history]
        idx = bisect.bisect_left(times, t_ns)
        if idx >= len(times) or (
            idx > 0 and t_ns - times[idx - 1] < times[idx] - t_ns
        ):
            idx -= 1
        t_pose, x0, y0, heading0, speed, steering = self.pose_history[idx]
        delta_t = (t_ns - t_pose) * 1e-9

        # eix davanter -> eix posterior
        x0 -= self.wheelbase * math.cos(heading0)
        y0 -= self.wheelbase * math.sin(heading0)

        yaw_rate = 0.0

        if abs(self.wheelbase) > 1e-6:
            yaw_rate = (
                speed / self.wheelbase
            ) * math.tan(steering)

        # arc exacte del model bicicleta (mateix càlcul que bycicle_mode)
        heading = heading0 + yaw_rate * delta_t
        if abs(yaw_rate) > 1e-6:
            radius = speed / yaw_rate
            x = x0 + radius * (math.sin(heading) - math.sin(heading0))
            y = y0 - radius * (math.cos(heading) - math.cos(heading0))
        else:
            x = x0 + speed * delta_t * math.cos(heading0)
            y = y0 + speed * delta_t * math.sin(heading0)

        # eix posterior -> eix davanter
        x += self.wheelbase * math.cos(heading)
        y += self.wheelbase * math.sin(heading)

        return x, y, heading

    # ======================================================
    # CALLBACK DEL LIDAR
    # ======================================================

    def listener_callback(self, msg: LaserScan):
        """
        Converteix el LaserScan a coordenades globals
        compensant el moviment del vehicle durant l'escaneig.
        """

        # --------------------------------------------------
        # NO PROCESSAR EL LIDAR FINS TENIR LOCALITZACIÓ
        # --------------------------------------------------
        # NOTA: aquest node fa servir un executor d'un sol fil, per tant
        # un bucle d'espera activa aquí bloquejaria per sempre el callback
        # de /bicycle_mode/pose (mai s'executaria). En comptes d'esperar,
        # simplement descartem aquest escaneig i esperem el següent.

        if not self.pose_received or not self.pose_history:
            return

        # --------------------------------------------------
        # Comprovar que hi ha dades
        # --------------------------------------------------

        if not msg.ranges:
            return

        # Esperar la pose d'aquest escaneig (arriba just després). Si
        # l'anterior encara esperava, es processa ara amb el que hi ha.
        if self.pending_scan is not None:
            self.process_scan(self.pending_scan)
        self.pending_scan = msg

    def process_scan(self, msg: LaserScan):

        # --------------------------------------------------
        # Temps entre punts del LaserScan
        # --------------------------------------------------

        time_increment = float(msg.time_increment)

        if (
            time_increment <= 0.0
            and len(msg.ranges) > 1
            and msg.scan_time > 0.0
        ):
            time_increment = (
                float(msg.scan_time)
                / float(len(msg.ranges) - 1)
            )

        # --------------------------------------------------
        # Processar punts
        # --------------------------------------------------

        last_index = len(msg.ranges) - 1

        # L'stamp del LaserScan correspon a l'últim punt de l'escaneig
        # (tant ldlidar_component com lidar_simulator_node el posen en publicar).
        scan_end_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        if scan_end_ns <= 0:
            scan_end_ns = self.get_clock().now().nanoseconds

        scan_start_ns = scan_end_ns - int(last_index * time_increment * 1e9)
        pose_gap = max(
            min(abs(p[0] - t) for p in self.pose_history)
            for t in (scan_start_ns, scan_end_ns)
        ) * 1e-9
        if pose_gap > self.max_pose_gap:
            self.get_logger().warning(
                f'Escaneig descartat: la pose més propera és a {pose_gap:.3f} s',
                throttle_duration_sec=1.0,
            )
            return

        angle = msg.angle_min

        cartesian_coords = []

        for i, distance in enumerate(msg.ranges):

            # Comprovar distància vàlida
            if (
                not math.isfinite(distance)
                or distance < msg.range_min
                or distance > msg.range_max
            ):
                angle += msg.angle_increment
                continue

            # Temps respecte de l'últim punt de l'escaneig
            delta_t = (
                (i - last_index)
                * time_increment
            )

            # Posició del robot en aquell instant
            pose_x, pose_y, pose_heading = (
                self.pose_at_time(scan_end_ns + int(delta_t * 1e9))
            )

            # Coordenades del punt respecte del cotxe (LIDAR = eix davanter)
            beam_angle = angle + self.lidar_yaw_offset
            local_x = (
                distance
                * math.cos(beam_angle)
            )

            local_y = (
                distance
                * math.sin(beam_angle)
            )

            # Ignorar els punts que són el propi cotxe
            if (
                self.body_min_x <= local_x <= self.body_max_x
                and abs(local_y) <= self.body_half_width
            ):
                angle += msg.angle_increment
                continue

            # Transformació a coordenades globals
            x = (
                pose_x
                + local_x * math.cos(pose_heading)
                - local_y * math.sin(pose_heading)
            )

            y = (
                pose_y
                + local_x * math.sin(pose_heading)
                + local_y * math.cos(pose_heading)
            )

            # Guardar X,Y
            cartesian_coords.append(x)
            cartesian_coords.append(y)

            angle += msg.angle_increment

        # --------------------------------------------------
        # Publicar
        # --------------------------------------------------

        out = Float32MultiArray()

        out.data = cartesian_coords

        self.pub.publish(out)

        self.get_logger().debug(
            f'Published '
            f'{len(cartesian_coords) // 2} pairs '
            f'from current scan'
        )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = LidarAngleDistancePublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

