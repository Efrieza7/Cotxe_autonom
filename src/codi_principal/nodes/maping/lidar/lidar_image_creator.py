
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

import math


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
            '/bicycle_mode/pose',
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
        self.servo_command_subscription = self.create_subscription(
            Float32,
            'servo_command',
            self.servo_command_callback,
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

        self.wheelbase = 0.18

        # Indica si ja hem rebut una localització
        self.pose_received = False

        self.get_logger().info(
            'Cartesian publisher started. '
            'Waiting for /bicycle_mode/pose'
        )

    def servo_command_callback(self, msg: Float32):
        """Callback per rebre dades del servo."""
        self.robot_steering = msg.data
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

        # Ja tenim una localització vàlida
        if not self.pose_received:
            self.pose_received = True

            self.get_logger().info(
                'Localització rebuda. '
                'Lidar processing activat.'
            )

    # ======================================================
    # CALCULAR POSICIÓ EN UN INSTANT DETERMINAT
    # ======================================================

    def pose_at_offset(self, delta_t: float):

        yaw_rate = 0.0

        if abs(self.wheelbase) > 1e-6:
            yaw_rate = (
                self.robot_speed / self.wheelbase
            ) * math.tan(self.robot_steering)

        heading = (
            self.robot_heading
            + yaw_rate * delta_t
        )

        x = (
            self.robot_x
            + self.robot_speed
            * delta_t
            * math.cos(heading)
        )

        y = (
            self.robot_y
            + self.robot_speed
            * delta_t
            * math.sin(heading)
        )

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

        while not self.pose_received:
            print("",end="")

        # --------------------------------------------------
        # Comprovar que hi ha dades
        # --------------------------------------------------

        if not msg.ranges:
            return

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
                self.pose_at_offset(delta_t)
            )

            # Coordenades del punt respecte del LIDAR
            local_x = (
                distance
                * math.cos(angle)
            )

            local_y = (
                distance
                * math.sin(angle)
            )

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

