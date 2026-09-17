import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import cos, sin, tan
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from time import monotonic


class BicycleLocation(Node):
    def __init__(self):
        self.temps_anterior_lidar = None
        super().__init__('bicycle_location')
        # Subscripció al tema 'bicycle_mode/pose'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/lidar_node/location_solved',
            self.listener_callback,
            10
        )

        # Subscripció al tema 'imu_transformed'
        self.imu_subscription = self.create_subscription(
            Float32,
            'imu_transformed',
            self.imu_callback,
            10
        )

        # Subscripció al tema 'motor_speed' (encoder)
        self.encoder_subscription = self.create_subscription(
            Float32,
            'motor_speed',
            self.encoder_callback,
            10
        )
        self.servo_command_subscription = self.create_subscription(
            Float32,
            'servo_command',
            self.servo_command_callback,
            10
        )

        # Publicador per a la posició calculada
        self.pose_publisher = self.create_publisher(Float32MultiArray, 'pose', 10)

        self.pose_publisher = self.create_publisher(Float32MultiArray, 'pose', 10)

        # Inicialitzar variables
        self.x = 0.0
        self.y = 0.0
        self.direccio_actual = 0.0
        self.v_motor = 0.0
        self.direccio_rodes = 0.0
        self.distancia_rodes = 1.0  # Exemple de valor per defecte
        self.dt = 0.1

        # Variables per a les dades de l'IMU i l'encoder
        self.imu_data = None
        self.encoder_data = None

    def imu_callback(self, msg):
        """Callback per rebre dades de l'IMU."""
        self.imu_data = msg.data
        self.update_motor_speed()

    def encoder_callback(self, msg):
        """Callback per rebre dades de l'encoder."""
        self.encoder_data = msg.data
        self.update_motor_speed()

    def servo_command_callback(self, msg):
        """Callback per rebre dades del servo."""
        self.direccio_rodes = msg.data

    def update_motor_speed(self):
        """Calcula la mitjana entre les dades de l'IMU i l'encoder."""
        if self.imu_data is not None and self.encoder_data is not None:
            self.v_motor = (self.imu_data + self.encoder_data) / 2
            self.get_logger().info(f'v_motor actualitzada: {self.v_motor}')
        self.laser_callback(LaserScan())  # Crida al callback del LaserScan amb un missatge buit per actualitzar la posició

    def listener_callback(self, msg):
        try:
            self.x = float(msg.data[0])
            self.y = float(msg.data[1])
            self.direccio_actual = float(msg.data[2])
            self.v_motor = float(msg.data[3])
            self.direccio_rodes = float(msg.data[4])
        except ValueError:
            self.get_logger().error('Received non-numeric data in msg.data')
            return

    def laser_callback(self, msg):
    
        temps_actual = monotonic()

        if self.temps_anterior_lidar is None:
            self.temps_anterior_lidar = temps_actual
            return

        self.dt = temps_actual - self.temps_anterior_lidar
        self.temps_anterior_lidar = temps_actual

    # A partir d'aquí s'executa tot el codi
        # Aquí pots processar el missatge LaserScan
        self.get_logger().info('LaserScan rebut')

        # Exemple: Processar dades del LaserScan (msg.ranges)
        if not msg.ranges:
            self.get_logger().error('LaserScan no conté dades')
            return

        # Lògica del callback existent
        try:
            # Comprovar que distancia_rodes no és zero
            if self.distancia_rodes == 0:
                self.get_logger().error('distancia_rodes cannot be zero')
                return

            # Convertir els valors a float
            self.v_motor = float(msg.data[0])
            self.direccio_rodes = float(msg.data[1])
        except ValueError:
            self.get_logger().error('Received non-numeric data in msg.data')
            return

        # Comprovar que distancia_rodes no és zero
        if self.distancia_rodes == 0:
            self.get_logger().error('distancia_rodes cannot be zero')
            return

        # Actualitzar la direcció, x i y
        self.direccio_actual += (self.v_motor / self.distancia_rodes) * tan(self.direccio_rodes) * self.dt
        self.x += self.v_motor * self.dt * cos(self.direccio_actual)
        self.y += self.v_motor * self.dt * sin(self.direccio_actual)

        # Crear i publicar el missatge
        pose_msg = Float32MultiArray()
        pose_msg.data = [
            self.x,
            self.y,
            self.direccio_actual,
            self.v_motor,
            self.direccio_rodes,
        ]
        self.pose_publisher.publish(pose_msg)  # Assegura't que tens un publisher definit

def main(args=None):
    try:
        rclpy.init(args=args)
        bicycle_location = BicycleLocation()
        rclpy.spin(bicycle_location)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

        
if __name__ == '__main__':
    main()