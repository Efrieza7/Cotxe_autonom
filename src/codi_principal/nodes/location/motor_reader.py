import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial  # Llibreria per comunicar-se amb el port sèrie


class MotorReader(Node):
	"""Llegeix una velocitat (un float per línia) d'un port sèrie i la publica a `motor_speed`.

	La localització fa servir `wheel_speed` de `speed_control` (encoder a la
	Raspberry); aquest node és opcional. Si el port no existeix, avisa i no
	fa res en lloc d'aturar tot el launch.
	"""

	def __init__(self):
		super().__init__('motor_reader')
		port = str(self.declare_parameter('serial_port', '/dev/ttyUSB0').value)
		baudrate = int(self.declare_parameter('baudrate', 9600).value)
		self.publisher = self.create_publisher(Float32, 'motor_speed', 10)
		try:
			self.serial_port = serial.Serial(port, baudrate, timeout=0)  # Configura el port sèrie
		except serial.SerialException as e:
			self.serial_port = None
			self.get_logger().warning(f'No es pot obrir {port}: {e}. motor_reader inactiu.')
			return
		self.buffer = b''
		self.timer = self.create_timer(0.1, self.timer_callback)  # Cada 0.1 segons

	def timer_callback(self):
		try:
			# Llegeix dades del port sèrie sense bloquejar
			self.buffer += self.serial_port.read(self.serial_port.in_waiting or 1)
			*lines, self.buffer = self.buffer.split(b'\n')
			for raw in lines:
				line = raw.decode('utf-8', errors='ignore').strip()
				if not line:
					continue
				motor_speed = float(line)  # Converteix la dada a float
				msg = Float32()
				msg.data = motor_speed
				self.publisher.publish(msg)  # Publica la velocitat del motor
		except ValueError:
			self.get_logger().error('Error converting data to float')
		except Exception as e:
			self.get_logger().error(f'Error reading from serial: {e}')


def main(args=None):
	rclpy.init(args=args)
	node = MotorReader()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info('Shutting down node...')
	finally:
		if node.serial_port is not None:
			node.serial_port.close()  # Tanca el port sèrie
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
