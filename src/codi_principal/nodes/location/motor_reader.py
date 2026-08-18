import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial  # Llibreria per comunicar-se amb el port sèrie

class MotorReader(Node):
	def __init__(self):
		super().__init__('motor_reader')
		self.publisher = self.create_publisher(Float32, 'motor_speed', 10)
		self.timer = self.create_timer(0.1, self.timer_callback)  # Cada 0.1 segons
		self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Configura el port sèrie

	def timer_callback(self):
		try:
			# Llegeix dades del port sèrie
			line = self.serial_port.readline().decode('utf-8').strip()
			if line:
				motor_speed = float(line)  # Converteix la dada a float
				self.get_logger().info(f'Motor speed: {motor_speed}')
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
		node.serial_port.close()  # Tanca el port sèrie
		rclpy.shutdown()

if __name__ == '__main__':
	main()