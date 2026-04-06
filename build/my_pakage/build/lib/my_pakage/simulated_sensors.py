# Node per simular un sensor de proximitat, mitjançant valors aleatoris entre 0 i 1 i un segon missatge que tindria el valor contrari
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SimulatedSensor(Node):
    def __init__(self):
        super().__init__('simulated_sensor')  # Nom del node
        # Publisher per el valor principal (0-1)
        self.publisher = self.create_publisher(Float32, 'proximity_topic', 10)
        # Publisher per el valor contrari (1 - valor)
        self.opposite_publisher = self.create_publisher(Float32, 'opposite_proximity_topic', 10)
        timer_period = 0.1  # Cada 0.1 segons
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Genera valor aleatori entre 0 i 1
        value = random.uniform(0, 1)
        opposite_value = 1.0 - value  # Valor contrari
        
        # Missatge principal
        msg = Float32()
        msg.data = value
        self.publisher.publish(msg)
        
        # Missatge contrari
        opposite_msg = Float32()
        opposite_msg.data = opposite_value
        self.opposite_publisher.publish(opposite_msg)
        
        # Logging
        self.get_logger().info(f"Proximity: {value:.3f}, Opposite: {opposite_value:.3f}")

def main(args=None):
    try:
        rclpy.init(args=args)
        simulated_sensor = SimulatedSensor()
        rclpy.spin(simulated_sensor)
    except KeyboardInterrupt:
        print("Exiting node")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()



def main(args=None):
    try:
        rclpy.init(args=args)
        simulated_sensor = SimulatedSensor()
        rclpy.spin(simulated_sensor)

    except KeyboardInterrupt:
        print("exit node")
    except Exception as e:
        print(e)

        
if __name__ == '__main__':
    main()
