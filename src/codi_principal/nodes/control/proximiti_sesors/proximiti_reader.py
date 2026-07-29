# Node per simular un sensor de proximitat, mitjançant valors aleatoris entre 0 i 1 i un segon missatge que tindria el valor contrari
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SimulatedSensor(Node):
    def __init__(self):
        super().__init__('simulated_sensor')  # Nom del node
        # Publisher per enviar els dos valors en un mateix missatge
        self.publisher = self.create_publisher(Float32MultiArray, 'proximity_values', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        #TODO: llegir el valor del sensor de proximitat i el seu contrari, i publicar-los en un missatge Float32MultiArray

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

if __name__ == '__main__':
    main()
