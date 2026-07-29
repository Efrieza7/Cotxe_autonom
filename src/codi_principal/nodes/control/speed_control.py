import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time


# -------------------------
# PID CLASS
# -------------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured

        self.integral += error
        derivative = error - self.prev_error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output


# -------------------------
# NODE
# -------------------------
class MyNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # -------- GPIO CONFIG --------
        self.PWM_PIN = 18
        self.IN1 = 23
        self.IN2 = 24

        self.ENC_A = 17
        self.ENC_B = 27

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)

        GPIO.setup(self.ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pwm = GPIO.PWM(self.PWM_PIN, 1000)
        self.pwm.start(0)

        # -------- ENCODER --------
        self.ticks = 0
        GPIO.add_event_detect(self.ENC_A, GPIO.RISING, callback=self.encoder_callback)

        # -------- CONTROL --------
        self.PULSES_PER_REV = 600
        self.target_rpm = 100

        self.pid = PID(kp=0.8, ki=0.02, kd=0.1)

        self.last_ticks = 0

        # timer ROS2
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Motor node iniciat")

    # -------------------------
    # ENCODER CALLBACK
    # -------------------------
    def encoder_callback(self, channel):
        self.ticks += 1

    # -------------------------
    # MOTOR CONTROL
    # -------------------------
    def set_motor(self, speed):
        speed = max(min(speed, 100), -100)

        if speed >= 0:
            GPIO.output(self.IN1, True)
            GPIO.output(self.IN2, False)
            self.pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.IN1, False)
            GPIO.output(self.IN2, True)
            self.pwm.ChangeDutyCycle(-speed)

    # -------------------------
    # MAIN LOOP (ROS TIMER)
    # -------------------------
    def timer_callback(self):
        delta_ticks = self.ticks - self.last_ticks
        self.last_ticks = self.ticks

        rpm = (delta_ticks / self.PULSES_PER_REV) * (60 / 0.5)

        control = self.pid.compute(self.target_rpm, rpm)

        self.set_motor(control)

        self.get_logger().info(
            f"RPM: {rpm:.2f} | PWM: {control:.2f}"
        )

    # -------------------------
    # CLEANUP
    # -------------------------
    def destroy_node(self):
        self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


# -------------------------
# MAIN
# -------------------------
def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("exit node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()