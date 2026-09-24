import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None


# -------------------------
# PID CLASS
# -------------------------
class PID:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self.prev_error = 0
        self.integral = 0

    def reset(self):
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured):
        error = setpoint - measured

        # anti-windup: l'integral no pot créixer sense límit
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral + error))
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
    """Control de velocitat del motor de tracció.

    - Subscriu `target_speed` (Float32, m/s), publicat per `path_follower`.
    - Llegeix l'encoder (canal A) i calcula les RPM de la roda.
    - PID sobre RPM -> cicle de treball del PWM del pont H.
    - Publica `wheel_speed` (Float32, m/s) mesurada, per a `bycicle_mode`.
    - Seguretat: si no arriba cap `target_speed` en `command_timeout_sec`,
      atura el motor.

    Si RPi.GPIO no està disponible o `dry_run` és True, no toca el motor i
    publica com a `wheel_speed` la velocitat ordenada (útil per provar-ho
    fora del cotxe).
    """

    def __init__(self):
        super().__init__('motor_node')

        # -------- PARAMETRES --------
        # Diàmetre de la roda motriu (m): MESURA'L i posa'l al launch.
        self.wheel_diameter = float(self.declare_parameter('wheel_diameter', 0.065).value)
        # Polsos del canal A de l'encoder per cada volta de la RODA
        # (si l'encoder és al motor, multiplica per la reducció).
        self.PULSES_PER_REV = float(self.declare_parameter('pulses_per_wheel_rev', 600.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 1.0).value)  # m/s
        self.command_timeout = float(self.declare_parameter('command_timeout_sec', 0.5).value)
        self.period = float(self.declare_parameter('control_period_sec', 0.1).value)
        kp = float(self.declare_parameter('kp', 0.8).value)
        ki = float(self.declare_parameter('ki', 0.02).value)
        kd = float(self.declare_parameter('kd', 0.1).value)
        dry_run = bool(self.declare_parameter('dry_run', False).value)

        # -------- GPIO CONFIG --------
        self.PWM_PIN = int(self.declare_parameter('pwm_pin', 18).value)
        self.IN1 = int(self.declare_parameter('in1_pin', 23).value)
        self.IN2 = int(self.declare_parameter('in2_pin', 24).value)

        self.ENC_A = int(self.declare_parameter('enc_a_pin', 17).value)
        self.ENC_B = int(self.declare_parameter('enc_b_pin', 27).value)

        self.hardware = not dry_run and GPIO is not None
        if self.hardware:
            GPIO.setmode(GPIO.BCM)

            GPIO.setup(self.PWM_PIN, GPIO.OUT)
            GPIO.setup(self.IN1, GPIO.OUT)
            GPIO.setup(self.IN2, GPIO.OUT)

            GPIO.setup(self.ENC_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.ENC_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            self.pwm = GPIO.PWM(self.PWM_PIN, 1000)
            self.pwm.start(0)
        else:
            self.get_logger().warning(
                'speed_control en mode dry_run (sense RPi.GPIO o dry_run=True): '
                'no es mou el motor, wheel_speed = target_speed'
            )

        # -------- ENCODER --------
        self.ticks = 0
        if self.hardware:
            GPIO.add_event_detect(self.ENC_A, GPIO.RISING, callback=self.encoder_callback)

        # -------- CONTROL --------
        self.target_speed = 0.0
        self.last_command_time = None
        self.pid = PID(kp=kp, ki=ki, kd=kd, integral_limit=100.0 / max(ki, 1e-6))

        self.last_ticks = 0

        self.create_subscription(Float32, 'target_speed', self.target_callback, 10)
        self.speed_pub = self.create_publisher(Float32, 'wheel_speed', 10)

        # timer ROS2
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f"Motor node iniciat: wheel_diameter={self.wheel_diameter} m "
            f"pulses_per_wheel_rev={self.PULSES_PER_REV}"
        )

    # -------------------------
    # CALLBACKS
    # -------------------------
    def encoder_callback(self, channel):
        self.ticks += 1

    def target_callback(self, msg: Float32):
        if math.isfinite(msg.data):
            self.target_speed = max(-self.max_speed, min(self.max_speed, float(msg.data)))
            self.last_command_time = self.get_clock().now()

    # -------------------------
    # MOTOR CONTROL
    # -------------------------
    def set_motor(self, speed):
        if not self.hardware:
            return
        speed = max(min(speed, 100), -100)

        if speed >= 0:
            GPIO.output(self.IN1, True)
            GPIO.output(self.IN2, False)
            self.pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(self.IN1, False)
            GPIO.output(self.IN2, True)
            self.pwm.ChangeDutyCycle(-speed)

    def rpm_to_speed(self, rpm):
        return rpm / 60.0 * math.pi * self.wheel_diameter

    def speed_to_rpm(self, speed):
        return speed * 60.0 / (math.pi * self.wheel_diameter)

    # -------------------------
    # MAIN LOOP (ROS TIMER)
    # -------------------------
    def timer_callback(self):
        now = self.get_clock().now()
        timed_out = (
            self.last_command_time is None
            or (now - self.last_command_time).nanoseconds * 1e-9 > self.command_timeout
        )
        target = 0.0 if timed_out else self.target_speed

        if not self.hardware:
            self.speed_pub.publish(Float32(data=float(target)))
            return

        delta_ticks = self.ticks - self.last_ticks
        self.last_ticks = self.ticks

        # l'encoder només té un canal llegit: el sentit és el de la comanda
        rpm = (delta_ticks / self.PULSES_PER_REV) * (60 / self.period)
        if target < 0:
            rpm = -rpm

        if target == 0.0:
            self.pid.reset()
            control = 0.0
        else:
            control = self.pid.compute(self.speed_to_rpm(target), rpm)

        self.set_motor(control)
        self.speed_pub.publish(Float32(data=float(self.rpm_to_speed(rpm))))

        self.get_logger().debug(
            f"target: {target:.2f} m/s | RPM: {rpm:.2f} | PWM: {control:.2f}"
        )

    # -------------------------
    # CLEANUP
    # -------------------------
    def destroy_node(self):
        if self.hardware:
            self.pwm.stop()
            GPIO.cleanup([self.PWM_PIN, self.IN1, self.IN2, self.ENC_A, self.ENC_B])
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
