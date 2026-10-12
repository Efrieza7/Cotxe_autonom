import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    import RPi.GPIO as GPIO
except (ImportError, RuntimeError):
    GPIO = None


# -------------------------
# HARDWARE: DRV8833 + ENCODER
# -------------------------
class DRV8833Motor:
    """Un canal del pont H DRV8833 (xIN1, xIN2).

    El DRV8833 no té pin de PWM separat: la velocitat es controla posant el
    PWM directament a les entrades. Endavant = PWM a IN1 i IN2 a 0; enrere =
    IN1 a 0 i PWM a IN2; aturat = tots dos a 0 (roda lliure).
    """

    def __init__(self, in1, in2, frequency_hz=1000, invert=False):
        if invert:
            in1, in2 = in2, in1
        self.pins = (in1, in2)
        GPIO.setup(in1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(in2, GPIO.OUT, initial=GPIO.LOW)
        self.pwm1 = GPIO.PWM(in1, frequency_hz)
        self.pwm2 = GPIO.PWM(in2, frequency_hz)
        self.pwm1.start(0)
        self.pwm2.start(0)

    def set(self, duty):
        """duty a [-100, 100] (%): signe = sentit de gir."""
        duty = max(-100.0, min(100.0, duty))
        if duty >= 0:
            self.pwm2.ChangeDutyCycle(0)
            self.pwm1.ChangeDutyCycle(duty)
        else:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(-duty)

    def brake(self):
        """Fre actiu: IN1 = IN2 = 1 curtcircuita el motor (fre del DRV8833)."""
        self.pwm1.ChangeDutyCycle(100)
        self.pwm2.ChangeDutyCycle(100)

    def stop(self):
        """Atura el PWM i deixa les dues entrades a 0.

        Sense això, si el procés s'acaba amb el pin a 1, el pin es queda a 1
        i el motor continua girant encara que el programa ja no existeixi.
        """
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        self.pwm1.stop()
        self.pwm2.stop()
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)


class QuadratureEncoder:
    """Encoder de dues fases (A, B).

    A cada flanc de pujada de A es mira B: segons el seu nivell el compte
    suma o resta, així `ticks` té el signe del sentit real de gir. Si en
    avançar el compte baixa, posa `invert=True`.
    """

    def __init__(self, pin_a, pin_b, invert=False):
        self.pins = (pin_a, pin_b)
        self.pin_b = pin_b
        self.step = -1 if invert else 1
        self.ticks = 0
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        try:
            GPIO.add_event_detect(pin_a, GPIO.RISING, callback=self._callback)
        except RuntimeError as e:
            # Error típic de RPi.GPIO amb kernels nous (Ubuntu 24.04, Pi OS
            # Bookworm): "Failed to add edge detection".
            raise RuntimeError(
                f'No es pot llegir l\'encoder al GPIO {pin_a} ({e}). Amb kernels '
                '>= 6.6 RPi.GPIO no detecta flancs: instal·la el substitut '
                'compatible amb `sudo apt install python3-rpi-lgpio`'
            ) from e

    def _callback(self, channel):
        self.ticks += self.step if GPIO.input(self.pin_b) else -self.step


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
    - Llegeix l'encoder en quadratura (fases A i B) i calcula les RPM de la
      roda, amb signe.
    - PID sobre RPM -> cicle de treball del PWM del DRV8833. Els dos motors
      (A amb encoder, B sense) reben la mateixa comanda.
    - Publica `wheel_speed` (Float32, m/s) mesurada, per a `bicycle_model`.
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

        # -------- GPIO CONFIG (BCM) --------
        # DRV8833: motor A (AIN1, AIN2) i motor B (BIN1, BIN2).
        self.AIN1 = int(self.declare_parameter('ain1_pin', 5).value)
        self.AIN2 = int(self.declare_parameter('ain2_pin', 6).value)
        self.BIN1 = int(self.declare_parameter('bin1_pin', 26).value)
        self.BIN2 = int(self.declare_parameter('bin2_pin', 16).value)
        pwm_frequency = float(self.declare_parameter('pwm_frequency_hz', 1000.0).value)
        # Si un motor gira al revés, inverteix-lo aquí en lloc de recablejar.
        invert_a = bool(self.declare_parameter('invert_motor_a', False).value)
        invert_b = bool(self.declare_parameter('invert_motor_b', False).value)

        self.ENC_A = int(self.declare_parameter('enc_a_pin', 27).value)
        self.ENC_B = int(self.declare_parameter('enc_b_pin', 17).value)
        encoder_invert = bool(self.declare_parameter('encoder_invert', False).value)

        self.hardware = not dry_run and GPIO is not None
        if self.hardware:
            GPIO.setmode(GPIO.BCM)
            self.motors = [
                DRV8833Motor(self.AIN1, self.AIN2, pwm_frequency, invert_a),
                DRV8833Motor(self.BIN1, self.BIN2, pwm_frequency, invert_b),
            ]
            self.encoder = QuadratureEncoder(self.ENC_A, self.ENC_B, encoder_invert)
        else:
            self.get_logger().warning(
                'speed_control en mode dry_run (sense RPi.GPIO o dry_run=True): '
                'no es mou el motor, wheel_speed = target_speed'
            )

        # -------- CONTROL --------
        self.target_speed = 0.0
        self.last_command_time = None
        self.pid = PID(kp=kp, ki=ki, kd=kd, integral_limit=100.0 / max(ki, 1e-6))

        self.last_ticks = 0
        self.last_time = None

        self.create_subscription(Float32, 'target_speed', self.target_callback, 10)
        self.speed_pub = self.create_publisher(Float32, 'wheel_speed', 10)
        # Per als LEDs d'estat (status_led): True mentre els motors tenen comanda.
        self.moving_pub = self.create_publisher(Bool, 'car_moving', 10)

        # timer ROS2
        self.timer = self.create_timer(self.period, self.timer_callback)

        self.get_logger().info(
            f"Motor node iniciat: wheel_diameter={self.wheel_diameter} m "
            f"pulses_per_wheel_rev={self.PULSES_PER_REV}"
        )

    # -------------------------
    # CALLBACKS
    # -------------------------
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
        for motor in self.motors:
            motor.set(speed)

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
        self.moving_pub.publish(Bool(data=self.hardware and target != 0.0))

        if not self.hardware:
            self.speed_pub.publish(Float32(data=float(target)))
            return

        ticks = self.encoder.ticks
        delta_ticks = ticks - self.last_ticks
        self.last_ticks = ticks

        # temps real entre crides (el timer pot tenir retard), no el període nominal
        dt = self.period if self.last_time is None else (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            dt = self.period

        # amb signe: l'encoder en quadratura dona el sentit real de gir
        rpm = (delta_ticks / self.PULSES_PER_REV) * (60 / dt)

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
            for motor in self.motors:
                motor.stop()
            # Els pins dels motors es queden com a sortides a 0: si es tornen
            # entrades (cleanup), el DRV8833 pot llegir un 1 i el motor arrenca.
            GPIO.cleanup([self.ENC_A, self.ENC_B])
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
