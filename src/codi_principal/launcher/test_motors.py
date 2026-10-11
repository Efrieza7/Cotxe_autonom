import rclpy
from rclpy.node import Node

from nodes.control.speed_control import GPIO, DRV8833Motor, QuadratureEncoder


class TestMotors(Node):
    """Node de prova dels motors de tracció (DRV8833) i de l'encoder.

    Sense PID: fa girar els motors endavant, els atura, enrere i els torna a
    aturar, i mostra els ticks de l'encoder a cada fase. Serveix per
    comprovar, amb el cotxe aixecat:

    - que els dos motors giren cap endavant a la fase "endavant"
      (si no, `invert_motor_a` / `invert_motor_b`);
    - que els ticks són positius endavant i negatius enrere
      (si és al revés, `encoder_invert`);
    - els polsos per volta de roda (`pulses_per_wheel_rev`): marca la roda,
      fes-la girar a mà una volta i mira quants ticks surten (`duty:=0`).

    Fa servir els mateixos pins per defecte que `speed_control`.
    """

    def __init__(self):
        super().__init__('test_motors')

        ain1 = int(self.declare_parameter('ain1_pin', 25).value)
        ain2 = int(self.declare_parameter('ain2_pin', 8).value)
        bin1 = int(self.declare_parameter('bin1_pin', 24).value)
        bin2 = int(self.declare_parameter('bin2_pin', 23).value)
        enc_a = int(self.declare_parameter('enc_a_pin', 27).value)
        enc_b = int(self.declare_parameter('enc_b_pin', 17).value)
        pwm_frequency = float(self.declare_parameter('pwm_frequency_hz', 1000.0).value)
        invert_a = bool(self.declare_parameter('invert_motor_a', False).value)
        invert_b = bool(self.declare_parameter('invert_motor_b', False).value)
        encoder_invert = bool(self.declare_parameter('encoder_invert', False).value)
        # Cicle de treball (%) de la prova i durada de cada fase.
        duty = float(self.declare_parameter('duty', 30.0).value)
        self.phase_s = float(self.declare_parameter('phase_s', 2.0).value)

        self.phases = [('endavant', duty), ('aturat', 0.0), ('enrere', -duty), ('aturat', 0.0)]
        self.phase = -1
        self.pins = [ain1, ain2, bin1, bin2, enc_a, enc_b]
        self.finished = False

        if GPIO is None:
            self.get_logger().error('RPi.GPIO no disponible: aquesta prova només funciona a la Raspberry')
            self.finished = True
            return

        GPIO.setmode(GPIO.BCM)
        self.motors = [
            DRV8833Motor(ain1, ain2, pwm_frequency, invert_a),
            DRV8833Motor(bin1, bin2, pwm_frequency, invert_b),
        ]
        self.encoder = QuadratureEncoder(enc_a, enc_b, encoder_invert)
        self.last_ticks = 0

        self.get_logger().info(
            f'TestMotors: A=({ain1},{ain2}) B=({bin1},{bin2}) encoder=({enc_a},{enc_b}) '
            f'duty={duty:.0f}% fase={self.phase_s:.1f}s'
        )
        self.next_phase()
        self.timer = self.create_timer(self.phase_s, self.next_phase)

    def next_phase(self):
        if self.phase >= 0:
            ticks = self.encoder.ticks
            name = self.phases[self.phase][0]
            self.get_logger().info(f'  {name}: {ticks - self.last_ticks:+d} ticks (total {ticks:+d})')
            self.last_ticks = ticks

        self.phase += 1
        if self.phase >= len(self.phases):
            self.get_logger().info('Prova acabada')
            self.timer.cancel()
            self.finished = True
            return

        name, duty = self.phases[self.phase]
        self.get_logger().info(f'{name} (duty {duty:+.0f}%)')
        for motor in self.motors:
            motor.set(duty)

    def destroy_node(self):
        if GPIO is not None and hasattr(self, 'motors'):
            for motor in self.motors:
                motor.set(0)
                motor.stop()
            GPIO.cleanup(self.pins)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestMotors()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
