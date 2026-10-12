#!/usr/bin/env python3
"""Mostra l'estat del cotxe a la matriu de LEDs de la Sense HAT.

- apagat:               la Raspberry encara arrenca (o el servei està aturat)
- verd fix:             preparada, no s'executa cap launch
- groc fix:             s'executa un launch, però el cotxe està aturat
- vermell parpellejant: el cotxe es mou (`speed_control` o `test_motors`
                        publiquen `car_moving` = True)

S'engega sol en arrencar la Raspberry amb el servei
`raspberry/status-led.service` (sudo ./raspberry/install_status_led.sh).
"""
import glob
import os
import struct
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

OFF = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 160, 0)
RED = (255, 0, 0)


class SenseHatMatrix:
    """Matriu 8x8 de la Sense HAT, escrivint directament al seu framebuffer.

    Així no cal la llibreria sense_hat, que a més obre la IMU i el joystick
    (els fa servir `imu_reader` i `start_car`).
    """

    def __init__(self):
        self.device = None
        for name_file in glob.glob('/sys/class/graphics/fb*/name'):
            with open(name_file) as f:
                if f.read().strip() == 'RPi-Sense FB':
                    self.device = '/dev/' + os.path.basename(os.path.dirname(name_file))
        if self.device is None:
            raise RuntimeError(
                "no es troba el framebuffer de la Sense HAT ('RPi-Sense FB'). "
                'Comprova que la placa estigui ben posada: cat /sys/class/graphics/fb*/name'
            )
        self.fb = open(self.device, 'wb')

    def fill(self, color):
        """Posa els 64 LEDs del mateix color (r, g, b de 0 a 255)."""
        r, g, b = color
        pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)  # RGB565
        self.fb.seek(0)
        self.fb.write(struct.pack('<64H', *([pixel] * 64)))
        self.fb.flush()

    def close(self):
        self.fb.close()


class StatusLed(Node):
    def __init__(self, matrix):
        super().__init__('status_led')
        self.matrix = matrix
        # Si en aquest temps no arriba cap car_moving = True, es considera aturat
        # (p. ex. si el node que movia el cotxe ha petat).
        self.moving_timeout = float(self.declare_parameter('moving_timeout_sec', 1.0).value)
        self.blink_hz = float(self.declare_parameter('blink_hz', 2.0).value)
        self.last_moving = None  # instant de l'últim car_moving = True

        self.create_subscription(Bool, 'car_moving', self.moving_callback, 10)
        # Es redibuixa sempre: si un altre programa esborra la matriu, es recupera.
        self.create_timer(0.1, self.update)
        self.get_logger().info(f'LEDs d\'estat a {matrix.device}')

    def moving_callback(self, msg):
        self.last_moving = time.monotonic() if msg.data else None

    def launch_running(self):
        """Hi ha algun altre node ROS en marxa (un launch o un ros2 run)?

        Els nodes ocults (_ros2cli_*, que crea l'eina `ros2`) no compten.
        """
        return any(
            name != self.get_name() and not name.startswith('_')
            for name in self.get_node_names()
        )

    def update(self):
        now = time.monotonic()
        if self.last_moving is not None and now - self.last_moving < self.moving_timeout:
            color = RED if int(now * self.blink_hz * 2) % 2 == 0 else OFF
        elif self.launch_running():
            color = YELLOW
        else:
            color = GREEN
        self.matrix.fill(color)


def main(args=None):
    try:
        matrix = SenseHatMatrix()
    except Exception as e:
        print(f'status_led: {e}', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = StatusLed(matrix)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        matrix.fill(OFF)
        matrix.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
