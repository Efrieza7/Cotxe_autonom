"""Prova tipus missió d'inspecció (FS Driverless): motors + direcció, < 10 s.

    ros2 launch my_pakage test_motors.launch.py
    ros2 launch my_pakage test_motors.launch.py duty:=40.0

COTXE AIXECAT: les rodes giren. No es pot executar alhora que full_car
(els dos fan servir els mateixos pins).
"""
import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Els pins i la configuració del cotxe es llegeixen de full_car.launch.py, així
# només s'han de canviar en un lloc.
_spec = importlib.util.spec_from_file_location(
    'full_car_config', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'full_car.launch.py')
)
car = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(car)


def generate_launch_description():
    duty = ParameterValue(LaunchConfiguration('duty'), value_type=float)

    test_motors = Node(
        package='my_pakage', executable='test_motors', name='test_motors', output='screen',
        parameters=[{
            'ain1_pin': car.MOTOR_AIN1_PIN,
            'ain2_pin': car.MOTOR_AIN2_PIN,
            'bin1_pin': car.MOTOR_BIN1_PIN,
            'bin2_pin': car.MOTOR_BIN2_PIN,
            'enc_a_pin': car.ENCODER_A_PIN,
            'enc_b_pin': car.ENCODER_B_PIN,
            'invert_motor_a': car.INVERT_MOTOR_A,
            'invert_motor_b': car.INVERT_MOTOR_B,
            'encoder_invert': car.ENCODER_INVERT,
            'servo_pin': car.SERVO_PIN,
            'min_pulse_us': car.SERVO_MIN_PULSE_US,
            'center_pulse_us': car.SERVO_CENTER_PULSE_US,
            'max_pulse_us': car.SERVO_MAX_PULSE_US,
            'servo_invert': car.SERVO_INVERT,
            'wheel_diameter': car.WHEEL_DIAMETER,
            'pulses_per_wheel_rev': car.PULSES_PER_WHEEL_REV,
            'duty': duty,
        }],
        # Quan acaba la prova, s'acaba el launch.
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('duty', default_value='30.0', description='PWM dels motors (%)'),
        test_motors,
    ])
