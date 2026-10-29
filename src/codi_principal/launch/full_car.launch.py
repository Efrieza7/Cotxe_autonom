import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# Paràmetres del cotxe real. Han de coincidir amb la simulació
# (car_simulator.launch.py i simulation_full.launch.py).
# ---------------------------------------------------------------------------
WHEELBASE = 0.18          # m, distància entre eixos
TARGET_SPEED = 0.2        # m/s
LOOKAHEAD = 0.45           # m, pure pursuit
SINGLE_SIDE_OFFSET = 0.20  # m, distància del camí a la paret quan només se'n veu una
# IMU (imu_reader): el gir de bicycle_model surt del giroscopi. Sense ella (False)
# el gir surt només del model, que es queda curt (a les proves el cotxe girava
# més del doble del calculat), i la velocitat només de l'encoder.
USE_IMU = True

# Motor i encoder (speed_control).
WHEEL_DIAMETER = 0.05436   # m
# Motor GA12-N20 amb encoder Hall: 7 polsos per volta del motor intern (fase A,
# flancs de pujada) x la reducció de la caixa. El motor és de 150 rpm a 6 V,
# que correspon a 1:100 (amb test_motors al 100 % es mesuren ~1470 polsos/s,
# ~700 per volta de l'eix de sortida a ~125 rpm amb 5 V).
GEAR_RATIO = 100.0
# Voltes de roda per cada volta de l'eix del motor: x5 als engranatges vermells
# i /2 al diferencial (corona / pinyó).
WHEEL_TURNS_PER_MOTOR_TURN = 2.5
PULSES_PER_WHEEL_REV = 7.0 * GEAR_RATIO / WHEEL_TURNS_PER_MOTOR_TURN  # 280

# Driver DRV8833 (pins BCM). Motor A amb encoder, motor B sense.
# NO facis servir els GPIO 8, 23, 24 ni 25: són de la Sense HAT (8 i 25
# programen i reinicien el seu microcontrolador, 23 i 24 són interrupcions
# de la IMU). Si es toquen, la matriu de LEDs no arrenca i els motors fan
# coses estranyes.
MOTOR_AIN1_PIN = 5        # BCM (GPIO 5 = pin físic 29)
MOTOR_AIN2_PIN = 6        # BCM (GPIO 6 = pin físic 31)
MOTOR_BIN1_PIN = 26       # BCM (GPIO 26 = pin físic 37)
MOTOR_BIN2_PIN = 16       # BCM (GPIO 16 = pin físic 36)
ENCODER_A_PIN = 27
ENCODER_B_PIN = 17
# Ajusta-ho amb `ros2 run my_pakage test_motors` (cotxe aixecat).
INVERT_MOTOR_A = True
INVERT_MOTOR_B = False
ENCODER_INVERT = False

# Servo de direcció (steering): pin BCM i amplades de pols als dos extrems i
# al centre. Calibrat amb `ros2 run my_pakage calibrate_servo` (29/10/2026):
# tope esquerre 1020 us, rodes rectes a 1390 us, tope dret 1710 us. steering
# suposa el mateix recorregut als dos costats: es fa servir el més curt (±320 us).
# (Abans, 27/10/2026: 1200 / 1560 / 1830 us; 26/10/2026: 810 / 1090 / 1510 us;
# 24/10/2026: 1060 / 1330 / 1600 us.)
SERVO_PIN = 18            # BCM (GPIO 18 = pin físic 12, PWM de maquinari)
SERVO_MIN_PULSE_US = 1070.0
SERVO_CENTER_PULSE_US = 1390.0
SERVO_MAX_PULSE_US = 1710.0
# Angle del servo a min/max pulse: 1000 us = 90 deg (±320 us = ±28.8 deg).
SERVO_MAX_ANGLE = math.radians(28.8)  # rad
SERVO_INVERT = True      # pols més curt = esquerra (tope esquerre a 1020 us, 29/10/2026)
# Suavitzat de la direcció (steering): velocitat màxima de gir de les rodes,
# filtre passa-baix de l'ordre i zona morta per ignorar correccions petites.
STEER_MAX_RATE = 1.0          # rad/s
STEER_SMOOTHING_TIME = 0.05   # s
STEER_DEADBAND = 0.01         # rad (~0.6 deg)

# Transmissió servo -> rodes (barres): el braç del servo i el de la mangueta
# desplacen la barra de direcció el mateix, de manera que
# angle_rodes = asin(SERVO_ARM_MM / KNUCKLE_ARM_MM · sin(angle_servo)).
SERVO_ARM_MM = 25.85
KNUCKLE_ARM_MM = 22.0    # de l'eix de gir de la roda a la barra de direcció (28/10/2026)
# Angle màxim de les rodes que permet el servo (~34.5 deg amb el servo a 28.8 deg).
MAX_STEER = math.asin(SERVO_ARM_MM / KNUCKLE_ARM_MM * math.sin(SERVO_MAX_ANGLE))  # rad

# El LIDAR (LD500) és sobre l'eix davanter (rodes de gir): és el punt (0, 0)
# de /pose. Només cal indicar si el seu zero no mira cap endavant.
# Està muntat girat 90 deg en sentit antihorari: el seu zero mira cap a
# l'esquerra del cotxe.
LIDAR_YAW_OFFSET = math.radians(90.0)    # rad, positiu = cap a l'esquerra
# Els punts del LIDAR més lluny d'això (m) s'ignoren al mapatge.
LIDAR_MAX_RANGE = 2.0
# Port del foxglove_bridge (Foxglove: ws://frieza-desktop.local:8765).
FOXGLOVE_PORT = 8765


def car_description():
    """car.urdf del simulador amb el LIDAR girat LIDAR_YAW_OFFSET, com al cotxe real."""
    urdf_path = os.path.join(get_package_share_directory('car_simulator'), 'urdf', 'car.urdf')
    with open(urdf_path, 'r') as f:
        urdf = f.read()
    mount = '<origin xyz="0 0 0.085" rpy="0 0 0"/>'
    if urdf.count(mount) != 1:
        raise RuntimeError(f'No trobo la unió del LIDAR a {urdf_path}')
    return urdf.replace(mount, f'<origin xyz="0 0 0.085" rpy="0 0 {LIDAR_YAW_OFFSET}"/>')


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')
    use_lidar = LaunchConfiguration('use_lidar')
    dry_run = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)
    start_yaw = ParameterValue(LaunchConfiguration('start_yaw'), value_type=float)
    viz = LaunchConfiguration('viz')

    # Driver del LIDAR (publica /ldlidar_node/scan). El LD500 (kit D500,
    # sensor STL-19P) fa servir el mateix protocol que el LD19, per això el
    # model per defecte és LDLiDAR_LD19.
    ldlidar_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_pakage'),
                'launch',
                'ldlidar_integration.launch.py',
            )
        ),
        launch_arguments={'serial_port': serial_port, 'lidar_model': lidar_model}.items(),
        condition=IfCondition(use_lidar),
    )

    nodes = [
        # --- Actuadors i sensors ---
        Node(
            package='my_pakage', executable='steering', name='steering', output='screen',
            parameters=[{
                'servo_pin': SERVO_PIN,
                'servo_max_angle': SERVO_MAX_ANGLE,
                'servo_arm_mm': SERVO_ARM_MM,
                'knuckle_arm_mm': KNUCKLE_ARM_MM,
                'min_pulse_us': SERVO_MIN_PULSE_US,
                'center_pulse_us': SERVO_CENTER_PULSE_US,
                'max_pulse_us': SERVO_MAX_PULSE_US,
                'invert': SERVO_INVERT,
                'max_delta_per_sec': STEER_MAX_RATE,
                'smoothing_time_sec': STEER_SMOOTHING_TIME,
                'deadband_rad': STEER_DEADBAND,
                'dry_run': dry_run,
            }],
        ),
        Node(
            package='my_pakage', executable='speed_control', name='speed_control', output='screen',
            parameters=[{
                'wheel_diameter': WHEEL_DIAMETER,
                'pulses_per_wheel_rev': PULSES_PER_WHEEL_REV,
                'ain1_pin': MOTOR_AIN1_PIN,
                'ain2_pin': MOTOR_AIN2_PIN,
                'bin1_pin': MOTOR_BIN1_PIN,
                'bin2_pin': MOTOR_BIN2_PIN,
                'enc_a_pin': ENCODER_A_PIN,
                'enc_b_pin': ENCODER_B_PIN,
                'invert_motor_a': INVERT_MOTOR_A,
                'invert_motor_b': INVERT_MOTOR_B,
                'encoder_invert': ENCODER_INVERT,
                'max_speed': 1.0,
                'dry_run': dry_run,
            }],
        ),
        # motor_reader no es llança: el seu port per defecte (/dev/ttyUSB0) és el
        # del LIDAR i el reconfigura a 9600 baud. La velocitat ja surt de
        # speed_control (wheel_speed, encoder).
        *([Node(package='my_pakage', executable='imu_reader', name='imu_reader', output='screen')]
          if USE_IMU else []),

        # --- Localització ---
        Node(
            package='my_pakage', executable='bicycle_model', name='bicycle_model', output='screen',
            parameters=[{'wheelbase': WHEELBASE, 'start_yaw': start_yaw}],
        ),

        # --- Mapatge de cons ---
        Node(
            package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator',
            output='screen',
            parameters=[{
                'wheelbase': WHEELBASE,
                'lidar_yaw_offset': LIDAR_YAW_OFFSET,
                'max_range': LIDAR_MAX_RANGE,
            }],
        ),
        Node(package='my_pakage', executable='lidar_processing', name='lidar_processing', output='screen'),

        # --- Planificació i control ---
        Node(
            package='my_pakage', executable='path_planner_bridge', name='path_planner_bridge',
            output='screen',
            parameters=[{
                'map_topic': '/ldlidar_node/cons_map',
                'pose_topic': '/pose',
                'path_topic': '/path_planning/waypoints',
                'mission': 'trackdrive',
                'planner_scale': 10.0,
                # m: distància a la paret quan només se'n veu una
                'single_side_offset': SINGLE_SIDE_OFFSET,
            }],
        ),
        Node(
            package='my_pakage', executable='path_follower', name='path_follower', output='screen',
            parameters=[{
                'path_topic': '/path_planning/waypoints',
                'pose_topic': '/pose',
                'steering_topic': 'target_angle',
                'speed_topic': 'target_speed',
                'lookahead': LOOKAHEAD,
                'wheelbase': WHEELBASE,
                'max_steer_rad': MAX_STEER,
                'target_speed': TARGET_SPEED,
            }],
        ),
    ]

    # --- Visualització (Foxglove) ---
    # Model del cotxe (TF base_link -> rodes, LIDAR), cons, camí i recorregut.
    # robot_description es publica a /car_description perquè el driver del
    # LIDAR ja en publica un altre a /robot_description.
    viz_nodes = [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='car_state_publisher', output='screen',
            parameters=[{'robot_description': car_description()}],
            remappings=[('/robot_description', '/car_description')],
            condition=IfCondition(viz),
        ),
        Node(package='my_pakage', executable='car_viz', name='car_viz', output='screen',
             condition=IfCondition(viz)),
        Node(package='my_pakage', executable='cone_map_viz', name='cone_map_viz', output='screen',
             condition=IfCondition(viz)),
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
            output='screen', parameters=[{'port': FOXGLOVE_PORT}],
            condition=IfCondition(viz),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ldlidar', description='Serial port for the lidar'
        ),
        DeclareLaunchArgument(
            'lidar_model', default_value='LDLiDAR_LD19',
            description='Driver protocol: the LD500 uses the LD19 protocol'
        ),
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the LD19 driver (false to feed /ldlidar_node/scan from elsewhere)'
        ),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true: do not drive motor/servo, echo commands as measurements (bench test)'
        ),
        DeclareLaunchArgument(
            'start_yaw', default_value='0.0', description='Initial heading of /pose (rad)'
        ),
        DeclareLaunchArgument(
            'viz', default_value='true',
            description='Start the Foxglove visualization (car model, cones, path) and foxglove_bridge'
        ),
        ldlidar_include,
        *nodes,
        *viz_nodes,
    ])
