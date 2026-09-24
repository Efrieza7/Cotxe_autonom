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
MAX_STEER = 0.785398      # rad (45 deg)
TARGET_SPEED = 0.5        # m/s
LOOKAHEAD = 0.2           # m, pure pursuit

# Motor i encoder (speed_control). MESURA el diàmetre de la roda!
WHEEL_DIAMETER = 0.065    # m
PULSES_PER_WHEEL_REV = 600.0

# Servo de direcció (direccion): pin BCM i amplades de pols a -45, 0 i +45 deg.
SERVO_PIN = 13
SERVO_MIN_PULSE_US = 1000.0
SERVO_CENTER_PULSE_US = 1500.0
SERVO_MAX_PULSE_US = 2000.0
SERVO_INVERT = False

# El LIDAR (LD500) és sobre l'eix davanter (rodes de gir): és el punt (0, 0)
# de /pose. Només cal indicar si el seu zero no mira cap endavant.
LIDAR_YAW_OFFSET = 0.0    # rad, positiu = cap a l'esquerra


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    lidar_model = LaunchConfiguration('lidar_model')
    use_lidar = LaunchConfiguration('use_lidar')
    dry_run = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)
    start_yaw = ParameterValue(LaunchConfiguration('start_yaw'), value_type=float)

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
            package='my_pakage', executable='direccion', name='direccion', output='screen',
            parameters=[{
                'servo_pin': SERVO_PIN,
                'max_angle': MAX_STEER,
                'min_pulse_us': SERVO_MIN_PULSE_US,
                'center_pulse_us': SERVO_CENTER_PULSE_US,
                'max_pulse_us': SERVO_MAX_PULSE_US,
                'invert': SERVO_INVERT,
                'dry_run': dry_run,
            }],
        ),
        Node(
            package='my_pakage', executable='speed_control', name='speed_control', output='screen',
            parameters=[{
                'wheel_diameter': WHEEL_DIAMETER,
                'pulses_per_wheel_rev': PULSES_PER_WHEEL_REV,
                'max_speed': 1.0,
                'dry_run': dry_run,
            }],
        ),
        Node(package='my_pakage', executable='motor_reader', name='motor_reader', output='screen'),
        Node(package='my_pakage', executable='imu_suscriber', name='imu_suscriber', output='screen'),

        # --- Localització ---
        Node(
            package='my_pakage', executable='bycicle_mode', name='bycicle_mode', output='screen',
            parameters=[{'wheelbase': WHEELBASE, 'start_yaw': start_yaw}],
        ),

        # --- Mapatge de cons ---
        Node(
            package='my_pakage', executable='lidar_image_creator', name='lidar_image_creator',
            output='screen',
            parameters=[{
                'wheelbase': WHEELBASE,
                'lidar_yaw_offset': LIDAR_YAW_OFFSET,
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
        ldlidar_include,
        *nodes,
    ])
