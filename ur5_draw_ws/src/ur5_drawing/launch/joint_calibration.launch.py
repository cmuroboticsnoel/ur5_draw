import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def load_drawing_config(config_path):
    """Load joint-based drawing configuration from YAML file"""
    config_file = os.path.join(config_path, 'drawing_config.yaml')
    
    try:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        print(f"✓ Loaded joint drawing configuration from: {config_file}")
        return config
    except FileNotFoundError:
        print(f"✗ Warning: Configuration file not found: {config_file}")
        print("  Using default configuration values.")
        return get_default_joint_config()
    except yaml.YAMLError as e:
        print(f"✗ Error parsing YAML configuration: {e}")
        print("  Using default configuration values.")
        return get_default_joint_config()

def get_default_joint_config():
    """Return default joint-based configuration if config file is not available"""
    return {
        'network': {
            'robot_ip': '192.168.1.102',
            'pc_ip': '192.168.1.101',
            'ur_type': 'ur5'
        },
        'joint_calibration': {
            'home_position': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            'origin_position': [0.0, -1.2, 0.5, -1.57, 0.0, 0.0],
            'corners': {
                'bottom_left': [-0.3, -1.1, 0.4, -1.5, 0.1, 0.0],
                'bottom_right': [0.3, -1.1, 0.4, -1.5, -0.1, 0.0],
                'top_left': [-0.3, -1.3, 0.6, -1.6, 0.1, 0.0],
                'top_right': [0.3, -1.3, 0.6, -1.6, -0.1, 0.0]
            },
            'lift_offset_joints': [0.0, -0.05, 0.0, -0.05, 0.0, 0.0]
        },
        'physical': {
            'paper_width': 0.2794,
            'paper_height': 0.2159,
            'drawing_speed': 0.15
        },
        'image': {
            'width': 800,
            'height': 600
        },
        'planning': {
            'planning_group': 'ur_manipulator',
            'max_planning_attempts': 5,
            'planning_time': 5.0,
            'replan_attempts': 3,
            'goal_joint_tolerance': 0.01
        },
        'joint_trajectory': {
            'max_velocity_scaling': 0.1,
            'max_acceleration_scaling': 0.1,
            'joint_velocity_limits': [1.0, 1.0, 1.5, 2.0, 2.0, 2.0],
            'time_from_start': 2.0
        },
        'interpolation': {
            'method': 'linear',
            'interpolation_points': 10
        },
        'files': {
            'drawing_sequences': 'image_description.json',
            'rviz_config': 'drawing.rviz',
            'calibration_file': 'calibration.yaml',
            'joint_calibration_file': 'joint_calibration.yaml'
        },
        'launch': {
            'use_fake_hardware': False,
            'headless_mode': True,
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'default_use_rviz': True,
            'default_use_sim_time': False
        },
        'safety': {
            'max_velocity': 0.5,
            'max_acceleration': 1.0,
            'emergency_stop_deceleration': 3.0
        },
        'collision': {
            'object_padding': 0.01
        }
    }


def generate_launch_description():
    """
    Launch file specifically for joint calibration only.
    This assumes the robot system is already running.
    """
    
    # Load joint drawing configuration
    config_path = os.path.expanduser('~/ur5_draw/ur5_draw_ws/src/ur5_drawing/config')
    config = load_drawing_config(config_path)
    
    # Extract configuration values
    network_config = config['network']
    joint_calibration_config = config['joint_calibration']
    physical_config = config['physical']
    image_config = config['image']
    planning_config = config['planning']
    joint_trajectory_config = config['joint_trajectory']
    interpolation_config = config['interpolation']
    files_config = config['files']
    launch_config = config['launch']
    safety_config = config['safety']
    collision_config = config['collision']

    pkg_ur_robot_driver = get_package_share_directory('ur_robot_driver')
    CONFIG_PATH = os.path.join(config_path)
    CALIBRATION_FILE_PATH = os.path.join(CONFIG_PATH, files_config['calibration_file'])
    JOINT_CALIBRATION_FILE_PATH = os.path.join(CONFIG_PATH, files_config['joint_calibration_file'])

    # Launch arguments
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.102')
    
    # Joint calibration tool
    calibration_tool = Node(
        package='ur5_drawing',
        executable='joint_calibration',
        name='joint_calibration',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
        }]
    )

    safety_warning = ExecuteProcess(
        cmd=[
            'echo',
            '⚠️  SAFETY WARNING: Ensure robot is in MANUAL MODE before calibration!'
        ],
        shell=False,
        output='screen'
    )

        # UR5 launch arguments for joint-based control
    ur_launch_args = {
        'ur_type': network_config['ur_type'],
        'robot_ip': network_config['robot_ip'],
        'use_fake_hardware': 'false',
        'launch_rviz': 'false',  # We'll launch our own RViz
        'initial_joint_controller': launch_config['initial_joint_controller'],
        'kinematics_params_file': CALIBRATION_FILE_PATH,
        'headless_mode': 'false',
        'reverse_ip': network_config['pc_ip'],
        # Joint trajectory controller specific parameters
        'controller_spawner_timeout': '20',
        'activate_joint_controller': 'true'
    }
    
    # UR5 robot control launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_robot_driver, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments=ur_launch_args.items()
    )

    #status confirmation nodes
    ur_control_status = ExecuteProcess(
        cmd=['echo', '✓ UR5 Control with joint trajectory controller launched successfully'],
        shell=False,
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.102',
            description='IP address of the UR5 robot'
        ),
        ur_control_launch,
        TimerAction(period=3.0, actions=[ur_control_status]),

        safety_warning,
        calibration_tool,
    ])