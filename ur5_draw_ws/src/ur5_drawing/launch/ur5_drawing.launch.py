import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
            'drawing_speed': 0.25
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
            'max_velocity_scaling': 0.3,
            'max_acceleration_scaling': 0.2,
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
    # Get package directories
    pkg_ur_drawing = get_package_share_directory('ur5_drawing')
    pkg_ur_robot_driver = get_package_share_directory('ur_robot_driver')
    pkg_ur_moveit_config = get_package_share_directory('ur_moveit_config')
    pkg_ur_calibration = get_package_share_directory('ur_calibration')

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

    # Build file paths - Updated to use your actual file locations
    CONFIG_PATH = os.path.join(config_path)
    MOVEIT_CONFIG_PATH = os.path.join(config_path, 'move_it')
    CALIBRATION_FILE_PATH = os.path.join(CONFIG_PATH, files_config['calibration_file'])
    JOINT_CALIBRATION_FILE_PATH = os.path.join(CONFIG_PATH, files_config['joint_calibration_file'])
    RVIZ_CONFIG_FILE = os.path.join(CONFIG_PATH, 'drawing.rviz')  # Updated path

    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz', default=str(launch_config['default_use_rviz']).lower())
    use_sim_time = LaunchConfiguration('use_sim_time', default=str(launch_config['default_use_sim_time']).lower())
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default=str(launch_config['use_fake_hardware']).lower())
    enable_calibration = LaunchConfiguration('enable_calibration', default='true')

    # Calibration launch (only for real hardware)
    ur_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_calibration, 'launch', 'calibration_correction.launch.py')
        ),
        launch_arguments={
            'robot_ip': network_config['robot_ip'],
            'target_filename': CALIBRATION_FILE_PATH
        }.items(),
        condition=UnlessCondition(use_fake_hardware)
    )

    # UR5 launch arguments for joint-based control
    ur_launch_args = {
        'ur_type': network_config['ur_type'],
        'robot_ip': network_config['robot_ip'],
        'use_fake_hardware': use_fake_hardware,
        'launch_rviz': 'false',  # We'll launch our own RViz
        'kinematics_params_file': CALIBRATION_FILE_PATH,
        'headless_mode': str(launch_config['headless_mode']).lower(),
        'reverse_ip': network_config['pc_ip'],
    }

    # UR5 robot control launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_robot_driver, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments=ur_launch_args.items()
    )

    # Custom MoveIt! launch with your configuration files
    moveit_launch_args = {
        'use_sim_time': use_sim_time,
        'launch_rviz': 'false',  # We'll launch our own RViz
        'ur_type': network_config['ur_type'],
        'warehouse_sqlite_path': os.path.expanduser('~/drawing_moveit_warehouse.sqlite'),
        # Use your custom MoveIt configuration files
        'moveit_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'ur.srdf.xacro'),
        'kinematics_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'kinematics.yaml'),
        'joint_limits_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'joint_limits.yaml'),
        'controllers_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'controllers.yaml'),
        'ompl_planning_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'ompl_planning.yaml'),
        'servo_config_file': os.path.join(MOVEIT_CONFIG_PATH, 'ur_servo.yaml')
    }

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_moveit_config, 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments=moveit_launch_args.items()
    )

    # RViz node with your custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_drawing',
        output='log',
        arguments=['-d', RVIZ_CONFIG_FILE],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description_semantic': 'robot_description_semantic'
        }],
        condition=IfCondition(use_rviz)
    )

    # Joint-based drawing node - MAIN NODE
    drawing_node = Node(
        package='ur5_drawing',
        executable='ur5_drawing_node',
        name='ur5_drawing_node',
        output='screen',  # This is the primary output node
        parameters=[{
            'use_sim_time': use_sim_time,
            # Pass configuration parameters directly
            'robot_ip': network_config['robot_ip'],
            'ur_type': network_config['ur_type'],
            'planning_group': planning_config['planning_group'],
            'max_velocity_scaling': joint_trajectory_config['max_velocity_scaling'],
            'max_acceleration_scaling': joint_trajectory_config['max_acceleration_scaling'],
            'interpolation_method': interpolation_config['method'],
            'interpolation_points': interpolation_config['interpolation_points'],
            'config_path': CONFIG_PATH,
            'moveit_config_path': MOVEIT_CONFIG_PATH,
            'joint_calibration_file': JOINT_CALIBRATION_FILE_PATH
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/scaled_joint_trajectory_controller/follow_joint_trajectory', 
             '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        ]
    )

    # Joint trajectory state broadcaster (for visualization)
    joint_state_broadcaster = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher_gui',
        output='log',
        condition=IfCondition(use_fake_hardware)
    )

    # Status confirmation nodes
    ur_control_status = ExecuteProcess(
        cmd=['echo', '✓ UR5 Control with joint trajectory controller launched successfully'],
        shell=False,
        output='screen'
    )

    moveit_status = ExecuteProcess(
        cmd=['echo', '✓ MoveIt joint planning with custom config launched successfully'],
        shell=False,
        output='screen'
    )

    rviz_status = ExecuteProcess(
        cmd=['echo', f'✓ RViz joint drawing visualization launched with config: {RVIZ_CONFIG_FILE}'],
        shell=False,
        output='screen',
        condition=IfCondition(use_rviz)
    )

    calibration_info = ExecuteProcess(
        cmd=['echo', 'To calibrate joint positions, run: ros2 run ur5_drawing joint_calibration_tool'],
        shell=False,
        output='screen'
    )

    drawing_info = ExecuteProcess(
        cmd=['echo', 'To start drawing, run: ros2 service call /start_drawing std_srvs/srv/Trigger'],
        shell=False,
        output='screen'
    )

    config_info = ExecuteProcess(
        cmd=['echo', f'Using MoveIt config from: {MOVEIT_CONFIG_PATH}'],
        shell=False,
        output='screen'
    )

    # Group actions for fake hardware mode
    fake_hardware_group = GroupAction(
        actions=[
            ur_control_launch,
            TimerAction(period=3.0, actions=[ur_control_status]),
            moveit_launch,
            TimerAction(period=6.0, actions=[moveit_status]),
            joint_state_broadcaster,
            rviz_node,
            TimerAction(period=8.0, actions=[rviz_status]),
            TimerAction(period=10.0, actions=[drawing_node, calibration_info]),
            TimerAction(period=11.0, actions=[config_info]),
            TimerAction(period=12.0, actions=[drawing_info])
        ],
        condition=IfCondition(use_fake_hardware)
    )

    # Group actions for real hardware mode
    real_hardware_group = GroupAction(
        actions=[
            ur_calibration_launch,
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=['echo', '✓ Calibration completed. Starting robot systems...'],
                        shell=False,
                        output='screen'
                    ),
                    ur_control_launch,
                    TimerAction(period=3.0, actions=[ur_control_status]),
                    moveit_launch,
                    TimerAction(period=6.0, actions=[moveit_status]),
                    rviz_node,
                    TimerAction(period=8.0, actions=[rviz_status]),
                    TimerAction(period=10.0, actions=[drawing_node, calibration_info]),
                    TimerAction(period=11.0, actions=[config_info]),
                    TimerAction(period=12.0, actions=[drawing_info])
                ]
            )
        ],
        condition=UnlessCondition(use_fake_hardware)
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value=str(launch_config['default_use_rviz']).lower(),
            description='Launch RViz for joint drawing visualization'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=str(launch_config['default_use_sim_time']).lower(),
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=str(launch_config['use_fake_hardware']).lower(),
            description='Use fake hardware (simulation mode)'
        ),

        DeclareLaunchArgument(
            'enable_calibration',
            default_value='true',
            description='Enable robot calibration (real hardware only)'
        ),

        # Launch components based on hardware mode
        fake_hardware_group,
        real_hardware_group,
    ])