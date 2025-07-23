import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def load_drawing_config(package_path):
    """Load drawing configuration from YAML file"""
    config_file = os.path.join(package_path, 'config', 'drawing_config.yaml')
    
    try:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        print(f"✓ Loaded drawing configuration from: {config_file}")
        return config
    except FileNotFoundError:
        print(f"✗ Warning: Configuration file not found: {config_file}")
        print("  Using default configuration values.")
        return get_default_config()
    except yaml.YAMLError as e:
        print(f"✗ Error parsing YAML configuration: {e}")
        print("  Using default configuration values.")
        return get_default_config()

def get_default_config():
    """Return default configuration if config file is not available"""
    return {
        'network': {
            'robot_ip': '192.168.1.102',
            'pc_ip': '192.168.1.101',
            'ur_type': 'ur5'
        },
        'physical': {
            'origin': [-0.345, -0.517, 0.032, 1.26, -2.928, 0],
            'paper_width': 0.2794,
            'paper_height': 0.2159,
            'lift_offset': 0.02,
            'drawing_speed': 0.25
        },
        'image': {
            'width': 800,
            'height': 600
        },
        'planning': {
            'planning_group': 'manipulator',
            'max_planning_attempts': 5,
            'planning_time': 5.0,
            'replan_attempts': 3,
            'goal_position_tolerance': 0.001,
            'goal_orientation_tolerance': 0.01
        },
        'files': {
            'drawing_sequences': 'drawing_sequences.json',
            'rviz_config': 'drawing.rviz',
            'calibration_file': 'calibration.yaml'
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
            'max_acceleration': 1.0
        },
        'collision': {
            'object_padding': 0.01
        },
        'cartesian': {
            'eef_step': 0.005,
            'jump_threshold': 0.0,
            'avoid_collisions': True
        }
    }

def generate_launch_description():
    # Get package directories
    pkg_ur_drawing = get_package_share_directory('ur5_drawing')
    pkg_ur_robot_driver = get_package_share_directory('ur_robot_driver')
    pkg_ur_moveit_config = get_package_share_directory('ur_moveit_config')
    pkg_ur_calibration = get_package_share_directory('ur_calibration')
    
    # Load drawing configuration
    config_path = os.path.expanduser('~/ur5_draw/ur5_draw_ws/src/ur5_drawing/config')
    config = load_drawing_config(config_path)
    
    # Extract configuration values
    network_config = config['network']
    physical_config = config['physical']
    image_config = config['image']
    planning_config = config['planning']
    files_config = config['files']
    launch_config = config['launch']
    safety_config = config['safety']
    collision_config = config['collision']
    cartesian_config = config.get('planning', {}).get('cartesian', config.get('cartesian', {}))
    
    # Build file paths
    CONFIG_PATH = os.path.join(config_path)
    CALIBRATION_FILE_PATH = os.path.join(CONFIG_PATH, files_config['calibration_file'])
    
    # Configuration variables from launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default=str(launch_config['default_use_rviz']).lower())
    use_sim_time = LaunchConfiguration('use_sim_time', default=str(launch_config['default_use_sim_time']).lower())
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default=str(launch_config['use_fake_hardware']).lower())

    # Calibration launch
    ur_calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_calibration, 'launch', 'calibration_correction.launch.py')
        ),
        launch_arguments={
            'robot_ip': network_config['robot_ip'],
            'target_filename': CALIBRATION_FILE_PATH
        }.items(),
    )
    
    # UR5 launch arguments
    ur_launch_args = {
        'ur_type': network_config['ur_type'],
        'robot_ip': network_config['robot_ip'],
        'use_fake_hardware': use_fake_hardware,
        'launch_rviz': 'false',  # We'll launch our own RViz
        'initial_joint_controller': launch_config['initial_joint_controller'],
        'kinematics_params_file': CALIBRATION_FILE_PATH,
        'headless_mode': str(launch_config['headless_mode']).lower(),
        'reverse_ip': network_config['pc_ip']
    }
    
    # UR5 robot control launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_robot_driver, 'launch', 'ur_control.launch.py')
        ),
        launch_arguments=dict(ur_launch_args, **{'launch_rviz': 'false'}).items()
    )
    
    # MoveIt! configuration
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur_moveit_config, 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': 'false',  # We'll launch our own RViz
            'ur_type': network_config['ur_type']
        }.items()
    )
    
    # RViz node with custom configuration
    rviz_config_file = os.path.join(pkg_ur_drawing, 'resource', files_config['rviz_config'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',  # Send RViz output to log files, not terminal
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # C++ Drawing node - ONLY this node outputs to screen
    drawing_node = Node(
        package='ur5_drawing',
        executable='ur5_drawing_node',
        name='ur5_drawing_node',
        output='screen',  # This is the only node that outputs to terminal
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Status confirmation nodes
    ur_control_status = ExecuteProcess(
        cmd=['echo', '✓ UR Control launched successfully'],
        shell=False,
        output='screen'
    )
    
    moveit_status = ExecuteProcess(
        cmd=['echo', '✓ MoveIt planning launched successfully'],
        shell=False,
        output='screen'
    )
    
    rviz_status = ExecuteProcess(
        cmd=['echo', '✓ RViz visualization launched successfully'],
        shell=False,
        output='screen',
        condition=IfCondition(use_rviz)
    )

    post_calibration_actions = [
        ExecuteProcess(
            cmd=['echo', '✓ Calibration completed. Starting robot systems...'],
            shell=False,
            output='screen'
        ),
        ur_control_launch,
        TimerAction(
            period=3.0,
            actions=[ur_control_status]
        ),
        moveit_launch,
        TimerAction(
            period=6.0,
            actions=[moveit_status]
        ),
        rviz_node,
        TimerAction(
            period=8.0,
            actions=[rviz_status]
        ),
        TimerAction(
            period=10.0,
            actions=[drawing_node]
        )
    ]
    
    # Print configuration summary
    print("\n" + "="*60)
    print("UR5 DRAWING C++ LAUNCH CONFIGURATION")
    print("="*60)
    print(f"Robot Type: {network_config['ur_type']}")
    print(f"Robot IP: {network_config['robot_ip']}")
    print(f"PC IP: {network_config['pc_ip']}")
    print(f"Use Fake Hardware: {launch_config['use_fake_hardware']}")
    print(f"Paper Size: {physical_config['paper_width']:.3f}m × {physical_config['paper_height']:.3f}m")
    print(f"Drawing Speed: {physical_config['drawing_speed']} m/s")
    print(f"Pen Lift Height: {physical_config['lift_offset']*1000:.1f} mm")
    print(f"Planning Group: {planning_config['planning_group']}")
    print(f"Config Path: {CONFIG_PATH}")
    print("="*60 + "\n")
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_rviz',
            default_value=str(launch_config['default_use_rviz']).lower(),
            description='Launch RViz for visualization'
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
        
        # Launch components
        ur_calibration_launch,  # Only runs with real hardware
        TimerAction(
            period=5.0,  # Adjust this delay (e.g., 5 seconds)
            actions=post_calibration_actions,
            cancel_on_shutdown=True
        ),
    ])