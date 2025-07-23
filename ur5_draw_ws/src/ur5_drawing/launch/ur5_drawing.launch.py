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
            'initial_joint_controller': 'joint_trajectory_controller',
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
    pkg_src = os.path.expanduser('~/ur5_draw/ur5_draw_ws/src/ur5_drawing')
    pkg_ur_robot_driver = get_package_share_directory('ur_robot_driver')
    pkg_ur_moveit_config = get_package_share_directory('ur_moveit_config')
    pkg_ur_calibration = get_package_share_directory('ur_calibration')
    
    # Load drawing configuration
    config = load_drawing_config(pkg_src)
    
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
    CONFIG_PATH = os.path.join(pkg_src, 'config')
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
        launch_arguments=ur_launch_args.items()
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
    rviz_config_file = os.path.join(pkg_src, 'resource', files_config['rviz_config'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Drawing node with all configuration parameters
    drawing_node = Node(
        package='ur5_drawing',
        executable='ur5_drawing_node',
        name='ur5_drawing_node',
        output='screen',
        parameters=[{
            # Physical configuration
            'origin': physical_config['origin'],
            'paper_width': physical_config['paper_width'],
            'paper_height': physical_config['paper_height'],
            'lift_offset': physical_config['lift_offset'],
            'drawing_speed': physical_config['drawing_speed'],
            
            # Image configuration
            'image_width': image_config['width'],
            'image_height': image_config['height'],
            
            # Planning configuration
            'planning_group': planning_config['planning_group'],
            'max_planning_attempts': planning_config['max_planning_attempts'],
            'planning_time': planning_config['planning_time'],
            'replan_attempts': planning_config['replan_attempts'],
            'goal_position_tolerance': planning_config['goal_position_tolerance'],
            'goal_orientation_tolerance': planning_config['goal_orientation_tolerance'],
            
            # Cartesian planning parameters
            'eef_step': cartesian_config.get('eef_step', 0.005),
            'jump_threshold': cartesian_config.get('jump_threshold', 0.0),
            'avoid_collisions': cartesian_config.get('avoid_collisions', True),
            
            # Safety parameters
            'max_velocity': safety_config.get('max_velocity', 0.5),
            'max_acceleration': safety_config.get('max_acceleration', 1.0),
            
            # Collision parameters
            'collision_object_padding': collision_config['object_padding'],
            
            # File paths
            'drawing_file': os.path.join(pkg_src, 'resource', files_config['drawing_sequences']),
        }]
    )

    post_calibration_actions = [
        ExecuteProcess(
            cmd=['echo', '✓ Calibration initiated. Waiting for robot control...'],
            shell=False,
            output='screen'
        ),
        # ur_control_launch,
        # moveit_launch,
        # rviz_node,
        drawing_node
    ]
    
    # Print configuration summary
    print("\n" + "="*60)
    print("UR5 DRAWING LAUNCH CONFIGURATION")
    print("="*60)
    print(f"Robot Type: {network_config['ur_type']}")
    print(f"Robot IP: {network_config['robot_ip']}")
    print(f"PC IP: {network_config['pc_ip']}")
    print(f"Use Fake Hardware: {launch_config['use_fake_hardware']}")
    print(f"Paper Size: {physical_config['paper_width']:.3f}m × {physical_config['paper_height']:.3f}m")
    print(f"Drawing Speed: {physical_config['drawing_speed']} m/s")
    print(f"Pen Lift Height: {physical_config['lift_offset']*1000:.1f} mm")
    print(f"Planning Group: {planning_config['planning_group']}")
    print(f"Config File: {os.path.join(pkg_src, 'config', 'drawing_config.yaml')}")
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
            period=5.0,  # <--- Adjust this delay (e.g., 5 seconds)
            actions=post_calibration_actions,
            cancel_on_shutdown=True
        ),
    ])