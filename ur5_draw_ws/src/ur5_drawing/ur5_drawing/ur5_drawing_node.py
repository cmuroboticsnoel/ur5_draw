import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from moveit_msgs.msg import DisplayTrajectory
import json
import math
import time
import numpy as np
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from .drawing_utils import DrawingUtils

class UR5DrawingNode(Node):
    def __init__(self):
        super().__init__('ur5_drawing_node')
        
        # Use ReentrantCallbackGroup for parallel service handling
        self.callback_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('origin', [-0.345, -0.517, 0.032, 1.26, -2.928, 0]),
                ('image_width', 800),
                ('image_height', 600),
                ('drawing_file', 'drawing_sequences.json'),
                ('lift_offset', 0.02),
                ('drawing_speed', 0.25),
                ('paper_width', 0.2794),  # 11 inches in meters
                ('paper_height', 0.2159),  # 8.5 inches in meters
                ('planning_group', 'manipulator'),
                ('max_planning_attempts', 5),
                ('planning_time', 5.0),
                ('replan_attempts', 3),
                ('goal_position_tolerance', 0.001),
                ('goal_orientation_tolerance', 0.01),
                ('collision_object_padding', 0.01),
            ]
        )
        
        # Get parameters
        self.get_parameters()
        
        # Create service for on-demand drawing
        self.service = self.create_service(
            Trigger, 
            'start_drawing', 
            self.start_drawing_callback,
            callback_group=self.callback_group
        )
        
        # Initialize MoveIt! components
        self.robot_commander = None
        self.move_group = None
        self.planning_scene = None
        self.initialize_moveit()
        
        # Create publisher for visualization
        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            'display_planned_path',
            10
        )
        
        # Create marker publisher for drawing visualization
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'drawing_markers',
            10
        )
        
        self.get_logger().info("UR5 Drawing Node initialized. Ready to start drawing.")

    def get_parameters(self):
        """Retrieve and process parameters"""
        origin_params = self.get_parameter('origin').value
        self.origin = {
            'position': origin_params[:3],
            'orientation': origin_params[3:]
        }
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.drawing_file = self.get_parameter('drawing_file').value
        self.lift_offset = self.get_parameter('lift_offset').value
        self.drawing_speed = self.get_parameter('drawing_speed').value
        self.paper_width = self.get_parameter('paper_width').value
        self.paper_height = self.get_parameter('paper_height').value
        self.planning_group = self.get_parameter('planning_group').value
        self.max_planning_attempts = self.get_parameter('max_planning_attempts').value
        self.planning_time = self.get_parameter('planning_time').value
        self.replan_attempts = self.get_parameter('replan_attempts').value
        self.goal_position_tolerance = self.get_parameter('goal_position_tolerance').value
        self.goal_orientation_tolerance = self.get_parameter('goal_orientation_tolerance').value
        self.collision_object_padding = self.get_parameter('collision_object_padding').value

    def initialize_moveit(self):
        """Initialize MoveIt! components"""
        self.get_logger().info("Initializing MoveIt!...")
        
        try:
            # Initialize moveit_commander
            moveit_commander.roscpp_initialize([])
            
            # Create robot commander
            self.robot_commander = RobotCommander()
            
            # Create planning scene interface
            self.planning_scene = PlanningSceneInterface()
            
            # Create move group
            self.move_group = MoveGroupCommander(self.planning_group)
            
            # Set planner parameters
            self.move_group.set_goal_position_tolerance(self.goal_position_tolerance)
            self.move_group.set_goal_orientation_tolerance(self.goal_orientation_tolerance)
            self.move_group.set_planning_time(self.planning_time)
            self.move_group.set_num_planning_attempts(self.max_planning_attempts)
            
            self.get_logger().info("MoveIt! initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt!: {str(e)}")
            raise

    def start_drawing_callback(self, request, response):
        """Service callback to start the drawing process"""
        self.get_logger().info("Starting drawing process...")
        
        try:
            # Load drawing sequences
            sequences = self.load_drawing_sequences()
            
            # Setup collision environment
            self.setup_collision_environment()
            
            # Execute drawing
            self.execute_drawing(sequences)
            
            # Visualize the entire drawing
            self.visualize_drawing(sequences)
            
            response.success = True
            response.message = "Drawing completed successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Drawing failed: {str(e)}")
        
        return response

    def load_drawing_sequences(self):
        """Load drawing sequences from JSON file"""
        try:
            with open(self.drawing_file, 'r') as f:
                sequences = json.load(f)
            self.get_logger().info(f"Loaded {len(sequences)} sequences from {self.drawing_file}")
            return sequences
        except Exception as e:
            self.get_logger().error(f"Failed to load drawing sequences: {str(e)}")
            raise

    def setup_collision_environment(self):
        """Set up collision objects for the drawing environment"""
        # Clear existing collision objects
        self.planning_scene.clear()
        
        # Add table as a collision object
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = self.origin['position'][0]
        table_pose.pose.position.y = self.origin['position'][1]
        table_pose.pose.position.z = self.origin['position'][2] - 0.05
        table_pose.pose.orientation.w = 1.0
        
        self.planning_scene.add_box("table", table_pose, (0.5, 0.5, 0.1))
        
        # Add paper as a thin box
        paper_pose = PoseStamped()
        paper_pose.header.frame_id = "base_link"
        paper_pose.pose.position.x = self.origin['position'][0]
        paper_pose.pose.position.y = self.origin['position'][1]
        paper_pose.pose.position.z = self.origin['position'][2] + 0.001
        paper_pose.pose.orientation.w = 1.0
        
        self.planning_scene.add_box("paper", paper_pose, (self.paper_width, self.paper_height, 0.002))
        
        self.get_logger().info("Collision environment setup complete")

    def execute_drawing(self, sequences):
        """Execute the drawing process"""
        # Move to home position
        self.move_to_home()
        
        # Lift pen to safe height above paper
        home_pos = self.image_to_robot_frame([self.image_width/2, self.image_height/2])
        home_pos[2] += self.lift_offset
        self.move_to_pose(home_pos)
        
        # Draw each sequence
        for seq_idx, sequence in enumerate(sequences):
            self.get_logger().info(
                f"Drawing sequence {seq_idx+1}/{len(sequences)} with {len(sequence)} points"
            )
            
            # Create waypoints for this sequence
            waypoints = []
            
            # Start with pen lifted
            start_point = sequence[0]
            start_pos = self.image_to_robot_frame(start_point)
            lifted_start = start_pos.copy()
            lifted_start[2] += self.lift_offset
            waypoints.append(
                DrawingUtils.create_pose(
                    lifted_start, 
                    origin_orientation=self.origin['orientation']
                )
            )
            
            # Lower pen to drawing height
            waypoints.append(
                DrawingUtils.create_pose(
                    start_pos, 
                    origin_orientation=self.origin['orientation']
                )
            )
            
            # Add all points in the sequence
            for point in sequence:
                point_pos = self.image_to_robot_frame(point)
                waypoints.append(
                    DrawingUtils.create_pose(
                        point_pos, 
                        origin_orientation=self.origin['orientation']
                    )
                )
            
            # Lift pen at end of sequence
            end_pos = self.image_to_robot_frame(sequence[-1])
            lifted_end = end_pos.copy()
            lifted_end[2] += self.lift_offset
            waypoints.append(
                DrawingUtils.create_pose(
                    lifted_end, 
                    origin_orientation=self.origin['orientation']
                )
            )
            
            # Plan and execute Cartesian path
            self.execute_cartesian_path(waypoints)
        
        # Return to home position
        self.move_to_home()
        self.get_logger().info("Drawing completed!")

    def image_to_robot_frame(self, point):
        """Convert image point to robot frame"""
        return DrawingUtils.image_to_robot_frame(
            point,
            (self.image_width, self.image_height),
            (self.paper_width, self.paper_height),
            self.origin
        )

    def move_to_home(self):
        """Move to a predefined home position"""
        home_joint_angles = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.get_logger().info("Moving to home position...")
        self.move_group.set_joint_value_target(home_joint_angles)
        self.execute_plan_with_retry()

    def move_to_pose(self, position):
        """Move to a specific pose"""
        target_pose = DrawingUtils.create_pose(
            position, 
            origin_orientation=self.origin['orientation']
        )
        self.move_group.set_pose_target(target_pose)
        self.execute_plan_with_retry()

    def execute_cartesian_path(self, waypoints):
        """Plan and execute a Cartesian path through waypoints"""
        # Skip planning if no waypoints
        if len(waypoints) < 2:
            return
            
        # Plan Cartesian path
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            eef_step=0.005,
            jump_threshold=0.0
        )
        
        # Check if path was planned successfully
        if fraction < 0.9:
            self.get_logger().warning(
                f"Only {fraction*100:.1f}% of the path was planned. Attempting to replan segments..."
            )
            self.execute_segmented_path(waypoints)
        else:
            # Visualize and execute the plan
            self.visualize_plan(plan)
            self.execute_plan_with_retry(plan)

    def execute_segmented_path(self, waypoints):
        """Execute a path by breaking it into smaller segments"""
        for i in range(len(waypoints) - 1):
            segment = [waypoints[i], waypoints[i+1]]
            
            # Plan segment
            (plan, fraction) = self.move_group.compute_cartesian_path(
                segment,
                eef_step=0.005,
                jump_threshold=0.0
            )
            
            if fraction >= 0.9:
                self.execute_plan_with_retry(plan)
            else:
                self.get_logger().warning(
                    f"Segment {i+1}/{len(waypoints)-1} planning failed (fraction={fraction:.2f}). Using point-to-point."
                )
                self.move_group.set_pose_target(waypoints[i+1])
                self.execute_plan_with_retry()

    def execute_plan_with_retry(self, plan=None):
        """Execute a plan with retry logic"""
        attempts = 0
        while attempts < self.replan_attempts:
            try:
                if plan:
                    result = self.move_group.execute(plan, wait=True)
                else:
                    plan = self.move_group.plan()
                    if plan[0]:  # plan[0] is success boolean, plan[1] is the trajectory
                        result = self.move_group.execute(plan[1], wait=True)
                    else:
                        result = False
                
                if result:
                    return True
                
                self.get_logger().warning(f"Execution failed on attempt {attempts+1}. Replanning...")
                attempts += 1
            except Exception as e:
                self.get_logger().error(f"Execution error: {str(e)}")
                attempts += 1
        
        raise RuntimeError(f"Failed to execute plan after {self.replan_attempts} attempts")

    def visualize_plan(self, plan):
        """Visualize the planned trajectory in RViz"""
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot_commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def visualize_drawing(self, sequences):
        """Visualize the entire drawing as markers"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for seq_idx, sequence in enumerate(sequences):
            # Create line strip marker for this sequence
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "drawing"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.001  # Line width
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            # Add points to the line strip
            for point in sequence:
                pos = self.image_to_robot_frame(point)
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Drawing visualization published")

def main(args=None):
    rclpy.init(args=args)
    
    # Use MultiThreadedExecutor to handle service calls while planning
    executor = MultiThreadedExecutor()
    node = UR5DrawingNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()