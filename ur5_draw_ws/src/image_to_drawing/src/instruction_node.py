#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener, Buffer
import numpy as np
import cv2
import math
from scipy.spatial.transform import Rotation

class DrawingNode(Node):
    def __init__(self):
        super().__init__('drawing_node')
        
        # Parameters
        self.declare_parameter('pen_up_offset', 0.05)  # 5cm above canvas
        self.declare_parameter('pen_down_offset', 0.0)  # Touching canvas
        self.declare_parameter('drawing_speed', 0.02)   # m/s
        self.declare_parameter('calibration_points', 4)
        
        # MoveIt services
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')
        
        # TF listener for frame transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint state publisher for execution
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # State variables
        self.calibration_points = []
        self.homography = None
        self.canvas_z = None
        self.drawing_orientation = None
        
        self.get_logger().info("Drawing node initialized. Waiting for calibration...")

    def calibrate(self, image_width, image_height):
        """Calibration routine to map image coordinates to robot workspace"""
        num_points = self.get_parameter('calibration_points').value
        self.calibration_points = []
        
        # Define image corner points
        img_corners = np.array([
            [0, 0],
            [image_width, 0],
            [image_width, image_height],
            [0, image_height]
        ], dtype=np.float32)
        
        self.get_logger().info("Starting calibration...")
        
        # Collect robot positions for each corner
        for i in range(num_points):
            input(f"Move robot to image corner ({img_corners[i][0]}, {img_corners[i][1]}) and press Enter")
            current_pose = self.get_current_pose()
            if current_pose:
                robot_point = [current_pose.position.x, current_pose.position.y]
                self.calibration_points.append(robot_point)
                
                # Use first point for Z and orientation reference
                if i == 0:
                    self.canvas_z = current_pose.position.z
                    self.drawing_orientation = current_pose.orientation
                    self.get_logger().info(f"Canvas Z set to: {self.canvas_z:.3f}m")
        
        # Compute homography matrix
        robot_points = np.array(self.calibration_points, dtype=np.float32)
        self.homography, _ = cv2.findHomography(img_corners, robot_points)
        self.get_logger().info("Calibration complete!")

    def get_current_pose(self):
        """Get current end-effector pose in base frame"""
        try:
            # Look up current transform from base to tool0
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
            
        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")
            return None

    def image_to_robot(self, point_img):
        """Convert image coordinate to robot position using homography"""
        if self.homography is None:
            self.get_logger().error("Calibration not done!")
            return None
        
        # Homogeneous coordinates
        point_img_hom = np.array([point_img[0], point_img[1], 1])
        point_robot_hom = self.homography @ point_img_hom
        point_robot = point_robot_hom[:2] / point_robot_hom[2]
        
        return point_robot

    def compute_ik(self, position, orientation=None):
        """Compute inverse kinematics for a position"""
        if orientation is None and self.drawing_orientation is None:
            self.get_logger().error("No orientation available!")
            return None
            
        req = GetPositionIK.Request()
        req.ik_request.group_name = "manipulator"
        req.ik_request.robot_state.joint_state = self.get_current_joint_state()
        req.ik_request.avoid_collisions = True
        
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation = orientation if orientation else self.drawing_orientation
        
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose = pose
        
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            return future.result().solution.joint_state
        return None

    def get_current_joint_state(self):
        """Get current joint state (simplified)"""
        # In real implementation, subscribe to /joint_states
        return JointState()

    def create_trajectory_point(self, positions, time_from_start):
        """Create a trajectory point"""
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rclpy.time.Duration(seconds=time_from_start).to_msg()
        return point

    def draw_sequence(self, sequence):
        """Draw a single edge sequence"""
        if not sequence or len(sequence) < 2:
            return
            
        pen_up_z = self.canvas_z + self.get_parameter('pen_up_offset').value
        pen_down_z = self.canvas_z + self.get_parameter('pen_down_offset').value
        speed = self.get_parameter('drawing_speed').value
        
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'shoulder_pan_joint', 
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Move to start position (pen up)
        start_pos_robot = self.image_to_robot(sequence[0])
        start_pose = [start_pos_robot[0], start_pos_robot[1], pen_up_z]
        start_joints = self.compute_ik(start_pose)
        
        if start_joints:
            trajectory.points.append(
                self.create_trajectory_point(start_joints.position, 2.0)
        
        # Lower pen
        start_pose_down = [start_pos_robot[0], start_pos_robot[1], pen_down_z]
        down_joints = self.compute_ik(start_pose_down)
        
        if down_joints:
            trajectory.points.append(
                self.create_trajectory_point(down_joints.position, 3.0))
        
        # Draw sequence
        time = 4.0
        for point in sequence:
            pos_robot = self.image_to_robot(point)
            if not pos_robot:
                continue
                
            pose = [pos_robot[0], pos_robot[1], pen_down_z]
            joints = self.compute_ik(pose)
            
            if joints:
                trajectory.points.append(
                    self.create_trajectory_point(joints.position, time))
                distance = math.sqrt(
                    (pos_robot[0] - trajectory.points[-1].positions[0])**2 +
                    (pos_robot[1] - trajectory.points[-1].positions[1])**2
                )
                time += max(0.1, distance / speed)
        
        # Lift pen
        last_point = sequence[-1]
        last_pos_robot = self.image_to_robot(last_point)
        up_pose = [last_pos_robot[0], last_pos_robot[1], pen_up_z]
        up_joints = self.compute_ik(up_pose)
        
        if up_joints:
            trajectory.points.append(
                self.create_trajectory_point(up_joints.position, time + 1.0))
        
        # Execute trajectory
        self.joint_pub.publish(trajectory)
        self.get_logger().info(f"Executing sequence with {len(trajectory.points)} points")

    def draw_all_sequences(self, sequences):
        """Draw all edge sequences"""
        if not sequences:
            self.get_logger().warn("No sequences to draw!")
            return
            
        for i, seq in enumerate(sequences):
            self.get_logger().info(f"Drawing sequence {i+1}/{len(sequences)}")
            self.draw_sequence(seq)
            
        self.get_logger().info("All sequences drawn!")

def main(args=None):
    rclpy.init(args=args)
    
    # Example usage:
    drawing_node = DrawingNode()
    
    # Load your edge sequences here (from image processing)
    # sequences = load_sequences_from_file()
    sequences = [
        [[10, 10], [100, 10], [100, 100], [10, 100]],  # Example square
        [[50, 50], [200, 200]]                           # Example diagonal
    ]
    
    # Calibrate (image size should match your processed image)
    drawing_node.calibrate(image_width=640, image_height=480)
    
    # Draw sequences
    drawing_node.draw_all_sequences(sequences)
    
    rclpy.spin(drawing_node)
    drawing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()