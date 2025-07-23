import math
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from tf_transformations import quaternion_from_euler

class DrawingUtils:
    @staticmethod
    def image_to_robot_frame(point, image_size, paper_size, origin):
        """
        Convert image coordinates to robot frame
        
        Parameters:
        - point: (x, y) in image pixels
        - image_size: (width, height) in pixels
        - paper_size: (width, height) in meters
        - origin: dictionary with 'position' and 'orientation'
        
        Returns:
        - (x, y, z) in robot base frame
        """
        x_img, y_img = point
        img_width, img_height = image_size
        paper_width, paper_height = paper_size
        
        # Normalize image coordinates (0-1)
        x_ratio = x_img / img_width
        y_ratio = 1.0 - (y_img / img_height)  # Flip Y-axis
        
        # Calculate position relative to paper
        x_rel = (x_ratio - 0.5) * paper_width
        y_rel = (y_ratio - 0.5) * paper_height
        
        # Apply origin transformation
        x_robot = origin['position'][0] + x_rel
        y_robot = origin['position'][1] + y_rel
        z_robot = origin['position'][2]
        
        return (x_robot, y_robot, z_robot)

    @staticmethod
    def create_pose(position, orientation=None, origin_orientation=None):
        """
        Create a Pose message
        
        Parameters:
        - position: (x, y, z) tuple
        - orientation: Quaternion (x, y, z, w) or None to use origin orientation
        - origin_orientation: Euler angles (roll, pitch, yaw) if orientation is None
        
        Returns:
        - geometry_msgs.msg.Pose
        """
        pose = Pose()
        pose.position = Point(x=position[0], y=position[1], z=position[2])
        
        if orientation:
            pose.orientation = Quaternion(
                x=orientation[0],
                y=orientation[1],
                z=orientation[2],
                w=orientation[3]
            )
        elif origin_orientation:
            q = quaternion_from_euler(
                origin_orientation[0],
                origin_orientation[1],
                origin_orientation[2]
            )
            pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        return pose

    @staticmethod
    def distance_between_poses(pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    @staticmethod
    def plan_cartesian_path(move_group, waypoints, eef_step=0.01, jump_threshold=0.0, avoid_collisions=True):
        """
        Plan a Cartesian path through a series of waypoints
        
        Parameters:
        - move_group: MoveGroupCommander instance
        - waypoints: list of Pose targets
        - eef_step: step size between points (meters)
        - jump_threshold: maximum allowed jump between points (0=disabled)
        - avoid_collisions: whether to consider collisions
        
        Returns:
        - fraction: how much of the path was planned (0.0-1.0)
        - plan: MoveIt! plan object
        """
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            eef_step,    # eef_step
            jump_threshold,  # jump_threshold
            avoid_collisions=avoid_collisions)
        
        return fraction, plan