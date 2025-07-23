#include "../include/drawing_utils.hpp"

namespace ur5_drawing {

DrawingUtils::Point3D DrawingUtils::imageToRobotFrame(
    const Point2D& point,
    const ImageSize& image_size,
    const PaperSize& paper_size,
    const Origin& origin) {
    
    double x_img = point[0];
    double y_img = point[1];
    int img_width = image_size[0];
    int img_height = image_size[1];
    double paper_width = paper_size[0];
    double paper_height = paper_size[1];
    
    // Normalize image coordinates (0-1)
    double x_ratio = x_img / static_cast<double>(img_width);
    double y_ratio = 1.0 - (y_img / static_cast<double>(img_height)); // Flip Y-axis
    
    // Calculate position relative to paper
    double x_rel = (x_ratio - 0.5) * paper_width;
    double y_rel = (y_ratio - 0.5) * paper_height;
    
    // Apply origin transformation
    Point3D robot_pos;
    robot_pos[0] = origin.position[0] + x_rel;
    robot_pos[1] = origin.position[1] + y_rel;
    robot_pos[2] = origin.position[2];
    
    return robot_pos;
}

geometry_msgs::msg::Pose DrawingUtils::createPose(
    const Point3D& position,
    const std::array<double, 4>* orientation,
    const Orientation* origin_orientation) {
    
    geometry_msgs::msg::Pose pose;
    
    // Set position
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];
    
    // Set orientation
    if (orientation) {
        pose.orientation.x = (*orientation)[0];
        pose.orientation.y = (*orientation)[1];
        pose.orientation.z = (*orientation)[2];
        pose.orientation.w = (*orientation)[3];
    } else if (origin_orientation) {
        auto quat = eulerToQuaternion(
            (*origin_orientation)[0],
            (*origin_orientation)[1],
            (*origin_orientation)[2]
        );
        pose.orientation.x = quat[0];
        pose.orientation.y = quat[1];
        pose.orientation.z = quat[2];
        pose.orientation.w = quat[3];
    } else {
        // Default orientation (identity quaternion)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
    }
    
    return pose;
}

double DrawingUtils::distanceBetweenPoses(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) {
    
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::pair<double, moveit_msgs::msg::RobotTrajectory> DrawingUtils::planCartesianPath(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double eef_step,
    double jump_threshold,
    bool avoid_collisions) {
    
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        avoid_collisions
    );
    
    return std::make_pair(fraction, trajectory);
}

std::array<double, 4> DrawingUtils::eulerToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    std::array<double, 4> result;
    result[0] = q.x();
    result[1] = q.y();
    result[2] = q.z();
    result[3] = q.w();
    
    return result;
}

} // namespace ur5_drawing