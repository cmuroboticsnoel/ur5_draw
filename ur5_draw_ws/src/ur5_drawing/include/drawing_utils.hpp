#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace ur5_drawing {

/**
 * @brief Utility class for drawing operations and coordinate transformations
 */
class DrawingUtils {
public:
    using Point2D = std::array<double, 2>;
    using Point3D = std::array<double, 3>;
    using ImageSize = std::array<int, 2>;
    using PaperSize = std::array<double, 2>;
    using Orientation = std::array<double, 3>; // Roll, Pitch, Yaw
    
    struct Origin {
        Point3D position;
        Orientation orientation;
    };

    /**
     * @brief Convert image coordinates to robot frame
     * @param point Image coordinates (x, y) in pixels
     * @param image_size Image dimensions (width, height) in pixels
     * @param paper_size Paper dimensions (width, height) in meters
     * @param origin Origin transformation with position and orientation
     * @return Robot coordinates (x, y, z) in meters
     */
    static Point3D imageToRobotFrame(
        const Point2D& point,
        const ImageSize& image_size,
        const PaperSize& paper_size,
        const Origin& origin
    );

    /**
     * @brief Create a Pose message from position and orientation
     * @param position 3D position (x, y, z)
     * @param orientation Optional quaternion (x, y, z, w), if nullptr uses origin_orientation
     * @param origin_orientation Euler angles (roll, pitch, yaw) used if orientation is nullptr
     * @return geometry_msgs::msg::Pose
     */
    static geometry_msgs::msg::Pose createPose(
        const Point3D& position,
        const std::array<double, 4>* orientation = nullptr,
        const Orientation* origin_orientation = nullptr
    );

    /**
     * @brief Calculate Euclidean distance between two poses
     * @param pose1 First pose
     * @param pose2 Second pose
     * @return Distance in meters
     */
    static double distanceBetweenPoses(
        const geometry_msgs::msg::Pose& pose1,
        const geometry_msgs::msg::Pose& pose2
    );

    /**
     * @brief Plan a Cartesian path through waypoints
     * @param move_group MoveGroupInterface instance
     * @param waypoints Vector of poses to follow
     * @param eef_step Step size between points (meters)
     * @param jump_threshold Maximum allowed jump between points (0=disabled)
     * @param avoid_collisions Whether to consider collisions
     * @return Pair of (fraction_planned, trajectory)
     */
    static std::pair<double, moveit_msgs::msg::RobotTrajectory> planCartesianPath(
        moveit::planning_interface::MoveGroupInterface& move_group,
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        double eef_step = 0.01,
        double jump_threshold = 0.0,
        bool avoid_collisions = true
    );

    /**
     * @brief Convert Euler angles to quaternion
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians  
     * @param yaw Yaw angle in radians
     * @return Quaternion as array [x, y, z, w]
     */
    static std::array<double, 4> eulerToQuaternion(double roll, double pitch, double yaw);

private:
    static constexpr double EPSILON = 1e-8;
};

} // namespace ur5_drawing