#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <memory>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace ur5_drawing {

/**
 * @brief Joint position type - 6 joint angles in radians
 */
using JointPosition = std::array<double, 6>;

/**
 * @brief 2D point in image coordinates
 */
using Point2D = std::array<double, 2>;

/**
 * @brief Image size [width, height]
 */
using ImageSize = std::array<int, 2>;

/**
 * @brief Corner positions for bilinear interpolation
 */
struct CornerPositions {
    JointPosition bottom_left;
    JointPosition bottom_right; 
    JointPosition top_left;
    JointPosition top_right;
};

/**
 * @brief Utility class for joint-based drawing operations
 */
class DrawingUtils {
public:
    /**
     * @brief Convert image coordinates to joint angles using bilinear interpolation
     * @param point Image coordinates (x, y) in pixels
     * @param image_size Image dimensions (width, height) in pixels
     * @param corners Corner joint positions for interpolation
     * @return Joint angles in radians
     */
    static JointPosition imageToJointSpace(
        const Point2D& point,
        const ImageSize& image_size,
        const CornerPositions& corners
    );

    /**
     * @brief Add pen lift offset to joint position
     * @param base_position Base joint position
     * @param lift_offset Joint offsets for pen lifting
     * @return Modified joint position with pen lifted
     */
    static JointPosition addPenLift(
        const JointPosition& base_position,
        const JointPosition& lift_offset
    );

    /**
     * @brief Interpolate between two joint positions
     * @param start_joints Starting joint position
     * @param end_joints Ending joint position
     * @param num_points Number of intermediate points to generate
     * @param method Interpolation method ("linear", "cubic", "quintic")
     * @return Vector of interpolated joint positions
     */
    static std::vector<JointPosition> interpolateJointPath(
        const JointPosition& start_joints,
        const JointPosition& end_joints,
        int num_points,
        const std::string& method = "linear"
    );

    /**
     * @brief Create joint trajectory message from joint positions
     * @param positions Vector of joint positions
     * @param joint_names Names of the joints
     * @param time_from_start Base time between waypoints
     * @param velocity_scaling Velocity scaling factor
     * @return Joint trajectory message
     */
    static trajectory_msgs::msg::JointTrajectory createJointTrajectory(
        const std::vector<JointPosition>& positions,
        const std::vector<std::string>& joint_names,
        double time_from_start = 2.0,
        double velocity_scaling = 0.3
    );

    /**
     * @brief Calculate joint space distance between two positions
     * @param pos1 First joint position
     * @param pos2 Second joint position
     * @return Joint space distance (sum of absolute joint angle differences)
     */
    static double jointSpaceDistance(
        const JointPosition& pos1,
        const JointPosition& pos2
    );

    /**
     * @brief Validate joint position limits
     * @param position Joint position to validate
     * @param min_limits Minimum joint limits
     * @param max_limits Maximum joint limits
     * @return True if position is within limits
     */
    static bool validateJointLimits(
        const JointPosition& position,
        const JointPosition& min_limits,
        const JointPosition& max_limits
    );

    /**
     * @brief Clamp joint position to limits
     * @param position Joint position to clamp
     * @param min_limits Minimum joint limits
     * @param max_limits Maximum joint limits
     * @return Clamped joint position
     */
    static JointPosition clampToJointLimits(
        const JointPosition& position,
        const JointPosition& min_limits,
        const JointPosition& max_limits
    );

    /**
     * @brief Get UR5 joint limits
     * @return Pair of (min_limits, max_limits) for UR5
     */
    static std::pair<JointPosition, JointPosition> getUR5JointLimits();
    
    /**
     * @brief Debug interpolation calculation for image point
     * @param point Image coordinates
     * @param image_size Image dimensions
     * @param corners Corner joint positions
     */
    static void debugInterpolation(
        const Point2D& point,
        const ImageSize& image_size,
        const CornerPositions& corners
    );

private:
    /**
     * @brief Bilinear interpolation between four corner values
     * @param bottom_left Bottom-left corner value
     * @param bottom_right Bottom-right corner value
     * @param top_left Top-left corner value
     * @param top_right Top-right corner value
     * @param u Horizontal interpolation parameter (0-1)
     * @param v Vertical interpolation parameter (0-1)
     * @return Interpolated value
     */
    static double bilinearInterpolate(
        double bottom_left, double bottom_right,
        double top_left, double top_right,
        double u, double v
    );

    /**
     * @brief Linear interpolation between two values
     * @param start Start value
     * @param end End value
     * @param t Interpolation parameter (0-1)
     * @return Interpolated value
     */
    static double lerp(double start, double end, double t);

    /**
     * @brief Cubic interpolation between two values with zero velocity endpoints
     * @param start Start value
     * @param end End value
     * @param t Interpolation parameter (0-1)
     * @return Interpolated value
     */
    static double cubicInterpolate(double start, double end, double t);

    /**
     * @brief Quintic interpolation between two values with zero velocity/acceleration endpoints
     * @param start Start value
     * @param end End value
     * @param t Interpolation parameter (0-1)
     * @return Interpolated value
     */
    static double quinticInterpolate(double start, double end, double t);

    static constexpr double EPSILON = 1e-8;
};

} // namespace ur5_drawing