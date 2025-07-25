#include "../include/drawing_utils.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace ur5_drawing {

JointPosition DrawingUtils::imageToJointSpace(
    const Point2D& point,
    const ImageSize& image_size,
    const CornerPositions& corners) {
    
    double x_img = point[0];
    double y_img = point[1];
    int img_width = image_size[0];
    int img_height = image_size[1];
    
    // Validate input data
    if (img_width <= 0 || img_height <= 0) {
        std::cerr << "ERROR: Invalid image dimensions: " << img_width << "x" << img_height << std::endl;
        
        // Return a default safe position rather than using uninitialized values
        JointPosition safe_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0}; // Common "home" position
        return safe_position;
    }
    
    // Check if corners are valid (no NaN or infinity values)
    for (int i = 0; i < 6; ++i) {
        if (std::isnan(corners.bottom_left[i]) || std::isinf(corners.bottom_left[i]) ||
            std::isnan(corners.bottom_right[i]) || std::isinf(corners.bottom_right[i]) ||
            std::isnan(corners.top_left[i]) || std::isinf(corners.top_left[i]) ||
            std::isnan(corners.top_right[i]) || std::isinf(corners.top_right[i])) {
            
            std::cerr << "ERROR: Invalid corner value detected at joint index " << i << std::endl;
            
            // Return a default safe position
            JointPosition safe_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
            return safe_position;
        }
    }
    
    // Normalize image coordinates (0-1)
    double u = x_img / static_cast<double>(img_width);
    double v = 1.0 - (y_img / static_cast<double>(img_height)); // Flip Y-axis
    
    // Clamp to valid range
    u = std::clamp(u, 0.0, 1.0);
    v = std::clamp(v, 0.0, 1.0);
    
    // Perform bilinear interpolation for each joint
    JointPosition result;
    for (int joint = 0; joint < 6; ++joint) {
        result[joint] = bilinearInterpolate(
            corners.bottom_left[joint],   // (0,0)
            corners.bottom_right[joint],  // (1,0)
            corners.top_left[joint],      // (0,1)
            corners.top_right[joint],     // (1,1)
            u, v
        );
        
        // Validate result to ensure no NaN or infinity values
        if (std::isnan(result[joint]) || std::isinf(result[joint])) {
            std::cerr << "WARNING: Calculated NaN or infinity for joint " << joint << std::endl;
            std::cerr << "  Using bottom_left value as fallback" << std::endl;
            result[joint] = corners.bottom_left[joint];
        }
    }
    
    return result;
}

JointPosition DrawingUtils::addPenLift(
    const JointPosition& base_position,
    const JointPosition& lift_offset) {
    
    JointPosition lifted_position;
    for (int i = 0; i < 6; ++i) {
        lifted_position[i] = base_position[i] + lift_offset[i];
    }
    return lifted_position;
}

std::vector<JointPosition> DrawingUtils::interpolateJointPath(
    const JointPosition& start_joints,
    const JointPosition& end_joints,
    int num_points,
    const std::string& method) {
    
    std::vector<JointPosition> path;
    
    if (num_points <= 1) {
        path.push_back(end_joints);
        return path;
    }
    
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_points);
        JointPosition interpolated_joints;
        
        for (int joint = 0; joint < 6; ++joint) {
            if (method == "cubic") {
                interpolated_joints[joint] = cubicInterpolate(
                    start_joints[joint], end_joints[joint], t);
            } else if (method == "quintic") {
                interpolated_joints[joint] = quinticInterpolate(
                    start_joints[joint], end_joints[joint], t);
            } else { // Default to linear
                interpolated_joints[joint] = lerp(
                    start_joints[joint], end_joints[joint], t);
            }
        }
        
        path.push_back(interpolated_joints);
    }
    
    return path;
}

trajectory_msgs::msg::JointTrajectory DrawingUtils::createJointTrajectory(
    const std::vector<JointPosition>& positions,
    const std::vector<std::string>& joint_names,
    double time_from_start,
    double velocity_scaling) {
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;
    
    for (size_t i = 0; i < positions.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        
        // Set positions
        for (int j = 0; j < 6; ++j) {
            point.positions.push_back(positions[i][j]);
        }
        
        // Set velocities (zero for waypoints, except smooth transitions)
        point.velocities.resize(6, 0.0);
        
        // Set accelerations (zero for waypoints)
        point.accelerations.resize(6, 0.0);
        
        // Set time from start
        double time_offset = time_from_start * static_cast<double>(i) / velocity_scaling;
        point.time_from_start = rclcpp::Duration::from_nanoseconds(
            static_cast<int64_t>(time_offset * 1e9));
        
        trajectory.points.push_back(point);
    }
    
    return trajectory;
}

double DrawingUtils::jointSpaceDistance(
    const JointPosition& pos1,
    const JointPosition& pos2) {
    
    double distance = 0.0;
    for (int i = 0; i < 6; ++i) {
        distance += std::abs(pos1[i] - pos2[i]);
    }
    return distance;
}

bool DrawingUtils::validateJointLimits(
    const JointPosition& position,
    const JointPosition& min_limits,
    const JointPosition& max_limits) {
    
    for (int i = 0; i < 6; ++i) {
        if (position[i] < min_limits[i] || position[i] > max_limits[i]) {
            return false;
        }
    }
    return true;
}

JointPosition DrawingUtils::clampToJointLimits(
    const JointPosition& position,
    const JointPosition& min_limits,
    const JointPosition& max_limits) {
    
    JointPosition clamped_position;
    for (int i = 0; i < 6; ++i) {
        clamped_position[i] = std::clamp(position[i], min_limits[i], max_limits[i]);
    }
    return clamped_position;
}

std::pair<JointPosition, JointPosition> DrawingUtils::getUR5JointLimits() {
    // UR5 joint limits in radians
    JointPosition min_limits = {-6.28, -6.28, -3.14, -6.28, -6.28, -6.28};
    JointPosition max_limits = {6.28, 6.28, 3.14, 6.28, 6.28, 6.28};
    
    return std::make_pair(min_limits, max_limits);
}

void DrawingUtils::debugInterpolation(
    const Point2D& point,
    const ImageSize& image_size,
    const CornerPositions& corners) {
    
    double x_img = point[0];
    double y_img = point[1];
    int img_width = image_size[0];
    int img_height = image_size[1];
    
    // Normalize image coordinates (0-1)
    double u = x_img / static_cast<double>(img_width);
    double v = 1.0 - (y_img / static_cast<double>(img_height)); // Flip Y-axis
    
    // Clamp to valid range
    u = std::clamp(u, 0.0, 1.0);
    v = std::clamp(v, 0.0, 1.0);
    
    std::cout << "===== DEBUG INTERPOLATION =====" << std::endl;
    std::cout << "Input point [" << x_img << ", " << y_img << "]" << std::endl;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;
    std::cout << "Normalized coordinates (u,v): [" << u << ", " << v << "]" << std::endl;
    std::cout << std::endl;
    
    // Compute and display interpolated values for each joint
    JointPosition result;
    std::cout << "Joint | Bottom Left | Bottom Right | Top Left | Top Right | Result" << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    
    for (int joint = 0; joint < 6; ++joint) {
        double bl = corners.bottom_left[joint];
        double br = corners.bottom_right[joint];
        double tl = corners.top_left[joint];
        double tr = corners.top_right[joint];
        
        // Calculate bottom and top interpolation first
        double bottom = lerp(bl, br, u);
        double top = lerp(tl, tr, u);
        
        // Then interpolate between bottom and top
        result[joint] = lerp(bottom, top, v);
        
        std::cout << "  " << joint << "  | " 
                  << std::fixed << std::setprecision(4) << bl << " | " 
                  << std::fixed << std::setprecision(4) << br << " | "
                  << std::fixed << std::setprecision(4) << tl << " | "
                  << std::fixed << std::setprecision(4) << tr << " | "
                  << std::fixed << std::setprecision(4) << result[joint] << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "Final joint positions (radians): [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(4) << result[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "Final joint positions (degrees): [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(2) << (result[i] * 180.0 / M_PI);
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    std::cout << "===============================" << std::endl;
}

// Private helper functions

double DrawingUtils::bilinearInterpolate(
    double bottom_left, double bottom_right,
    double top_left, double top_right,
    double u, double v) {
    
    // Interpolate along bottom edge
    double bottom = lerp(bottom_left, bottom_right, u);
    
    // Interpolate along top edge
    double top = lerp(top_left, top_right, u);
    
    // Interpolate between bottom and top
    return lerp(bottom, top, v);
}

double DrawingUtils::lerp(double start, double end, double t) {
    return start + t * (end - start);
}

double DrawingUtils::cubicInterpolate(double start, double end, double t) {
    // Cubic interpolation with zero velocity at endpoints
    // p(t) = a*t^3 + b*t^2 + c*t + d
    // With constraints: p(0)=start, p(1)=end, p'(0)=0, p'(1)=0
    
    double t2 = t * t;
    double t3 = t2 * t;
    
    // Coefficients for smooth cubic curve
    double a = 2.0 * start - 2.0 * end;
    double b = -3.0 * start + 3.0 * end;
    double c = 0.0;  // Zero initial velocity
    double d = start;
    
    return a * t3 + b * t2 + c * t + d;
}

double DrawingUtils::quinticInterpolate(double start, double end, double t) {
    // Quintic interpolation with zero velocity and acceleration at endpoints
    // p(t) = a*t^5 + b*t^4 + c*t^3 + d*t^2 + e*t + f
    // With constraints: p(0)=start, p(1)=end, p'(0)=0, p'(1)=0, p''(0)=0, p''(1)=0
    
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    
    // Coefficients for smooth quintic curve
    double a = 6.0 * start - 6.0 * end;
    double b = -15.0 * start + 15.0 * end;
    double c = 10.0 * start - 10.0 * end;
    double d = 0.0;  // Zero initial acceleration
    double e = 0.0;  // Zero initial velocity
    double f = start;
    
    return a * t5 + b * t4 + c * t3 + d * t2 + e * t + f;
}

} // namespace ur5_drawing