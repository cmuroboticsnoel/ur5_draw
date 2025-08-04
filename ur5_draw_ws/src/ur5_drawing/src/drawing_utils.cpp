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

std::pair<JointPosition, JointPosition> DrawingUtils::getUR5JointLimits() {
    // UR5 joint limits in radians
    JointPosition min_limits = {-6.28, -6.28, -3.14, -6.28, -6.28, -6.28};
    JointPosition max_limits = {6.28, 6.28, 3.14, 6.28, 6.28, 6.28};
    
    return std::make_pair(min_limits, max_limits);
}