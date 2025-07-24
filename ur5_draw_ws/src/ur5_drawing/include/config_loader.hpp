#pragma once

#include <string>
#include <vector>
#include <map>
#include <array>
#include <yaml-cpp/yaml.h>

namespace ur5_drawing {

/**
 * @brief Joint position type - 6 joint angles in radians
 */
using JointPosition = std::array<double, 6>;

/**
 * @brief Configuration structure for joint-based drawing system
 */
struct JointDrawingConfig {
    // Network configuration
    struct Network {
        std::string robot_ip = "192.168.1.102";
        std::string pc_ip = "192.168.1.101";
        std::string ur_type = "ur5";
    } network;

    // Joint calibration configuration
    struct JointCalibration {
        JointPosition home_position = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
        JointPosition origin_position = {0.0, -1.2, 0.5, -1.57, 0.0, 0.0};
        
        struct Corners {
            JointPosition bottom_left = {-0.3, -1.1, 0.4, -1.5, 0.1, 0.0};
            JointPosition bottom_right = {0.3, -1.1, 0.4, -1.5, -0.1, 0.0};
            JointPosition top_left = {-0.3, -1.3, 0.6, -1.6, 0.1, 0.0};
            JointPosition top_right = {0.3, -1.3, 0.6, -1.6, -0.1, 0.0};
        } corners;
        
        JointPosition lift_offset_joints = {0.0, -0.05, 0.0, -0.05, 0.0, 0.0};
    } joint_calibration;

    // Physical configuration
    struct Physical {
        double paper_width = 0.2794;
        double paper_height = 0.2159;
        double drawing_speed = 0.25;
    } physical;

    // Image configuration
    struct Image {
        int width = 800;
        int height = 600;
    } image;

    // Planning configuration
    struct Planning {
        std::string planning_group = "ur_manipulator";
        int max_planning_attempts = 5;
        double planning_time = 5.0;
        int replan_attempts = 3;
        double goal_joint_tolerance = 0.01;
        double goal_position_tolerance = 0.001;
        double goal_orientation_tolerance = 0.01;
    } planning;

    // Joint trajectory parameters
    struct JointTrajectory {
        double max_velocity_scaling = 0.3;
        double max_acceleration_scaling = 0.2;
        std::vector<double> joint_velocity_limits = {1.0, 1.0, 1.5, 2.0, 2.0, 2.0};
        double time_from_start = 2.0;
    } joint_trajectory;

    // Interpolation parameters
    struct Interpolation {
        std::string method = "linear";
        int interpolation_points = 10;
    } interpolation;

    // Safety parameters
    struct Safety {
        double max_velocity = 0.5;
        double max_acceleration = 1.0;
        double emergency_stop_deceleration = 3.0;
    } safety;

    // Collision parameters
    struct Collision {
        double object_padding = 0.01;
    } collision;

    // File paths
    struct Files {
        std::string drawing_sequences = "image_description.json";
        std::string rviz_config = "drawing.rviz";
        std::string calibration_file = "calibration.yaml";
        std::string joint_calibration_file = "joint_calibration.yaml";
    } files;

    // Launch configuration
    struct Launch {
        bool use_fake_hardware = false;
        bool headless_mode = true;
        std::string initial_joint_controller = "scaled_joint_trajectory_controller";
        bool default_use_rviz = true;
        bool default_use_sim_time = false;
    } launch;
};

/**
 * @brief Configuration loader for joint-based YAML files
 */
class ConfigLoader {
public:
    /**
     * @brief Load joint-based configuration from YAML file
     * @param config_path Path to the configuration file
     * @return JointDrawingConfig structure with loaded values
     */
    static JointDrawingConfig loadJointDrawingConfig(const std::string& config_path);

    /**
     * @brief Get default joint-based configuration
     * @return Default JointDrawingConfig structure
     */
    static JointDrawingConfig getDefaultConfig();

    /**
     * @brief Print configuration summary
     * @param config Configuration to print
     */
    static void printConfigSummary(const JointDrawingConfig& config);

    /**
     * @brief Save joint calibration to file
     * @param config Configuration to save
     * @param file_path Path to save calibration file
     */
    static void saveJointCalibration(const JointDrawingConfig& config, const std::string& file_path);

    /**
     * @brief Load joint calibration from file
     * @param file_path Path to calibration file
     * @return Loaded joint calibration or default if failed
     */
    static JointDrawingConfig::JointCalibration loadJointCalibration(const std::string& file_path);

private:
    /**
     * @brief Safely extract value from YAML node with default fallback
     */
    template<typename T>
    static T safeExtract(const YAML::Node& node, const std::string& key, const T& default_value);

    /**
     * @brief Extract joint position from YAML node
     */
    static JointPosition extractJointPosition(const YAML::Node& node, const std::string& key, 
                                            const JointPosition& default_value);

    /**
     * @brief Extract vector from YAML node
     */
    static std::vector<double> extractVector(const YAML::Node& node, const std::string& key, 
                                           const std::vector<double>& default_value);
};

} // namespace ur5_drawing