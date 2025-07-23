#pragma once

#include <string>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>

namespace ur5_drawing {

/**
 * @brief Configuration structure for the drawing system
 */
struct DrawingConfig {
    // Network configuration
    struct Network {
        std::string robot_ip = "192.168.1.102";
        std::string pc_ip = "192.168.1.101";
        std::string ur_type = "ur5";
    } network;

    // Physical configuration
    struct Physical {
        std::vector<double> origin = {-0.517, -0.345, 0.032, 1.26, -2.928, 0};  // Cartesian pose
        double paper_width = 0.2794;
        double paper_height = 0.2159;
        double lift_offset = 0.02;
        double drawing_speed = 0.25;
        std::vector<double> start_position_pose = {-0.345, -0.517, 0.032, 1.26, -2.928, 0};  // Cartesian pose
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
        double goal_position_tolerance = 0.001;
        double goal_orientation_tolerance = 0.01;
    } planning;

    // Cartesian planning parameters
    struct Cartesian {
        double eef_step = 0.005;
        double jump_threshold = 0.0;
        bool avoid_collisions = true;
    } cartesian;

    // Safety parameters
    struct Safety {
        double max_velocity = 0.5;
        double max_acceleration = 1.0;
    } safety;

    // Collision parameters
    struct Collision {
        double object_padding = 0.01;
    } collision;

    // File paths
    struct Files {
        std::string drawing_sequences = "drawing_sequences.json";
        std::string rviz_config = "drawing.rviz";
        std::string calibration_file = "calibration.yaml";
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
 * @brief Configuration loader for YAML files
 */
class ConfigLoader {
public:
    /**
     * @brief Load configuration from YAML file
     * @param config_path Path to the configuration file
     * @return DrawingConfig structure with loaded values
     */
    static DrawingConfig loadDrawingConfig(const std::string& config_path);

    /**
     * @brief Get default configuration
     * @return Default DrawingConfig structure
     */
    static DrawingConfig getDefaultConfig();

    /**
     * @brief Print configuration summary
     * @param config Configuration to print
     */
    static void printConfigSummary(const DrawingConfig& config);

private:
    /**
     * @brief Safely extract value from YAML node with default fallback
     */
    template<typename T>
    static T safeExtract(const YAML::Node& node, const std::string& key, const T& default_value);

    /**
     * @brief Extract vector from YAML node
     */
    static std::vector<double> extractVector(const YAML::Node& node, const std::string& key, 
                                           const std::vector<double>& default_value);
};

} // namespace ur5_drawing