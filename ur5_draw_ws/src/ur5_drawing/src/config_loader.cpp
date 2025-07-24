#include "../include/config_loader.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

namespace ur5_drawing {

JointDrawingConfig ConfigLoader::loadJointDrawingConfig(const std::string& config_path) {
    std::cout << "Loading joint-based drawing configuration from: " << config_path << std::endl;
    
    JointDrawingConfig config;
    
    try {
        if (!std::filesystem::exists(config_path)) {
            std::cout << "✗ Warning: Configuration file not found: " << config_path << std::endl;
            std::cout << "  Using default configuration values." << std::endl;
            return getDefaultConfig();
        }
        
        YAML::Node yaml_config = YAML::LoadFile(config_path);
        
        // Load network configuration
        if (yaml_config["network"]) {
            const auto& net = yaml_config["network"];
            config.network.robot_ip = safeExtract<std::string>(net, "robot_ip", config.network.robot_ip);
            config.network.pc_ip = safeExtract<std::string>(net, "pc_ip", config.network.pc_ip);
            config.network.ur_type = safeExtract<std::string>(net, "ur_type", config.network.ur_type);
        }
        
        // Load joint calibration configuration
        if (yaml_config["joint_calibration"]) {
            const auto& joint_cal = yaml_config["joint_calibration"];
            
            config.joint_calibration.home_position = extractJointPosition(
                joint_cal, "home_position", config.joint_calibration.home_position);
            config.joint_calibration.origin_position = extractJointPosition(
                joint_cal, "origin_position", config.joint_calibration.origin_position);
            config.joint_calibration.lift_offset_joints = extractJointPosition(
                joint_cal, "lift_offset_joints", config.joint_calibration.lift_offset_joints);
            
            // Load corner positions
            if (joint_cal["corners"]) {
                const auto& corners = joint_cal["corners"];
                config.joint_calibration.corners.bottom_left = extractJointPosition(
                    corners, "bottom_left", config.joint_calibration.corners.bottom_left);
                config.joint_calibration.corners.bottom_right = extractJointPosition(
                    corners, "bottom_right", config.joint_calibration.corners.bottom_right);
                config.joint_calibration.corners.top_left = extractJointPosition(
                    corners, "top_left", config.joint_calibration.corners.top_left);
                config.joint_calibration.corners.top_right = extractJointPosition(
                    corners, "top_right", config.joint_calibration.corners.top_right);
            }
        }
        
        // Load physical configuration
        if (yaml_config["physical"]) {
            const auto& phys = yaml_config["physical"];
            config.physical.paper_width = safeExtract<double>(phys, "paper_width", config.physical.paper_width);
            config.physical.paper_height = safeExtract<double>(phys, "paper_height", config.physical.paper_height);
            config.physical.drawing_speed = safeExtract<double>(phys, "drawing_speed", config.physical.drawing_speed);
        }
        
        // Load image configuration
        if (yaml_config["image"]) {
            const auto& img = yaml_config["image"];
            config.image.width = safeExtract<int>(img, "width", config.image.width);
            config.image.height = safeExtract<int>(img, "height", config.image.height);
        }
        
        // Load planning configuration
        if (yaml_config["planning"]) {
            const auto& plan = yaml_config["planning"];
            config.planning.planning_group = safeExtract<std::string>(plan, "planning_group", config.planning.planning_group);
            config.planning.max_planning_attempts = safeExtract<int>(plan, "max_planning_attempts", config.planning.max_planning_attempts);
            config.planning.planning_time = safeExtract<double>(plan, "planning_time", config.planning.planning_time);
            config.planning.replan_attempts = safeExtract<int>(plan, "replan_attempts", config.planning.replan_attempts);
            config.planning.goal_joint_tolerance = safeExtract<double>(plan, "goal_joint_tolerance", config.planning.goal_joint_tolerance);
            config.planning.goal_position_tolerance = safeExtract<double>(plan, "goal_position_tolerance", config.planning.goal_position_tolerance);
            config.planning.goal_orientation_tolerance = safeExtract<double>(plan, "goal_orientation_tolerance", config.planning.goal_orientation_tolerance);
        }
        
        // Load joint trajectory configuration
        if (yaml_config["joint_trajectory"]) {
            const auto& joint_traj = yaml_config["joint_trajectory"];
            config.joint_trajectory.max_velocity_scaling = safeExtract<double>(joint_traj, "max_velocity_scaling", config.joint_trajectory.max_velocity_scaling);
            config.joint_trajectory.max_acceleration_scaling = safeExtract<double>(joint_traj, "max_acceleration_scaling", config.joint_trajectory.max_acceleration_scaling);
            config.joint_trajectory.joint_velocity_limits = extractVector(joint_traj, "joint_velocity_limits", config.joint_trajectory.joint_velocity_limits);
            config.joint_trajectory.time_from_start = safeExtract<double>(joint_traj, "time_from_start", config.joint_trajectory.time_from_start);
        }
        
        // Load interpolation configuration
        if (yaml_config["interpolation"]) {
            const auto& interp = yaml_config["interpolation"];
            config.interpolation.method = safeExtract<std::string>(interp, "method", config.interpolation.method);
            config.interpolation.interpolation_points = safeExtract<int>(interp, "interpolation_points", config.interpolation.interpolation_points);
        }
        
        // Load safety configuration
        if (yaml_config["safety"]) {
            const auto& safety = yaml_config["safety"];
            config.safety.max_velocity = safeExtract<double>(safety, "max_velocity", config.safety.max_velocity);
            config.safety.max_acceleration = safeExtract<double>(safety, "max_acceleration", config.safety.max_acceleration);
            config.safety.emergency_stop_deceleration = safeExtract<double>(safety, "emergency_stop_deceleration", config.safety.emergency_stop_deceleration);
        }
        
        // Load collision configuration
        if (yaml_config["collision"]) {
            const auto& coll = yaml_config["collision"];
            config.collision.object_padding = safeExtract<double>(coll, "object_padding", config.collision.object_padding);
        }
        
        // Load files configuration
        if (yaml_config["files"]) {
            const auto& files = yaml_config["files"];
            config.files.drawing_sequences = safeExtract<std::string>(files, "drawing_sequences", config.files.drawing_sequences);
            config.files.rviz_config = safeExtract<std::string>(files, "rviz_config", config.files.rviz_config);
            config.files.calibration_file = safeExtract<std::string>(files, "calibration_file", config.files.calibration_file);
            config.files.joint_calibration_file = safeExtract<std::string>(files, "joint_calibration_file", config.files.joint_calibration_file);
        }
        
        // Load launch configuration
        if (yaml_config["launch"]) {
            const auto& launch = yaml_config["launch"];
            config.launch.use_fake_hardware = safeExtract<bool>(launch, "use_fake_hardware", config.launch.use_fake_hardware);
            config.launch.headless_mode = safeExtract<bool>(launch, "headless_mode", config.launch.headless_mode);
            config.launch.initial_joint_controller = safeExtract<std::string>(launch, "initial_joint_controller", config.launch.initial_joint_controller);
            config.launch.default_use_rviz = safeExtract<bool>(launch, "default_use_rviz", config.launch.default_use_rviz);
            config.launch.default_use_sim_time = safeExtract<bool>(launch, "default_use_sim_time", config.launch.default_use_sim_time);
        }
        
        std::cout << "✓ Loaded joint-based drawing configuration from: " << config_path << std::endl;
        
    } catch (const YAML::Exception& e) {
        std::cout << "✗ Error parsing YAML configuration: " << e.what() << std::endl;
        std::cout << "  Using default configuration values." << std::endl;
        return getDefaultConfig();
    } catch (const std::exception& e) {
        std::cout << "✗ Error loading configuration: " << e.what() << std::endl;
        std::cout << "  Using default configuration values." << std::endl;
        return getDefaultConfig();
    }
    
    return config;
}

JointDrawingConfig ConfigLoader::getDefaultConfig() {
    return JointDrawingConfig{}; // Uses default values from struct initialization
}

void ConfigLoader::printConfigSummary(const JointDrawingConfig& config) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "UR5 JOINT-BASED DRAWING CONFIGURATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Robot Type: " << config.network.ur_type << std::endl;
    std::cout << "Robot IP: " << config.network.robot_ip << std::endl;
    std::cout << "PC IP: " << config.network.pc_ip << std::endl;
    std::cout << "Use Fake Hardware: " << (config.launch.use_fake_hardware ? "true" : "false") << std::endl;
    std::cout << "Paper Size: " << config.physical.paper_width << "m × " << config.physical.paper_height << "m" << std::endl;
    std::cout << "Drawing Speed: " << config.physical.drawing_speed << std::endl;
    std::cout << "Planning Group: " << config.planning.planning_group << std::endl;
    std::cout << "Interpolation Method: " << config.interpolation.method << std::endl;
    std::cout << "Interpolation Points: " << config.interpolation.interpolation_points << std::endl;
    
    // Print joint calibration summary  
    std::cout << "\nJoint Calibration Summary:" << std::endl;
    std::cout << "Home Position: [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(3) << config.joint_calibration.home_position[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "Origin Position: [";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::fixed << std::setprecision(3) << config.joint_calibration.origin_position[i];
        if (i < 5) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << std::string(60, '=') << "\n" << std::endl;
}

void ConfigLoader::saveJointCalibration(const JointDrawingConfig& config, const std::string& file_path) {
    try {
        YAML::Node yaml_config;
        
        // Save joint calibration data
        YAML::Node joint_cal;
        
        // Home position
        YAML::Node home_node;
        for (int i = 0; i < 6; ++i) {
            home_node.push_back(config.joint_calibration.home_position[i]);
        }
        joint_cal["home_position"] = home_node;
        
        // Origin position
        YAML::Node origin_node;
        for (int i = 0; i < 6; ++i) {
            origin_node.push_back(config.joint_calibration.origin_position[i]);
        }
        joint_cal["origin_position"] = origin_node;
        
        // Corner positions
        YAML::Node corners_node;
        
        YAML::Node bl_node, br_node, tl_node, tr_node;
        for (int i = 0; i < 6; ++i) {
            bl_node.push_back(config.joint_calibration.corners.bottom_left[i]);
            br_node.push_back(config.joint_calibration.corners.bottom_right[i]);
            tl_node.push_back(config.joint_calibration.corners.top_left[i]);
            tr_node.push_back(config.joint_calibration.corners.top_right[i]);
        }
        
        corners_node["bottom_left"] = bl_node;
        corners_node["bottom_right"] = br_node;
        corners_node["top_left"] = tl_node;
        corners_node["top_right"] = tr_node;
        joint_cal["corners"] = corners_node;
        
        // Lift offset
        YAML::Node lift_node;
        for (int i = 0; i < 6; ++i) {
            lift_node.push_back(config.joint_calibration.lift_offset_joints[i]);
        }
        joint_cal["lift_offset_joints"] = lift_node;
        
        yaml_config["joint_calibration"] = joint_cal;
        
        // Write to file
        std::ofstream file(file_path);
        file << yaml_config;
        file.close();
        
        std::cout << "Joint calibration saved to: " << file_path << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to save joint calibration: " << e.what() << std::endl;
        throw;
    }
}

JointDrawingConfig::JointCalibration ConfigLoader::loadJointCalibration(const std::string& file_path) {
    JointDrawingConfig::JointCalibration calibration;
    
    try {
        if (!std::filesystem::exists(file_path)) {
            std::cout << "Joint calibration file not found: " << file_path << std::endl;
            std::cout << "Using default calibration values." << std::endl;
            return calibration;
        }
        
        YAML::Node yaml_config = YAML::LoadFile(file_path);
        
        if (yaml_config["joint_calibration"]) {
            const auto& joint_cal = yaml_config["joint_calibration"];
            
            calibration.home_position = extractJointPosition(
                joint_cal, "home_position", calibration.home_position);
            calibration.origin_position = extractJointPosition(
                joint_cal, "origin_position", calibration.origin_position);
            calibration.lift_offset_joints = extractJointPosition(
                joint_cal, "lift_offset_joints", calibration.lift_offset_joints);
            
            if (joint_cal["corners"]) {
                const auto& corners = joint_cal["corners"];
                calibration.corners.bottom_left = extractJointPosition(
                    corners, "bottom_left", calibration.corners.bottom_left);
                calibration.corners.bottom_right = extractJointPosition(
                    corners, "bottom_right", calibration.corners.bottom_right);
                calibration.corners.top_left = extractJointPosition(
                    corners, "top_left", calibration.corners.top_left);
                calibration.corners.top_right = extractJointPosition(
                    corners, "top_right", calibration.corners.top_right);
            }
        }
        
        std::cout << "✓ Loaded joint calibration from: " << file_path << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "✗ Error loading joint calibration: " << e.what() << std::endl;
        std::cout << "  Using default calibration values." << std::endl;
    }
    
    return calibration;
}

template<typename T>
T ConfigLoader::safeExtract(const YAML::Node& node, const std::string& key, const T& default_value) {
    try {
        if (node[key]) {
            return node[key].as<T>();
        }
    } catch (const YAML::Exception& e) {
        std::cout << "Warning: Failed to extract '" << key << "': " << e.what() << ". Using default." << std::endl;
    }
    return default_value;
}

JointPosition ConfigLoader::extractJointPosition(const YAML::Node& node, const std::string& key, 
                                                     const JointPosition& default_value) {
    try {
        if (node[key] && node[key].IsSequence()) {
            JointPosition result = default_value;
            auto sequence = node[key];
            
            for (size_t i = 0; i < std::min(sequence.size(), static_cast<size_t>(6)); ++i) {
                result[i] = sequence[i].as<double>();
            }
            
            return result;
        }
    } catch (const YAML::Exception& e) {
        std::cout << "Warning: Failed to extract joint position '" << key << "': " << e.what() << ". Using default." << std::endl;
    }
    return default_value;
}

std::vector<double> ConfigLoader::extractVector(const YAML::Node& node, const std::string& key, 
                                                    const std::vector<double>& default_value) {
    try {
        if (node[key] && node[key].IsSequence()) {
            std::vector<double> result;
            for (const auto& item : node[key]) {
                result.push_back(item.as<double>());
            }
            return result;
        }
    } catch (const YAML::Exception& e) {
        std::cout << "Warning: Failed to extract vector '" << key << "': " << e.what() << ". Using default." << std::endl;
    }
    return default_value;
}

} // namespace ur5_drawing