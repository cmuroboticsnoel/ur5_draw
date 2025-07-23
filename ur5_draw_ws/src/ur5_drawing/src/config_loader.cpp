#include "../include/config_loader.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>

namespace ur5_drawing {

DrawingConfig ConfigLoader::loadDrawingConfig(const std::string& config_path) {
    std::cout << "Loading drawing configuration from: " << config_path << std::endl;
    
    DrawingConfig config;
    
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
        
        // Load physical configuration
        if (yaml_config["physical"]) {
            const auto& phys = yaml_config["physical"];
            config.physical.origin = extractVector(phys, "origin", config.physical.origin);
            config.physical.paper_width = safeExtract<double>(phys, "paper_width", config.physical.paper_width);
            config.physical.paper_height = safeExtract<double>(phys, "paper_height", config.physical.paper_height);
            config.physical.lift_offset = safeExtract<double>(phys, "lift_offset", config.physical.lift_offset);
            config.physical.drawing_speed = safeExtract<double>(phys, "drawing_speed", config.physical.drawing_speed);
            config.physical.start_position_pose = extractVector(phys, "start_position_pose", config.physical.start_position_pose);
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
            config.planning.goal_position_tolerance = safeExtract<double>(plan, "goal_position_tolerance", config.planning.goal_position_tolerance);
            config.planning.goal_orientation_tolerance = safeExtract<double>(plan, "goal_orientation_tolerance", config.planning.goal_orientation_tolerance);
        }
        
        // Load cartesian configuration
        if (yaml_config["cartesian"]) {
            const auto& cart = yaml_config["cartesian"];
            config.cartesian.eef_step = safeExtract<double>(cart, "eef_step", config.cartesian.eef_step);
            config.cartesian.jump_threshold = safeExtract<double>(cart, "jump_threshold", config.cartesian.jump_threshold);
            config.cartesian.avoid_collisions = safeExtract<bool>(cart, "avoid_collisions", config.cartesian.avoid_collisions);
        }
        
        // Load safety configuration
        if (yaml_config["safety"]) {
            const auto& safety = yaml_config["safety"];
            config.safety.max_velocity = safeExtract<double>(safety, "max_velocity", config.safety.max_velocity);
            config.safety.max_acceleration = safeExtract<double>(safety, "max_acceleration", config.safety.max_acceleration);
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
        
        std::cout << "✓ Loaded drawing configuration from: " << config_path << std::endl;
        
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

DrawingConfig ConfigLoader::getDefaultConfig() {
    return DrawingConfig{}; // Uses default values from struct initialization
}

void ConfigLoader::printConfigSummary(const DrawingConfig& config) {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "UR5 DRAWING CONFIGURATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Robot Type: " << config.network.ur_type << std::endl;
    std::cout << "Robot IP: " << config.network.robot_ip << std::endl;
    std::cout << "PC IP: " << config.network.pc_ip << std::endl;
    std::cout << "Use Fake Hardware: " << (config.launch.use_fake_hardware ? "true" : "false") << std::endl;
    std::cout << "Paper Size: " << config.physical.paper_width << "m × " << config.physical.paper_height << "m" << std::endl;
    std::cout << "Drawing Speed: " << config.physical.drawing_speed << " m/s" << std::endl;
    std::cout << "Pen Lift Height: " << (config.physical.lift_offset * 1000.0) << " mm" << std::endl;
    std::cout << "Planning Group: " << config.planning.planning_group << std::endl;
    std::cout << std::string(60, '=') << "\n" << std::endl;
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