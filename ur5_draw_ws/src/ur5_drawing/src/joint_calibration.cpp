#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp> // Using Trigger service for simplicity
#include <iostream>
#include <string>
#include <array>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief Interactive joint calibration tool for UR5 drawing system
 *
 * This tool helps you calibrate the joint positions for:
 * 1. Home position (pen lifted above paper center)
 * 2. Origin position (pen touching paper center)
 * 3. Four corner positions (pen touching each corner)
 *
 * Usage:
 * 1. Start the UR5 robot system
 * 2. Run this calibration tool
 * 3. Manually move the robot to each required position.
 * 4. Call the corresponding ROS 2 service to save each position.
 * 5. The tool will save the calibration to a YAML file.
 */

using JointPosition = std::array<double, 6>;

// Forward declaration of calibration data structure
struct CalibrationData {
    JointPosition home_position;
    JointPosition origin_position;
    JointPosition bottom_left;
    JointPosition bottom_right;
    JointPosition top_left;
    JointPosition top_right;
    JointPosition lift_offset;
};

class JointCalibration : public rclcpp::Node {
public:
    JointCalibration() : Node("joint_calibration") {
        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointCalibration::jointStateCallback, this, std::placeholders::_1)
        );
        
        // Service client for emergency stop (if needed)
        emergency_stop_client_ = this->create_client<std_srvs::srv::Trigger>("/ur_hardware_interface/dashboard_client/stop");

        // Create service servers for each calibration step
        home_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/home_position",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "HOME_POSITION")
        );
        origin_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/origin_position",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "ORIGIN_POSITION")
        );
        bottom_left_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/bottom_left",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "BOTTOM_LEFT")
        );
        bottom_right_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/bottom_right",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "BOTTOM_RIGHT")
        );
        top_left_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/top_left",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "TOP_LEFT")
        );
        top_right_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/top_right",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "TOP_RIGHT")
        );
        save_cal_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_all",
            std::bind(&JointCalibration::handleCalibrationService, this, std::placeholders::_1, std::placeholders::_2, "SAVE_ALL")
        );
        
        RCLCPP_INFO(this->get_logger(), "Joint Calibration node initialized. Waiting for joint states and service calls...");
        RCLCPP_INFO(this->get_logger(), "Available services: calibrate_home_position, calibrate_origin_position, etc. and save_all_calibration");
    }

private:
    std::vector<std::string> joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_stop_client_;
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;

    // Service servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr origin_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bottom_left_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr bottom_right_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr top_left_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr top_right_cal_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_cal_service_;

    CalibrationData calibration_data_; // Store calibration data within the class

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        current_joint_state_ = msg;

        // Temporary array to hold reordered positions
        std::array<double, 6> reordered_positions;
        bool all_found = true;

        // Find each joint in received message and reorder
        for (size_t i = 0; i < joint_names_.size(); i++) {
            const std::string& target_joint = joint_names_[i];
            auto it = std::find(msg->name.begin(), msg->name.end(), target_joint);
            
            if (it != msg->name.end()) {
                size_t index = std::distance(msg->name.begin(), it);
                if (index < msg->position.size()) {
                    reordered_positions[i] = msg->position[index];
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Position data missing for joint '%s'", target_joint.c_str());
                    all_found = false;
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Joint '%s' not found in joint state message", target_joint.c_str());
                all_found = false;
            }
        }

        // Only update current position if all joints were found with valid data
        if (all_found) {
            current_joint_state_->name = joint_names_;
            current_joint_state_->set__position(std::vector<double>(reordered_positions.begin(), reordered_positions.end()));
        }
    }

    void handleCalibrationService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response,
        const std::string& step_name)
    {
        (void)request; // Suppress unused parameter warning

        if (!current_joint_state_ || current_joint_state_->position.size() < 6) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive joint states. Cannot perform calibration step %s.", step_name.c_str());
            response->success = false;
            response->message = "Failed to get current joint states.";
            return;
        }

        JointPosition current_pos;
        for (int i = 0; i < 6; ++i) {
            current_pos[i] = current_joint_state_->position[i];
        }

        std::stringstream ss_angles;
        ss_angles << "Current joint angles (degrees): [";
        for (size_t i = 0; i < 6; ++i) {
            double angle_deg = current_joint_state_->position[i] * 180.0 / M_PI;
            ss_angles << std::fixed << std::setprecision(1) << angle_deg;
            if (i < 5) ss_angles << ", ";
        }
        ss_angles << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss_angles.str().c_str());

        if (step_name == "HOME_POSITION") {
            calibration_data_.home_position = current_pos;
            RCLCPP_INFO(this->get_logger(), "HOME POSITION saved.");
        } else if (step_name == "ORIGIN_POSITION") {
            calibration_data_.origin_position = current_pos;
            RCLCPP_INFO(this->get_logger(), "ORIGIN POSITION saved.");
        } else if (step_name == "BOTTOM_LEFT") {
            calibration_data_.bottom_left = current_pos;
            RCLCPP_INFO(this->get_logger(), "BOTTOM-LEFT CORNER saved.");
        } else if (step_name == "BOTTOM_RIGHT") {
            calibration_data_.bottom_right = current_pos;
            RCLCPP_INFO(this->get_logger(), "BOTTOM-RIGHT CORNER saved.");
        } else if (step_name == "TOP_LEFT") {
            calibration_data_.top_left = current_pos;
            RCLCPP_INFO(this->get_logger(), "TOP-LEFT CORNER saved.");
        } else if (step_name == "TOP_RIGHT") {
            calibration_data_.top_right = current_pos;
            RCLCPP_INFO(this->get_logger(), "TOP-RIGHT CORNER saved.");
        } else if (step_name == "SAVE_ALL") {
            // Calculate lift offset before saving all
            RCLCPP_INFO(this->get_logger(), "\n=== CALCULATING PEN LIFT OFFSET ===");
            RCLCPP_INFO(this->get_logger(), "The lift offset will be calculated as the difference between");
            RCLCPP_INFO(this->get_logger(), "the home position and origin position.\n");
            
            for (int i = 0; i < 6; ++i) {
                calibration_data_.lift_offset[i] = calibration_data_.home_position[i] - calibration_data_.origin_position[i];
            }

            // Display calculated offset
            std::stringstream ss_offset;
            ss_offset << "Calculated lift offset (radians): [";
            for (int i = 0; i < 6; ++i) {
                ss_offset << std::fixed << std::setprecision(4) << calibration_data_.lift_offset[i];
                if (i < 5) ss_offset << ", ";
            }
            ss_offset << "]";
            RCLCPP_INFO(this->get_logger(), "%s", ss_offset.str().c_str());

            // Save all calibration data
            saveCalibration(calibration_data_);
            RCLCPP_INFO(this->get_logger(), "\n=== CALIBRATION COMPLETE ===");
            RCLCPP_INFO(this->get_logger(), "All calibration data saved successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown calibration step requested: %s", step_name.c_str());
            response->success = false;
            response->message = "Unknown calibration step.";
            return;
        }

        response->success = true;
        response->message = "Position for " + step_name + " saved successfully.";
    }

    void saveCalibration(const CalibrationData& calibration) {
        std::string calibration_file = std::string(getenv("HOME")) + 
            "/ur5_draw/ur5_draw_ws/src/ur5_drawing/config/joint_calibration.yaml";

        try {
            YAML::Node yaml_config;
            
            // Save home position
            YAML::Node home_node;
            for (int i = 0; i < 6; ++i) {
                home_node.push_back(calibration.home_position[i]);
            }
            yaml_config["joint_calibration"]["home_position"] = home_node;
            
            // Save origin position
            YAML::Node origin_node;
            for (int i = 0; i < 6; ++i) {
                origin_node.push_back(calibration.origin_position[i]);
            }
            yaml_config["joint_calibration"]["origin_position"] = origin_node;
            
            // Save corner positions
            YAML::Node corners_node;
            
            YAML::Node bl_node, br_node, tl_node, tr_node;
            for (int i = 0; i < 6; ++i) {
                bl_node.push_back(calibration.bottom_left[i]);
                br_node.push_back(calibration.bottom_right[i]);
                tl_node.push_back(calibration.top_left[i]);
                tr_node.push_back(calibration.top_right[i]);
            }
            
            corners_node["bottom_left"] = bl_node;
            corners_node["bottom_right"] = br_node;
            corners_node["top_left"] = tl_node;
            corners_node["top_right"] = tr_node;
            yaml_config["joint_calibration"]["corners"] = corners_node;
            
            // Save lift offset
            YAML::Node lift_node;
            for (int i = 0; i < 6; ++i) {
                lift_node.push_back(calibration.lift_offset[i]);
            }
            yaml_config["joint_calibration"]["lift_offset_joints"] = lift_node;
            
            // Write to file
            std::ofstream file(calibration_file);
            if (!file.is_open()) {
                throw std::runtime_error("Could not open calibration file for writing: " + calibration_file);
            }
            
            file << yaml_config;
            file.close();
            
            RCLCPP_INFO(this->get_logger(), "Calibration saved to: %s", calibration_file.c_str());
            
            // Also update the main config file
            updateMainConfig(calibration);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save calibration: %s", e.what());
        }
    }

    void updateMainConfig(const CalibrationData& calibration) {
        std::string main_config_file = std::string(getenv("HOME")) + 
            "/ur5_draw/ur5_draw_ws/src/ur5_drawing/config/drawing_config.yaml";

        try {
            YAML::Node yaml_config;
            
            // Try to load existing config
            if (std::ifstream(main_config_file)) {
                yaml_config = YAML::LoadFile(main_config_file);
            }
            
            // Update joint calibration section
            YAML::Node joint_cal;
            
            // Home position
            YAML::Node home_node;
            for (int i = 0; i < 6; ++i) {
                home_node.push_back(calibration.home_position[i]);
            }
            joint_cal["home_position"] = home_node;
            
            // Origin position
            YAML::Node origin_node;
            for (int i = 0; i < 6; ++i) {
                origin_node.push_back(calibration.origin_position[i]);
            }
            joint_cal["origin_position"] = origin_node;
            
            // Corners
            YAML::Node corners;
            YAML::Node bl, br, tl, tr;
            for (int i = 0; i < 6; ++i) {
                bl.push_back(calibration.bottom_left[i]);
                br.push_back(calibration.bottom_right[i]);
                tl.push_back(calibration.top_left[i]);
                tr.push_back(calibration.top_right[i]);
            }
            corners["bottom_left"] = bl;
            corners["bottom_right"] = br;
            corners["top_left"] = tl;
            corners["top_right"] = tr;
            joint_cal["corners"] = corners;
            
            // Lift offset
            YAML::Node lift;
            for (int i = 0; i < 6; ++i) {
                lift.push_back(calibration.lift_offset[i]);
            }
            joint_cal["lift_offset_joints"] = lift;
            
            yaml_config["joint_calibration"] = joint_cal;
            
            // Save updated config
            std::ofstream file(main_config_file);
            if (file.is_open()) {
                file << yaml_config;
                file.close();
                RCLCPP_INFO(this->get_logger(), "Main configuration updated: %s", main_config_file.c_str());
            } else {
                 RCLCPP_WARN(this->get_logger(), "Could not open main config file for writing: %s", main_config_file.c_str());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to update main config: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(3)); // Give time for ROS 2 to initialize
    
    auto calibration_tool = std::make_shared<JointCalibration>();
    
    RCLCPP_INFO(calibration_tool->get_logger(), "\n=== UR5 JOINT CALIBRATION TOOL (SERVICE MODE) ===");
    RCLCPP_INFO(calibration_tool->get_logger(), "This node is now waiting for service calls to calibrate joint positions.");
    RCLCPP_INFO(calibration_tool->get_logger(), "Instructions:");
    RCLCPP_INFO(calibration_tool->get_logger(), "- Manually move the robot to each required position.");
    RCLCPP_INFO(calibration_tool->get_logger(), "- Call the corresponding ROS 2 service (e.g., 'ros2 service call /home_position std_srvs/srv/Trigger {}') to save the current position.");
    RCLCPP_INFO(calibration_tool->get_logger(), "- Once all positions are set, call '/save_all_calibration' to save the complete configuration.");
    RCLCPP_INFO(calibration_tool->get_logger(), "- Make sure the robot is in MANUAL MODE!\n");

    rclcpp::spin(calibration_tool); // Keep the node alive to listen for service calls
    
    rclcpp::shutdown();
    return 0;
}