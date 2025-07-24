#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include "drawing_utils.hpp"
#include "config_loader.hpp"

#ifdef __has_include
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#else
#include <jsoncpp/json/json.h>
#endif
#else
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

namespace ur5_drawing {

/**
 * @brief Main joint-based drawing node for UR5 robot
 */
class UR5DrawingNode : public rclcpp::Node {
public:
    using DrawingSequence = std::vector<std::array<double, 2>>;
    using DrawingSequences = std::vector<DrawingSequence>;
    using FollowJointTrajectoryAction = control_msgs::action::FollowJointTrajectory;
    using JointTrajectoryActionClient = rclcpp_action::Client<FollowJointTrajectoryAction>;

    /**
     * @brief Constructor
     */
    UR5DrawingNode();

    /**
     * @brief Destructor
     */
    ~UR5DrawingNode();

private:
    // Configuration
    JointDrawingConfig config_;
    CornerPositions corner_positions_;

    // ROS2 components
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drawing_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_stop_service_;
    
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    // Action client for joint trajectory control
    JointTrajectoryActionClient::SharedPtr joint_trajectory_action_client_;

    // MoveIt components (used for visualization and planning)
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    // Robot model components
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    // Joint state tracking
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    JointPosition current_joint_position_;
    std::vector<std::string> joint_names_;

    // Emergency stop flag
    std::atomic<bool> emergency_stop_;

    // Callback group for parallel service handling
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr moveit_init_timer_;

    /**
     * @brief Initialize parameters from config
     */
    void initializeParameters();

    /**
     * @brief Initialize MoveIt components
     */
    void initializeMoveIt();

    /**
     * @brief Initialize joint trajectory action client
     */
    void initializeJointTrajectoryClient();

    /**
     * @brief Service callback to start drawing
     */
    void startDrawingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    /**
     * @brief Service callback for joint calibration
     */
    void calibrationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    /**
     * @brief Service callback for emergency stop
     */
    void emergencyStopCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );

    /**
     * @brief Joint state callback
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Load drawing sequences from JSON file
     * @return Vector of drawing sequences
     */
    DrawingSequences loadDrawingSequences();

    /**
     * @brief Execute the complete joint-based drawing process
     * @param sequences Drawing sequences to execute
     */
    void executeJointDrawing(const DrawingSequences& sequences);

    /**
     * @brief Convert image point to joint angles
     * @param point Image coordinates [x, y]
     * @return Joint angles in radians
     */
    JointPosition imageToJointAngles(const std::array<double, 2>& point);

    /**
     * @brief Move robot to joint position
     * @param target_joints Target joint angles
     * @param with_interpolation Whether to use interpolated path
     */
    void moveToJointPosition(const JointPosition& target_joints, bool with_interpolation = true);

    /**
     * @brief Execute joint trajectory using action client
     * @param trajectory Joint trajectory to execute
     * @return True if execution succeeded
     */
    bool executeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Execute joint path through multiple waypoints
     * @param waypoints Vector of joint positions to follow
     * @param lift_pen_between Whether to lift pen between waypoints
     */
    void executeJointPath(const std::vector<JointPosition>& waypoints, bool lift_pen_between = false);

    /**
     * @brief Move to home position
     */
    void moveToHome();

    /**
     * @brief Perform joint calibration routine
     */
    void performJointCalibration();

    /**
     * @brief Visualize joint trajectory in RViz
     * @param trajectory Joint trajectory to visualize
     */
    void visualizeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Visualize complete drawing as markers
     * @param sequences Drawing sequences to visualize  
     */
    void visualizeDrawing(const DrawingSequences& sequences);

    /**
     * @brief Check if robot is in emergency stop state
     */
    bool isEmergencyStop() const { return emergency_stop_.load(); }

    /**
     * @brief Validate joint trajectory for safety
     * @param trajectory Trajectory to validate
     * @return True if trajectory is safe
     */
    bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

    /**
     * @brief Get current joint position from joint state
     * @return Current joint position
     */
    JointPosition getCurrentJointPosition();

    /**
     * @brief Update corner positions from configuration
     */
    void updateCornerPositions();

    // UR5 joint limits (in radians)
    static constexpr JointPosition UR5_MIN_LIMITS = {-6.28, -6.28, -3.14, -6.28, -6.28, -6.28};
    static constexpr JointPosition UR5_MAX_LIMITS = {6.28, 6.28, 3.14, 6.28, 6.28, 6.28};
};

} // namespace ur5_drawing