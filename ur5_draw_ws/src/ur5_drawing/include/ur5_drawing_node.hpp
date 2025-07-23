#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include "./drawing_utils.hpp"
#include "./config_loader.hpp"

#ifdef __has_include
#if __has_include(<nlohmann/json.hpp>)
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#elif __has_include(<jsoncpp/json/json.h>)
#include <jsoncpp/json/json.h>
#else
#error "No JSON library found. Please install nlohmann-json3-dev or libjsoncpp-dev"
#endif
#else
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#endif

namespace ur5_drawing {

/**
 * @brief Main drawing node for UR5 robot
 */
class UR5DrawingNode : public rclcpp::Node {
public:
    using DrawingSequence = std::vector<std::array<double, 2>>;
    using DrawingSequences = std::vector<DrawingSequence>;

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
    DrawingConfig config_;
    DrawingUtils::Origin origin_;

    // ROS2 components
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr drawing_service_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    // MoveIt components
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    // Robot model components
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

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
     * @brief Service callback to start drawing
     */
    void startDrawingCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
    );

    /**
     * @brief Load drawing sequences from JSON file
     * @return Vector of drawing sequences
     */
    DrawingSequences loadDrawingSequences();

    /**
     * @brief Setup collision environment
     */
    void setupCollisionEnvironment();

    /**
     * @brief Execute the complete drawing process
     * @param sequences Drawing sequences to execute
     */
    void executeDrawing(const DrawingSequences& sequences);

    /**
     * @brief Convert image point to robot frame
     * @param point Image coordinates [x, y]
     * @return Robot coordinates [x, y, z]
     */
    DrawingUtils::Point3D imageToRobotFrame(const std::array<double, 2>& point);

    /**
     * @brief Move robot to home position
     */
    void moveToHome();

    /**
     * @brief Move robot to specific pose
     * @param position Target position [x, y, z]
     */
    void moveToPose(const DrawingUtils::Point3D& position);

    /**
     * @brief Execute Cartesian path through waypoints
     * @param waypoints Vector of poses to follow
     */
    void executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);

    /**
     * @brief Execute path by breaking into smaller segments if needed
     * @param waypoints Vector of poses to follow
     */
    void executeSegmentedPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);

    /**
     * @brief Execute plan with retry logic
     * @param trajectory Optional pre-planned trajectory
     * @return True if execution succeeded
     */
    bool executePlanWithRetry(const moveit_msgs::msg::RobotTrajectory* trajectory = nullptr);

    /**
     * @brief Visualize planned trajectory in RViz
     * @param trajectory Trajectory to visualize
     */
    void visualizePlan(const moveit_msgs::msg::RobotTrajectory& trajectory);

    /**
     * @brief Visualize complete drawing as markers
     * @param sequences Drawing sequences to visualize
     */
    void visualizeDrawing(const DrawingSequences& sequences);

    /**
     * @brief Get current robot state safely
     * @return Current robot state
     */
    moveit::core::RobotStatePtr getCurrentState();

    // Home joint positions
    static constexpr std::array<double, 6> HOME_JOINT_ANGLES = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
};

} // namespace ur5_drawing