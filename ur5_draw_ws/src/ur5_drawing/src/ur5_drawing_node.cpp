#include "../include/ur5_drawing_node.hpp"
#include <chrono>
#include <thread>
#include <filesystem>
#include <fcntl.h>
#include <future>

using namespace std::chrono_literals;

namespace ur5_drawing {

UR5DrawingNode::UR5DrawingNode() : Node("ur5_drawing_node"), emergency_stop_(false) {
    // Initialize joint names for UR5
    joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", 
               "wrist_2_joint", "wrist_3_joint"}; 
    
    // Create callback group for parallel service handling
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Load configuration
    std::string config_path = std::string(getenv("HOME")) + "/ur5_draw/ur5_draw_ws/src/ur5_drawing/config/drawing_config.yaml";
    config_ = ConfigLoader::loadJointDrawingConfig(config_path);
    
    // Print configuration summary
    ConfigLoader::printConfigSummary(config_);
    
    // Initialize parameters
    initializeParameters();
    
    // Create publishers
    display_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "display_planned_path", 10);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "drawing_markers", 10);
    
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "joint_trajectory", 10);
    
    // Create joint state subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&UR5DrawingNode::jointStateCallback, this, std::placeholders::_1)
    );
    
    // Create services with proper callback group assignment
    // auto service_options = rclcpp::ServiceOptions(); // This line is causing the error
    // service_options.callback_group = callback_group_; // This line is causing the error
    
    drawing_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_drawing",
        std::bind(&UR5DrawingNode::startDrawingCallback, this, 
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_ // Directly pass the callback group here
    );
    
    calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate_joints",
        std::bind(&UR5DrawingNode::calibrationCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_ // Directly pass the callback group here
    );
    
    emergency_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "emergency_stop",
        std::bind(&UR5DrawingNode::emergencyStopCallback, this,
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_ // Directly pass the callback group here
    );
    
    // Initialize joint trajectory action client
    initializeJointTrajectoryClient();
    
    // Initialize MoveIt components after a delay
    moveit_init_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            this->initializeMoveIt();
            this->moveit_init_timer_->cancel();
        }
    );
    
    RCLCPP_INFO(this->get_logger(), "UR5 Joint Drawing Node initialized. MoveIt will be initialized in 3 seconds.");
}

UR5DrawingNode::~UR5DrawingNode() {
    // Cleanup is handled automatically by smart pointers
}

void UR5DrawingNode::initializeParameters() {
    // Update corner positions from configuration
    updateCornerPositions();
    
    RCLCPP_INFO(this->get_logger(), "Joint-based drawing parameters initialized");
}

void UR5DrawingNode::initializeMoveIt() {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt! for visualization...");
    
    try {
        // Initialize robot model loader
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
            shared_from_this(), "robot_description");
        
        robot_model_ = robot_model_loader_->getModel();
        if (!robot_model_) {
            throw std::runtime_error("Failed to load robot model");
        }
        
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();
        
        // Create move group interface (for visualization only)
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), config_.planning.planning_group);
        
        // Create planning scene interface
        planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

        move_group_->setGoalPositionTolerance(config_.planning.goal_position_tolerance);
        move_group_->setGoalOrientationTolerance(config_.planning.goal_orientation_tolerance);
        move_group_->setPlanningTime(config_.planning.planning_time);
        move_group_->setNumPlanningAttempts(config_.planning.max_planning_attempts);
        move_group_->setMaxVelocityScalingFactor(config_.safety.max_velocity);
        move_group_->setMaxAccelerationScalingFactor(config_.safety.max_acceleration);
        
        RCLCPP_INFO(this->get_logger(), "MoveIt! initialized for visualization. Ready for joint-based drawing.");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt!: %s", e.what());
        throw;
    }
}

void UR5DrawingNode::initializeJointTrajectoryClient() {
    joint_trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectoryAction>(
        this, "scaled_joint_trajectory_controller/follow_joint_trajectory");
    
    // Wait for action server
    if (!joint_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Joint trajectory action server not available. Commands will be published to topic instead.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Joint trajectory action client initialized");
    }
}

void UR5DrawingNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    current_joint_state_ = msg;
    
    // Update current joint position
    if (msg->position.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            current_joint_position_[i] = msg->position[i];
        }
    }
}

void UR5DrawingNode::startDrawingCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    if (isEmergencyStop()) {
        response->success = false;
        response->message = "Cannot start drawing: Emergency stop is active";
        return;
    }
    
    // Check if joint trajectory client is ready
    if (!current_joint_state_) {
        response->success = false;
        response->message = "Joint drawing not ready: No joint state received";
        RCLCPP_ERROR(this->get_logger(), "Drawing request rejected: No joint state");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting joint-based drawing process...");
    
    try {
        // Load drawing sequences
        auto sequences = loadDrawingSequences();
        
        // Execute joint-based drawing
        executeJointDrawing(sequences);
        
        // Visualize the entire drawing
        visualizeDrawing(sequences);
        
        response->success = true;
        response->message = "Joint-based drawing completed successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Joint drawing failed: %s", e.what());
    }
}

void UR5DrawingNode::calibrationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request;
    
    RCLCPP_INFO(this->get_logger(), "Starting joint calibration process...");
    
    try {
        performJointCalibration();
        response->success = true;
        response->message = "Joint calibration completed successfully";
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Joint calibration failed: %s", e.what());
    }
}

void UR5DrawingNode::emergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    emergency_stop_.store(request->data);
    
    if (request->data) {
        RCLCPP_WARN(this->get_logger(), "Emergency stop activated!");
        response->message = "Emergency stop activated";
    } else {
        RCLCPP_INFO(this->get_logger(), "Emergency stop deactivated");
        response->message = "Emergency stop deactivated";
    }
    
    response->success = true;
}

UR5DrawingNode::DrawingSequences UR5DrawingNode::loadDrawingSequences() {
    std::string drawing_file = std::string(getenv("HOME")) + 
        "/ur5_draw/ur5_draw_ws/src/ur5_drawing/image_description/image_description.json";
    
    try {
        std::ifstream file(drawing_file);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open drawing file: " + drawing_file);
        }
        
        DrawingSequences sequences;
        
#ifdef __has_include
#if __has_include(<nlohmann/json.hpp>)
        json j;
        file >> j;
        
        for (const auto& sequence : j) {
            DrawingSequence seq;
            for (const auto& point : sequence) {
                if (point.size() >= 2) {
                    seq.push_back({point[0].get<double>(), point[1].get<double>()});
                }
            }
            if (!seq.empty()) {
                sequences.push_back(seq);
            }
        }
#else
        Json::Value root;
        Json::Reader reader;
        std::string file_content((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
        
        if (!reader.parse(file_content, root)) {
            throw std::runtime_error("Failed to parse JSON: " + reader.getFormattedErrorMessages());
        }
        
        for (const auto& sequence : root) {
            DrawingSequence seq;
            for (const auto& point : sequence) {
                if (point.size() >= 2) {
                    seq.push_back({point[0].asDouble(), point[1].asDouble()});
                }
            }
            if (!seq.empty()) {
                sequences.push_back(seq);
            }
        }
#endif
#else
        json j;
        file >> j;
        
        for (const auto& sequence : j) {
            DrawingSequence seq;
            for (const auto& point : sequence) {
                if (point.size() >= 2) {
                    seq.push_back({point[0].get<double>(), point[1].get<double>()});
                }
            }
            if (!seq.empty()) {
                sequences.push_back(seq);
            }
        }
#endif
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu sequences from %s", 
                   sequences.size(), drawing_file.c_str());
        
        return sequences;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load drawing sequences: %s", e.what());
        throw;
    }
}

void UR5DrawingNode::executeJointDrawing(const DrawingSequences& sequences) {
    RCLCPP_INFO(this->get_logger(), "Starting joint-based drawing execution...");
    
    // Move to home position first
    moveToHome();
    
    // Draw each sequence
    for (size_t seq_idx = 0; seq_idx < sequences.size(); ++seq_idx) {
        if (isEmergencyStop()) {
            RCLCPP_WARN(this->get_logger(), "Drawing stopped due to emergency stop");
            break;
        }
        
        const auto& sequence = sequences[seq_idx];
        
        RCLCPP_INFO(this->get_logger(), 
                   "Drawing sequence %zu/%zu with %zu points",
                   seq_idx + 1, sequences.size(), sequence.size());
        // Create joint waypoints for this sequence
        std::vector<JointPosition> waypoints;
        
        // Start with pen lifted above first point
        auto start_joints = imageToJointAngles(sequence[0]);
        auto lifted_start = DrawingUtils::addPenLift(start_joints, config_.joint_calibration.lift_offset_joints);
        waypoints.push_back(lifted_start);
        
        // Lower pen to drawing position
        waypoints.push_back(start_joints);
        
        // Add all points in the sequence
        for (const auto& point : sequence) {
            auto point_joints = imageToJointAngles(point);
            waypoints.push_back(point_joints);
        }
        
        // Lift pen at end of sequence
        auto end_joints = imageToJointAngles(sequence.back());
        auto lifted_end = DrawingUtils::addPenLift(end_joints, config_.joint_calibration.lift_offset_joints);
        waypoints.push_back(lifted_end);
        
        // Execute joint path
        executeJointPath(waypoints, false);
    }
    
    // Return to home position
    moveToHome();
    RCLCPP_INFO(this->get_logger(), "Joint-based drawing completed!");
}

JointPosition UR5DrawingNode::imageToJointAngles(const std::array<double, 2>& point) {
    return DrawingUtils::imageToJointSpace(
        point,
        {config_.image.width, config_.image.height},
        corner_positions_
    );
}

void UR5DrawingNode::moveToJointPosition(const JointPosition& target_joints, bool with_interpolation) {
    if (isEmergencyStop()) {
        RCLCPP_WARN(this->get_logger(), "Movement cancelled due to emergency stop");
        return;
    }

    if (!move_group_) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt! MoveGroupInterface not initialized. Cannot plan trajectory.");
        throw std::runtime_error("MoveIt! not ready for planning.");
    }

    RCLCPP_INFO(this->get_logger(), "--- Planning and Executing Joint Trajectory (using MoveIt!) ---");

    // Convert std::array to std::vector for MoveIt!
    std::vector<double> moveit_target_joints(target_joints.begin(), target_joints.end());
    
    // 1. Set the joint value target
    move_group_->setJointValueTarget(moveit_target_joints);

    // 2. Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(this->get_logger(), "MoveIt! planning successful. Executing trajectory...");

        // Log the first point of MoveIt!'s planned trajectory for sanity check
        if (!my_plan.trajectory_.joint_trajectory.points.empty()) {
            const auto& first_point = my_plan.trajectory_.joint_trajectory.points[0];
            std::string joint_positions_str = "MoveIt! planned first point (degrees): [";
            for (size_t i = 0; i < first_point.positions.size(); ++i) {
                joint_positions_str += std::to_string((first_point.positions[i] * 180.0) / M_PI);
                if (i < first_point.positions.size() - 1) {
                    joint_positions_str += ", ";
                }
            }
            joint_positions_str += "]";
            RCLCPP_INFO(this->get_logger(), "%s", joint_positions_str.c_str());
        }

        // 3. Execute the trajectory using your existing executeJointTrajectory
        // This will send the trajectory to the scaled_joint_trajectory_controller
        if (!executeJointTrajectory(my_plan.trajectory_.joint_trajectory)) {
            throw std::runtime_error("Failed to execute MoveIt! planned trajectory");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "MoveIt! planning failed for target joint position.");
        throw std::runtime_error("MoveIt! planning failed.");
    }
}

bool UR5DrawingNode::executeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {

    if (trajectory.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Attempted to execute an empty joint trajectory.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "--- Executing Joint Trajectory ---");
    if (!trajectory.points.empty()) {
        const auto& first_point = trajectory.points[0];
        std::string joint_positions_str = "First point joint positions: [";
        for (size_t i = 0; i < first_point.positions.size(); ++i) {
            joint_positions_str += std::to_string((first_point.positions[i] * 180)/ M_PI);
            if (i < first_point.positions.size() - 1) {
                joint_positions_str += ", ";
            }
        }
        joint_positions_str += "]";
        RCLCPP_INFO(this->get_logger(), "%s", joint_positions_str.c_str());

        std::string joint_names_str = "Joint names: [";
        for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
            joint_names_str += trajectory.joint_names[i];
            if (i < trajectory.joint_names.size() - 1) {
                joint_names_str += ", ";
            }
        }
        joint_names_str += "]";
        RCLCPP_INFO(this->get_logger(), "%s", joint_names_str.c_str());
    }

    if (isEmergencyStop()) {
        RCLCPP_WARN(this->get_logger(), "Trajectory execution cancelled due to emergency stop");
        return false;
    }
    
    // Validate trajectory first
    if (!validateTrajectory(trajectory)) {
        RCLCPP_ERROR(this->get_logger(), "Trajectory validation failed");
        return false;
    }
    
    try {
        if (joint_trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            // Use action client with promise/future pattern to avoid executor conflicts
            auto goal_msg = FollowJointTrajectoryAction::Goal();
            goal_msg.trajectory = trajectory;
            
            std::promise<bool> result_promise;
            std::future<bool> result_future = result_promise.get_future();
            
            auto send_goal_options = rclcpp_action::Client<FollowJointTrajectoryAction>::SendGoalOptions();
            
            send_goal_options.result_callback = [&result_promise](const auto& result) {
                bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                result_promise.set_value(success);
            };
            
            send_goal_options.feedback_callback = [this](auto, const auto& feedback) {
                // Optional: Handle feedback
                (void)feedback;
                RCLCPP_DEBUG(this->get_logger(), "Trajectory execution in progress");
            };
            
            // Send goal asynchronously
            auto goal_handle_future = joint_trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
            
            // Wait for result with timeout
            auto status = result_future.wait_for(std::chrono::seconds(30));
            if (status == std::future_status::ready) {
                bool success = result_future.get();
                if (success) {
                    RCLCPP_DEBUG(this->get_logger(), "Joint trajectory executed successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Joint trajectory execution failed");
                }
                return success;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Joint trajectory execution timed out");
                return false;
            }
            
        } else {
            // Fallback to topic publishing
            RCLCPP_DEBUG(this->get_logger(), "Publishing joint trajectory to topic");
            joint_trajectory_pub_->publish(trajectory);
            
            // Wait for trajectory completion (estimate based on last point time)
            if (!trajectory.points.empty()) {
                auto last_point_time = trajectory.points.back().time_from_start;
                auto duration = std::chrono::seconds(last_point_time.sec) + 
                               std::chrono::nanoseconds(last_point_time.nanosec) + 500ms;
                std::this_thread::sleep_for(duration);
            }
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute joint trajectory: %s", e.what());
        return false;
    }
}

void UR5DrawingNode::executeJointPath(const std::vector<JointPosition>& waypoints, bool lift_pen_between) {
    if (waypoints.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty waypoint list");
        return;
    }
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        if (isEmergencyStop()) {
            RCLCPP_WARN(this->get_logger(), "Path execution stopped due to emergency stop");
            break;
        }
        
        // Move to waypoint
        moveToJointPosition(waypoints[i], true);
        
        // Optional pen lift between waypoints
        if (lift_pen_between && i < waypoints.size() - 1) {
            auto lifted_pos = DrawingUtils::addPenLift(waypoints[i], config_.joint_calibration.lift_offset_joints);
            moveToJointPosition(lifted_pos, false);
        }
    }
}

void UR5DrawingNode::moveToHome() {
    RCLCPP_INFO(this->get_logger(), "Moving to home position...");
    moveToJointPosition(config_.joint_calibration.home_position, true);
}

void UR5DrawingNode::performJointCalibration() {
    RCLCPP_INFO(this->get_logger(), "Joint calibration should be performed using the calibration tool");
    RCLCPP_INFO(this->get_logger(), "Run: ros2 run ur5_drawing joint_calibration");
    
    // This could also trigger the calibration tool or provide guidance
    throw std::runtime_error("Use the joint_calibration for interactive calibration");
}

void UR5DrawingNode::visualizeJointTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (!move_group_ || !robot_state_) {
        RCLCPP_WARN(this->get_logger(), "MoveIt not initialized, cannot visualize trajectory");
        return;
    }
    
    // Convert joint trajectory to display trajectory
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    
    // Set trajectory start state
    moveit_msgs::msg::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, robot_state_msg);
    display_trajectory.trajectory_start = robot_state_msg;
    
    // Convert joint trajectory to robot trajectory
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    robot_trajectory.joint_trajectory = trajectory;
    
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_trajectory_pub_->publish(display_trajectory);
}

void UR5DrawingNode::visualizeDrawing(const DrawingSequences& sequences) {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    for (size_t seq_idx = 0; seq_idx < sequences.size(); ++seq_idx) {
        const auto& sequence = sequences[seq_idx];
        
        // Create line strip marker for this sequence
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "joint_drawing";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01; // Line width
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0; // Blue for joint-based drawing
        marker.color.a = 1.0;
        
        // Convert joint positions to Cartesian for visualization
        // This requires forward kinematics, which we can do with MoveIt
        if (move_group_ && robot_state_) {
            for (const auto& point : sequence) {
                auto joint_pos = imageToJointAngles(point);
                
                // Set robot state to joint position
                robot_state_->setJointGroupPositions(config_.planning.planning_group, 
                    std::vector<double>(joint_pos.begin(), joint_pos.end()));
                
                // Get end-effector pose
                const auto& end_effector_state = robot_state_->getGlobalLinkTransform("tool0");
                
                geometry_msgs::msg::Point p;
                p.x = end_effector_state.translation().x();
                p.y = end_effector_state.translation().y();
                p.z = end_effector_state.translation().z();
                marker.points.push_back(p);
            }
        }
        
        if (!marker.points.empty()) {
            marker_array.markers.push_back(marker);
        }
    }
    
    // Publish marker array
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Joint-based drawing visualization published");
}

bool UR5DrawingNode::validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    auto [min_limits, max_limits] = DrawingUtils::getUR5JointLimits();
    
    for (const auto& point : trajectory.points) {
        if (point.positions.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid trajectory point: expected 6 joints, got %zu", 
                        point.positions.size());
            return false;
        }
        
        JointPosition joint_pos;
        std::copy(point.positions.begin(), point.positions.begin() + 6, joint_pos.begin());
        
        if (!DrawingUtils::validateJointLimits(joint_pos, min_limits, max_limits)) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory point exceeds joint limits");
            return false;
        }
    }
    
    return true;
}

JointPosition UR5DrawingNode::getCurrentJointPosition() {
    if (current_joint_state_ && current_joint_state_->position.size() >= 6) {
        return current_joint_position_;
    } else {
        RCLCPP_WARN(this->get_logger(), "No current joint state available, using home position");
        return config_.joint_calibration.home_position;
    }
}

void UR5DrawingNode::updateCornerPositions() {
    corner_positions_.bottom_left = config_.joint_calibration.corners.bottom_left;
    corner_positions_.bottom_right = config_.joint_calibration.corners.bottom_right;
    corner_positions_.top_left = config_.joint_calibration.corners.top_left;
    corner_positions_.top_right = config_.joint_calibration.corners.top_right;
}

} // namespace ur5_drawing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ur5_drawing::UR5DrawingNode>();
    
    // Use MultiThreadedExecutor to handle service calls while executing trajectories
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in executor: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
