#include "../include/ur5_drawing_node.hpp"
#include <chrono>
#include <thread>
#include <filesystem>

using namespace std::chrono_literals;

namespace ur5_drawing {

UR5DrawingNode::UR5DrawingNode() : Node("ur5_drawing_node") {
    // Create callback group for parallel service handling
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Load configuration
    std::string config_path = std::string(getenv("HOME")) + "/ur5_draw/ur5_draw_ws/src/ur5_drawing/config/drawing_config.yaml";
    config_ = ConfigLoader::loadDrawingConfig(config_path);
    
    // Print configuration summary
    ConfigLoader::printConfigSummary(config_);
    
    // Initialize parameters
    initializeParameters();
    
    // Create publishers first
    display_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "display_planned_path", 10);
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "drawing_markers", 10);
    
    // Create service
    drawing_service_ = this->create_service<std_srvs::srv::Trigger>(
        "start_drawing",
        std::bind(&UR5DrawingNode::startDrawingCallback, this, 
                 std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );
    
    // Initialize MoveIt components after a delay to ensure everything is ready
    moveit_init_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            this->initializeMoveIt();
            this->moveit_init_timer_->cancel();
        }
    );
    
    RCLCPP_INFO(this->get_logger(), "UR5 Drawing Node initialized. MoveIt will be initialized in 3 seconds.");
}

UR5DrawingNode::~UR5DrawingNode() {
    // Cleanup is handled automatically by smart pointers
}

void UR5DrawingNode::initializeParameters() {
    // Setup origin structure
    if (config_.physical.origin.size() >= 6) {
        origin_.position[0] = config_.physical.origin[0];
        origin_.position[1] = config_.physical.origin[1];
        origin_.position[2] = config_.physical.origin[2];
        origin_.orientation[0] = config_.physical.origin[3]; // roll
        origin_.orientation[1] = config_.physical.origin[4]; // pitch
        origin_.orientation[2] = config_.physical.origin[5]; // yaw

        RCLCPP_INFO(this->get_logger(), "ORIGIN: %f, %f, %f", 
                    origin_.position[0], origin_.position[1], origin_.position[2]);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid origin configuration. Using defaults.");
        origin_.position = {-0.345, -0.517, 0.032};
        origin_.orientation = {1.26, -2.928, 0};
    }
}

void UR5DrawingNode::initializeMoveIt() {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt!...");
    
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
        
        // Create move group interface
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), config_.planning.planning_group);
        
        // Create planning scene interface
        planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
        
        // Set planner parameters
        move_group_->setGoalPositionTolerance(config_.planning.goal_position_tolerance);
        move_group_->setGoalOrientationTolerance(config_.planning.goal_orientation_tolerance);
        move_group_->setPlanningTime(config_.planning.planning_time);
        move_group_->setNumPlanningAttempts(config_.planning.max_planning_attempts);
        move_group_->setMaxVelocityScalingFactor(config_.safety.max_velocity);
        move_group_->setMaxAccelerationScalingFactor(config_.safety.max_acceleration);
        
        RCLCPP_INFO(this->get_logger(), "MoveIt! initialized successfully. Ready to start drawing.");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt!: %s", e.what());
        throw;
    }
}

void UR5DrawingNode::startDrawingCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    (void)request; // Suppress unused parameter warning
    
    // Check if MoveIt is initialized
    if (!move_group_ || !planning_scene_) {
        response->success = false;
        response->message = "MoveIt not initialized yet. Please wait a moment and try again.";
        RCLCPP_ERROR(this->get_logger(), "Drawing request rejected: MoveIt not initialized");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting drawing process...");
    
    try {
        // Load drawing sequences
        auto sequences = loadDrawingSequences();
        
        // Setup collision environment
        setupCollisionEnvironment();

        // moveToHome();
        
        // // Execute drawing
        executeDrawing(sequences);
        
        // Visualize the entire drawing
        visualizeDrawing(sequences);
        
        response->success = true;
        response->message = "Drawing completed successfully";
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = e.what();
        RCLCPP_ERROR(this->get_logger(), "Drawing failed: %s", e.what());
    }
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
#elif __has_include(<jsoncpp/json/json.h>)
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


void UR5DrawingNode::setupCollisionEnvironment() {
    RCLCPP_INFO(this->get_logger(), "Setting up collision environment...");
    
    // Clear existing collision objects
    planning_scene_->removeCollisionObjects(planning_scene_->getKnownObjectNames());
    
    // Convert origin RPY to Quaternion
    tf2::Quaternion q_origin;
    // The origin_.orientation stores roll, pitch, yaw in this order
    q_origin.setRPY(origin_.orientation[0], origin_.orientation[1], origin_.orientation[2]);
    geometry_msgs::msg::Quaternion ros_q_origin = tf2::toMsg(q_origin);

    // Add table as collision object
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";
    
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[table_primitive.BOX_X] = 0.5;
    table_primitive.dimensions[table_primitive.BOX_Y] = 0.5;
    table_primitive.dimensions[table_primitive.BOX_Z] = 0.1;
    
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = origin_.position[0];
    table_pose.position.y = origin_.position[1];
    table_pose.position.z = origin_.position[2] - 0.05; // Slightly below origin Z
    table_pose.orientation = ros_q_origin; // Use the calculated quaternion
    
    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    
    // Add paper as collision object
    moveit_msgs::msg::CollisionObject paper;
    paper.header.frame_id = move_group_->getPlanningFrame();
    paper.id = "paper";
    
    shape_msgs::msg::SolidPrimitive paper_primitive;
    paper_primitive.type = paper_primitive.BOX;
    paper_primitive.dimensions.resize(3);
    paper_primitive.dimensions[paper_primitive.BOX_X] = config_.physical.paper_width;
    paper_primitive.dimensions[paper_primitive.BOX_Y] = config_.physical.paper_height;
    paper_primitive.dimensions[paper_primitive.BOX_Z] = 0.002;
    
    geometry_msgs::msg::Pose paper_pose;
    paper_pose.position.x = origin_.position[0];
    paper_pose.position.y = origin_.position[1];
    paper_pose.position.z = origin_.position[2] + 0.001; // Slightly above origin Z
    paper_pose.orientation = ros_q_origin; // Use the calculated quaternion
    
    paper.primitives.push_back(paper_primitive);
    paper.primitive_poses.push_back(paper_pose);
    paper.operation = paper.ADD;
    
    // Apply collision objects
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects = {table, paper};
    planning_scene_->addCollisionObjects(collision_objects);
    
    // Wait for collision objects to be added
    std::this_thread::sleep_for(500ms);
    
    RCLCPP_INFO(this->get_logger(), "Collision environment setup complete with rotated table and paper");
}

void UR5DrawingNode::executeDrawing(const DrawingSequences& sequences) {
    RCLCPP_INFO(this->get_logger(), "Starting drawing execution...");
    
    // // Move to home position
    // moveToHome();
    
    // Lift pen to safe height above paper center
    std::array<double, 2> center_point = {
        static_cast<double>(config_.image.width) / 2.0,
        static_cast<double>(config_.image.height) / 2.0
    };
    auto home_pos = imageToRobotFrame(center_point);
    home_pos[2] += config_.physical.lift_offset;
    moveToPose(home_pos);
    
    // Draw each sequence
    for (size_t seq_idx = 0; seq_idx < sequences.size(); ++seq_idx) {
        const auto& sequence = sequences[seq_idx];
        
        RCLCPP_INFO(this->get_logger(), 
                   "Drawing sequence %zu/%zu with %zu points",
                   seq_idx + 1, sequences.size(), sequence.size());
        
        // Create waypoints for this sequence
        std::vector<geometry_msgs::msg::Pose> waypoints;
        
        // Start with pen lifted
        auto start_pos = imageToRobotFrame(sequence[0]);
        auto lifted_start = start_pos;
        lifted_start[2] += config_.physical.lift_offset;
        waypoints.push_back(DrawingUtils::createPose(lifted_start, nullptr, &origin_.orientation));
        
        // Lower pen to drawing height
        waypoints.push_back(DrawingUtils::createPose(start_pos, nullptr, &origin_.orientation));
        
        // Add all points in the sequence
        for (const auto& point : sequence) {
            auto point_pos = imageToRobotFrame(point);
            waypoints.push_back(DrawingUtils::createPose(point_pos, nullptr, &origin_.orientation));
        }
        
        // Lift pen at end of sequence
        auto end_pos = imageToRobotFrame(sequence.back());
        auto lifted_end = end_pos;
        lifted_end[2] += config_.physical.lift_offset;
        waypoints.push_back(DrawingUtils::createPose(lifted_end, nullptr, &origin_.orientation));
        
        // Execute Cartesian path
        executeCartesianPath(waypoints);
    }
    
    // // Return to home position
    // moveToHome();
    RCLCPP_INFO(this->get_logger(), "Drawing completed!");
}

DrawingUtils::Point3D UR5DrawingNode::imageToRobotFrame(const std::array<double, 2>& point) {
    return DrawingUtils::imageToRobotFrame(
        point,
        {config_.image.width, config_.image.height},
        {config_.physical.paper_width, config_.physical.paper_height},
        origin_
    );
}

// void UR5DrawingNode::moveToHome() {
//     RCLCPP_INFO(this->get_logger(), "Moving to home position...");
    
//     std::vector<double> joint_values(HOME_JOINT_ANGLES.begin(), HOME_JOINT_ANGLES.end());
//     move_group_->setJointValueTarget(joint_values);
    
//     if (!executePlanWithRetry()) {
//         throw std::runtime_error("Failed to move to home position");
//     }
// }

void UR5DrawingNode::moveToPose(const DrawingUtils::Point3D& position) {
    auto target_pose = DrawingUtils::createPose(position, nullptr, &origin_.orientation);
    move_group_->setPoseTarget(target_pose);
    
    if (!executePlanWithRetry()) {
        throw std::runtime_error("Failed to move to target pose");
    }
}

void UR5DrawingNode::executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    if (waypoints.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Not enough waypoints for Cartesian path");
        return;
    }
    
    // Plan Cartesian path
    auto [fraction, trajectory] = DrawingUtils::planCartesianPath(
        *move_group_,
        waypoints,
        config_.cartesian.eef_step,
        config_.cartesian.jump_threshold,
        config_.cartesian.avoid_collisions
    );
    
    // Check if path was planned successfully
    if (fraction < 0.9) {
        RCLCPP_WARN(this->get_logger(),
                   "Only %.1f%% of the path was planned. Attempting to replan segments...",
                   fraction * 100.0);
        executeSegmentedPath(waypoints);
    } else {
        // Visualize and execute the plan
        visualizePlan(trajectory);
        if (!executePlanWithRetry(&trajectory)) {
            throw std::runtime_error("Failed to execute Cartesian path");
        }
    }
}

void UR5DrawingNode::executeSegmentedPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        std::vector<geometry_msgs::msg::Pose> segment = {waypoints[i], waypoints[i + 1]};
        
        // Plan segment
        auto [fraction, trajectory] = DrawingUtils::planCartesianPath(
            *move_group_,
            segment,
            config_.cartesian.eef_step,
            config_.cartesian.jump_threshold,
            config_.cartesian.avoid_collisions
        );
        
        if (fraction >= 0.9) {
            if (!executePlanWithRetry(&trajectory)) {
                throw std::runtime_error("Failed to execute path segment");
            }
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Segment %zu/%zu planning failed (fraction=%.2f). Using point-to-point.",
                       i + 1, waypoints.size() - 1, fraction);
            
            move_group_->setPoseTarget(waypoints[i + 1]);
            if (!executePlanWithRetry()) {
                throw std::runtime_error("Failed to execute point-to-point movement");
            }
        }
    }
}

bool UR5DrawingNode::executePlanWithRetry(const moveit_msgs::msg::RobotTrajectory* trajectory) {
    int attempts = 0;
    
    while (attempts < config_.planning.replan_attempts) {
        try {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            if (trajectory) {
                plan.trajectory_ = *trajectory;
                auto error_code = move_group_->execute(plan);
                if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
                    return true;
                }
            } else {
                auto error_code = move_group_->plan(plan);
                if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
                    error_code = move_group_->execute(plan);
                    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
                        return true;
                    }
                }
            }
            
            RCLCPP_WARN(this->get_logger(), 
                       "Execution failed on attempt %d. Replanning...", attempts + 1);
            attempts++;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Execution error: %s", e.what());
            attempts++;
        }
    }
    
    RCLCPP_ERROR(this->get_logger(), 
                "Failed to execute plan after %d attempts", config_.planning.replan_attempts);
    return false;
}

void UR5DrawingNode::visualizePlan(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    
    // Convert current robot state to RobotState message
    moveit_msgs::msg::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*getCurrentState(), robot_state_msg);
    display_trajectory.trajectory_start = robot_state_msg;
    
    display_trajectory.trajectory.push_back(trajectory);
    display_trajectory_pub_->publish(display_trajectory);
}

void UR5DrawingNode::visualizeDrawing(const DrawingSequences& sequences) {
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    
    for (size_t seq_idx = 0; seq_idx < sequences.size(); ++seq_idx) {
        const auto& sequence = sequences[seq_idx];
        
        // Create line strip marker for this sequence
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = move_group_->getPlanningFrame();
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "drawing";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01   ; // Line width
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Add points to the line strip
        for (const auto& point : sequence) {
            auto pos = imageToRobotFrame(point);
            geometry_msgs::msg::Point p;
            p.x = pos[0];
            p.y = pos[1];
            p.z = pos[2];
            marker.points.push_back(p);
        }
        
        marker_array.markers.push_back(marker);
    }
    
    // Publish marker array
    marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Drawing visualization published");
}

moveit::core::RobotStatePtr UR5DrawingNode::getCurrentState() {
    return move_group_->getCurrentState();
}

} // namespace ur5_drawing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ur5_drawing::UR5DrawingNode>();
    
    // Use MultiThreadedExecutor to handle service calls while planning
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