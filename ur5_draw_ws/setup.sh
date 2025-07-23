#!/bin/bash

# UR5 Drawing Package Complete Setup Script
# This script sets up configuration files, directories, and .bashrc sourcing

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration paths
PACKAGE_PATH="$HOME/ur5_draw/ur5_draw_ws/src/ur5_drawing"
CONFIG_DIR="$PACKAGE_PATH/config"
RESOURCE_DIR="$PACKAGE_PATH/resource"
WORKSPACE_PATH="$HOME/ur5_draw/ur5_draw_ws"
BASHRC_FILE="$HOME/.bashrc"

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  UR5 Drawing Complete Setup Script            ${NC}"
echo -e "${BLUE}  - Configuration files                        ${NC}"
echo -e "${BLUE}  - Directory structure                        ${NC}"
echo -e "${BLUE}  - .bashrc workspace sourcing                 ${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""

# Function to check if directory exists and create if not
create_directory() {
    local dir_path="$1"
    local dir_name="$2"
    
    if [ ! -d "$dir_path" ]; then
        echo -e "${YELLOW}Creating $dir_name directory...${NC}"
        mkdir -p "$dir_path"
        echo -e "${GREEN}✓ Created: $dir_path${NC}"
    else
        echo -e "${GREEN}✓ $dir_name directory already exists${NC}"
    fi
}

# Function to create configuration file from template
create_config_file() {
    local file_path="$1"
    local file_name="$2"
    
    if [ ! -f "$file_path" ]; then
        echo -e "${YELLOW}Creating $file_name...${NC}"
        cat > "$file_path" << 'EOF'
# UR5 Drawing Configuration File
# ~/ur5_draw/ur5_draw_ws/src/ur5_drawing/config/drawing_config.yaml

# Network Configuration
network:
  robot_ip: "192.168.1.102"  # IP address of the UR5 robot
  pc_ip: "192.168.1.101"     # IP address of the control PC
  ur_type: "ur5"             # Robot type (ur3, ur5, ur10, ur16)

# Physical Setup Configuration
physical:
  # Origin position and orientation on the robot base frame
  # [x, y, z, roll, pitch, yaw] - position in meters, orientation in radians
  origin: [-0.345, -0.517, 0.032, 1.26, -2.928, 0]
  
  # Paper dimensions in meters
  paper_width: 0.2794   # 11 inches = 0.2794 meters
  paper_height: 0.2159  # 8.5 inches = 0.2159 meters
  
  # Safety and drawing parameters
  lift_offset: 0.02     # Height to lift pen above paper (meters)
  drawing_speed: 0.25   # Drawing speed (m/s)

# Image Processing Configuration
image:
  width: 800   # Expected image width in pixels
  height: 600  # Expected image height in pixels

# Robot Planning Configuration
planning:
  planning_group: "manipulator"        # MoveIt planning group name
  max_planning_attempts: 5             # Maximum planning attempts per motion
  planning_time: 5.0                   # Planning timeout (seconds)
  replan_attempts: 3                   # Number of replan attempts if execution fails
  goal_position_tolerance: 0.001       # Position tolerance for goal (meters)
  goal_orientation_tolerance: 0.01     # Orientation tolerance for goal (radians)
  
  # Cartesian path planning parameters
  cartesian:
    eef_step: 0.005          # End-effector step size for Cartesian paths (meters)
    jump_threshold: 0.0      # Jump threshold for Cartesian paths (0 = disabled)
    avoid_collisions: true   # Enable collision avoidance during planning

# File Paths Configuration
files:
  drawing_sequences: "image_description.json"  # Name of the JSON file containing drawing sequences
  rviz_config: "drawing.rviz"                  # RViz configuration file name
  calibration_file: "calibration.yaml"        # Robot calibration file name

# Launch Configuration
launch:
  use_fake_hardware: false   # Set to true for simulation, false for real robot
  headless_mode: true        # Run without robot pendant interface
  initial_joint_controller: "joint_trajectory_controller"  # Initial controller to load
  
  # Default launch arguments
  default_use_rviz: true
  default_use_sim_time: false

# Safety Limits
safety:
  max_velocity: 0.5         # Maximum allowed velocity (m/s)
  max_acceleration: 1.0     # Maximum allowed acceleration (m/s²)
  workspace_limits:         # Workspace boundaries (meters from robot base)
    x_min: -0.8
    x_max: 0.8
    y_min: -0.8
    y_max: 0.8
    z_min: 0.0
    z_max: 0.5

# Collision Environment Configuration
collision:
  object_padding: 0.01      # Padding around collision objects (meters)
  
  # Table collision object
  table:
    enabled: true
    size: [0.5, 0.5, 0.1]   # [width, depth, height] in meters
    offset: [0.0, 0.0, -0.05]  # Offset from origin [x, y, z]
  
  # Paper collision object  
  paper:
    enabled: true
    thickness: 0.002         # Paper thickness (meters)
    offset: [0.0, 0.0, 0.001]  # Slight offset above table

# Debugging and Visualization
debug:
  debug_output_path: "/tmp/ur5_drawing_debug/"  # Path for debug outputs
  verbose_planning: true     # Enable verbose MoveIt planning output
  
# Advanced Parameters (usually don't need to change)
advanced:
  # Joint limits safety margin
  joint_margin: 0.05         # Margin from joint limits (radians)
  
  # Trajectory execution parameters
  execution_timeout: 30.0    # Maximum time to wait for trajectory execution (seconds)
  
  # Service call timeouts
  service_timeout: 10.0      # Timeout for service calls (seconds)
  
  # Threading configuration
  callback_group_type: "reentrant"  # Type of callback group for service handling
EOF
        echo -e "${GREEN}✓ Created: $file_path${NC}"
    else
        echo -e "${GREEN}✓ $file_name already exists${NC}"
        echo -e "${YELLOW}  (Existing file preserved - no changes made)${NC}"
    fi
}

# Function to validate YAML file
validate_yaml() {
    local file_path="$1"
    
    echo -e "${YELLOW}Validating YAML configuration...${NC}"
    if python3 -c "import yaml; yaml.safe_load(open('$file_path'))" 2>/dev/null; then
        echo -e "${GREEN}✓ YAML configuration is valid${NC}"
    else
        echo -e "${RED}✗ YAML configuration has syntax errors${NC}"
        echo -e "${RED}  Please check the configuration file manually${NC}"
    fi
}

# Function to setup .bashrc sourcing for UR5 workspace
setup_bashrc_sourcing() {
    local source_line="source $WORKSPACE_PATH/install/setup.bash"
    local comment_line="# UR5 Drawing workspace"
    
    echo -e "${YELLOW}Setting up .bashrc sourcing for UR5 workspace...${NC}"
    
    # Check if the workspace source line already exists
    if grep -q "UR5 Drawing workspace" "$BASHRC_FILE" 2>/dev/null; then
        echo -e "${GREEN}✓ UR5 workspace sourcing already configured in .bashrc${NC}"
        return 0
    fi
    
    # Check if .bashrc exists, create if not
    if [ ! -f "$BASHRC_FILE" ]; then
        echo -e "${YELLOW}  Creating .bashrc file...${NC}"
        touch "$BASHRC_FILE"
    fi
    
    # Add sourcing to .bashrc
    echo -e "${YELLOW}  Adding UR5 workspace sourcing to .bashrc...${NC}"
    cat >> "$BASHRC_FILE" << EOF

$comment_line
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    $source_line
    echo "✓ UR5 Drawing workspace sourced"
else
    echo "⚠ UR5 Drawing workspace not found (build it first)"
fi
EOF
    
    echo -e "${GREEN}✓ Added UR5 workspace sourcing to .bashrc${NC}"
    echo -e "${YELLOW}  Note: Open a new terminal or run 'source ~/.bashrc' to apply changes${NC}"
}

# Function to test workspace availability
test_workspace() {
    echo -e "${YELLOW}Checking workspace status...${NC}"
    
    if [ -d "$WORKSPACE_PATH" ]; then
        echo -e "${GREEN}✓ Workspace directory exists${NC}"
        
        if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
            echo -e "${GREEN}✓ Workspace is built and ready${NC}"
        else
            echo -e "${YELLOW}⚠ Workspace exists but needs to be built${NC}"
            echo -e "${YELLOW}  Run: cd $WORKSPACE_PATH && colcon build --packages-select ur5_drawing${NC}"
        fi
    else
        echo -e "${YELLOW}⚠ Workspace directory not found${NC}"
        echo -e "${YELLOW}  Create the workspace first or check the path${NC}"
    fi
}

# Function to check dependencies
check_dependencies() {
    echo -e "${YELLOW}Checking dependencies...${NC}"
    
    # Check Python
    if command -v python3 &> /dev/null; then
        echo -e "${GREEN}✓ Python3 available${NC}"
    else
        echo -e "${RED}✗ Python3 not found${NC}"
    fi
    
    # Check PyYAML
    if python3 -c "import yaml" 2>/dev/null; then
        echo -e "${GREEN}✓ PyYAML available${NC}"
    else
        echo -e "${YELLOW}⚠ PyYAML not found${NC}"
        echo -e "${YELLOW}  Install with: pip3 install PyYAML${NC}"
    fi
    
    # Check ROS 2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${GREEN}✓ ROS 2 Humble found${NC}"
    elif [ -f "/opt/ros/foxy/setup.bash" ]; then
        echo -e "${GREEN}✓ ROS 2 Foxy found${NC}"
    else
        echo -e "${YELLOW}⚠ ROS 2 not found in standard location${NC}"
    fi
}

# Main setup process
echo "Setting up UR5 Drawing Package..."
echo "Package path: $PACKAGE_PATH"
echo "Workspace path: $WORKSPACE_PATH"
echo ""

# Check dependencies first
check_dependencies
echo ""

# Check if package directory exists
if [ ! -d "$PACKAGE_PATH" ]; then
    echo -e "${RED}✗ Package directory not found: $PACKAGE_PATH${NC}"
    echo -e "${RED}  Please ensure you have created the UR5 drawing package first${NC}"
    echo ""
    echo "To create the package structure:"
    echo -e "${BLUE}  mkdir -p $PACKAGE_PATH${NC}"
    echo -e "${BLUE}  mkdir -p $PACKAGE_PATH/ur5_drawing${NC}"
    echo -e "${BLUE}  mkdir -p $PACKAGE_PATH/launch${NC}"
    echo -e "${BLUE}  mkdir -p $PACKAGE_PATH/resource${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Package directory found${NC}"
echo ""

# Create directories
create_directory "$CONFIG_DIR" "config"
create_directory "$RESOURCE_DIR" "resource"
echo ""

# Create configuration file
create_config_file "$CONFIG_DIR/drawing_config.yaml" "drawing_config.yaml"
echo ""

# Validate the configuration file
if [ -f "$CONFIG_DIR/drawing_config.yaml" ]; then
    validate_yaml "$CONFIG_DIR/drawing_config.yaml"
fi
echo ""

# Setup .bashrc sourcing
setup_bashrc_sourcing
echo ""

# Test workspace status
test_workspace
echo ""

# Create placeholder for sequences file if it doesn't exist
SEQUENCES_FILE="$RESOURCE_DIR/drawing_sequences.json"
if [ ! -f "$SEQUENCES_FILE" ]; then
    echo -e "${YELLOW}Creating placeholder drawing sequences file...${NC}"
    echo '[]' > "$SEQUENCES_FILE"
    echo -e "${GREEN}✓ Created: $SEQUENCES_FILE${NC}"
    echo -e "${YELLOW}  (Empty placeholder - populate with actual drawing data)${NC}"
else
    echo -e "${GREEN}✓ Drawing sequences file already exists${NC}"
fi

echo ""
echo -e "${BLUE}================================================${NC}"
echo -e "${GREEN}✓ UR5 Drawing setup complete!${NC}"
echo -e "${BLUE}================================================${NC}"
echo ""
echo "📋 Next steps:"
echo ""
echo "1. Review and customize your configuration:"
echo "   ${BLUE}nano $CONFIG_DIR/drawing_config.yaml${NC}"
echo ""
echo "2. Update network settings for your robot:"
echo "   - robot_ip: IP address of your UR5"
echo "   - pc_ip: IP address of this computer"
echo ""
echo "3. Apply .bashrc changes:"
echo "   ${BLUE}source ~/.bashrc${NC}"
echo "   ${BLUE}# OR restart your terminal${NC}"
echo ""
echo "4. Build your package (if workspace exists):"
echo "   ${BLUE}cd $WORKSPACE_PATH${NC}"
echo "   ${BLUE}colcon build --packages-select ur5_drawing${NC}"
echo ""
echo "5. Test your configuration:"
echo "   ${BLUE}ros2 launch ur5_drawing ur5_drawing.launch.py use_fake_hardware:=true${NC}"
echo ""
echo "6. Generate drawing sequences:"
echo "   ${BLUE}cd ~/ur5_draw/image_processing${NC}"
echo "   ${BLUE}python main.py${NC}"
echo ""

# Final dependency check
if ! python3 -c "import yaml" 2>/dev/null; then
    echo -e "${YELLOW}⚠ Remember to install PyYAML:${NC}"
    echo "   ${BLUE}pip3 install PyYAML${NC}"
    echo ""
fi

echo -e "${GREEN}🎉 Setup completed successfully!${NC}"
echo ""
echo "Your UR5 Drawing system is now configured and ready to use!"
echo "Open a new terminal to see the workspace sourcing in action."