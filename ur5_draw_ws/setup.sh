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
WORKSPACE_PATH="$HOME/ur5_draw/ur5_draw_ws"
BASHRC_FILE="$HOME/.bashrc"

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  UR5 Drawing Complete Setup Script           ${NC}"
echo -e "${BLUE}  - Configuration files                       ${NC}"
echo -e "${BLUE}  - Directory structure                       ${NC}"
echo -e "${BLUE}  - .bashrc workspace sourcing                ${NC}"
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
    local custom_rc_file="$WORKSPACE_PATH/.ur5_drawrc"
    local custom_rc_comment="# UR5 Drawing custom settings (ur5_drawrc)"
    
    echo -e "${YELLOW}Setting up .bashrc sourcing for UR5 workspace...${NC}"
    
    # Check if .bashrc exists, create if not
    if [ ! -f "$BASHRC_FILE" ]; then
        echo -e "${YELLOW}  Creating .bashrc file...${NC}"
        touch "$BASHRC_FILE"
    fi
    
    local changes_made=false

    # Check if the workspace source line already exists
    if ! grep -q "UR5 Drawing workspace" "$BASHRC_FILE" 2>/dev/null; then
        # Add workspace sourcing to .bashrc
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
        changes_made=true
    else
        echo -e "${GREEN}✓ UR5 workspace sourcing already configured in .bashrc${NC}"
    fi
    
    # Check if the custom ur5_drawrc source line already exists
    if ! grep -q "UR5 Drawing custom settings" "$BASHRC_FILE" 2>/dev/null; then
        # Add custom rc sourcing to .bashrc
        echo -e "${YELLOW}  Adding ur5_drawrc sourcing to .bashrc...${NC}"
        cat >> "$BASHRC_FILE" << EOF

$custom_rc_comment
if [ -f "$custom_rc_file" ]; then
    source "$custom_rc_file"
    echo "✓ UR5 Drawing custom settings (ur5_drawrc) sourced"
fi
EOF
        echo -e "${GREEN}✓ Added ur5_drawrc sourcing to .bashrc${NC}"
        changes_made=true
    else
        echo -e "${GREEN}✓ Custom ur5_drawrc sourcing already configured in .bashrc${NC}"
    fi

    if [ "$changes_made" = true ]; then
        echo -e "${YELLOW}  Note: Open a new terminal or run 'source ~/.bashrc' to apply changes${NC}"
    fi
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
    exit 1
fi

echo -e "${GREEN}✓ Package directory found${NC}"
echo ""

# Create directories
create_directory "$CONFIG_DIR" "config"
echo ""

# Create configuration file
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
echo "3. (Optional) Add custom aliases or exports to your workspace:"
echo "   ${BLUE}nano $WORKSPACE_PATH/ur5_drawrc${NC}"
echo ""
echo "4. Apply .bashrc changes:"
echo "   ${BLUE}source ~/.bashrc${NC}"
echo "   ${BLUE}# OR restart your terminal${NC}"
echo ""
echo "5. Build your package (if workspace exists):"
echo "   ${BLUE}cd $WORKSPACE_PATH${NC}"
echo "   ${BLUE}colcon build --packages-select ur5_drawing${NC}"
echo ""
echo "6. Test your configuration:"
echo "   ${BLUE}ros2 launch ur5_drawing ur5_drawing.launch.py use_fake_hardware:=true${NC}"
echo ""
echo "7. Generate drawing sequences:"
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