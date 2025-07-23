#!/bin/bash

# UR5 Drawing System - Clean Launch Script
# This script launches the UR5 drawing system with minimal terminal output

echo "🤖 UR5 Drawing System - Clean Launch"
echo "=================================="
echo ""

# Check if the workspace is built
if [ ! -d "$HOME/ur5_draw/ur5_draw_ws/install" ]; then
    echo "❌ Workspace not built. Please run: colcon build"
    exit 1
fi

# Source the workspace
source $HOME/ur5_draw/ur5_draw_ws/install/setup.bash

echo "Launching UR5 Drawing System..."
echo "   - Only UR5 Drawing Node output will be shown"
echo "   - Other system messages are logged to files"
echo "   - Check ~/.ros/log/ for detailed logs if needed"
echo ""

# Set ROS log level to reduce noise from other nodes
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=30  # ERROR level for other nodes

# Launch the system
ros2 launch ur5_drawing ur5_drawing.launch.py

echo ""
echo "🛑 UR5 Drawing System stopped"