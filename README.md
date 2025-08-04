# Robotics & Drawing

## Overview

This project combines:

- **Universal Robots ROS2 Driver** (hard fork for simplified setup)
- **MoveIt 2** for trajectory planning and execution
- **Computer Vision Pipeline** for image-to-trajectory conversion
- **Joint-based Control** for precise drawing movements

## Quick Start

### 1. Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- Network connection to UR5 robot

### 2. Installation

```bash
# Clone and setup
git clone https://github.com/cmuroboticsnoel/ur5_draw.git
cd ur5_draw/ur5_draw_ws
chmod +x setup.sh
./setup.sh

# Build and source
colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release
sros

# Install Python dependencies
cd ../image_processing
pip3 install -r requirements.txt
```

### 3. Configuration

Edit `ur5_draw_ws/src/ur5_drawing/config/drawing_config.yaml`:

```yaml
network:
  robot_ip: "192.168.1.102" # Your UR5 IP
  pc_ip: "192.168.1.101" # Your PC IP
  ur_type: "ur5"

physical:
  paper_width: 0.2794 # A4 width (m)
  paper_height: 0.2159 # A4 height (m)
  drawing_speed: 0.20 # Drawing speed (m/s)
```

### 4. Run the System

```bash
# Launch (simulation mode)
ros2 launch ur5_drawing ur5_drawing.launch.py use_fake_hardware:=true

# Process an image
cd image_processing
python3 src/main.py

# Start drawing
_START
```

## Hardware Setup

### Required Components

- **UR5 Robot Arm** with network connectivity
- **Drawing Surface** (paper/canvas) - default A4 size
- **Drawing Tool** (pen/marker) mounted to end effector (Robotiq 2f-140 Gripper)
- **Network Connection**: PC ↔ Router ↔ UR5

### Calibration

```bash
# Calibrate drawing area corners
ros2 launch ur5_drawing joint_calibration.launch.py
```

## Usage

### Available Commands - View .ur5_drawrc to see what each command is aliased to

| Command                    | Description                                                          |
| -------------------------- | -------------------------------------------------------------------- |
| `_HOME`                    | Save home position (above center of paper)                           |
| `_ORIGIN`                  | Save origin position (center of paper)                               |
| `_TL`, `_TR`, `_BL`, `_BR` | Save top left, top right, bottom left, bottom right corner positions |
| `_SAVE`                    | Save all positions to configuration file                             |

### Image Processing

1. Place images in `image_processing/images/`
2. Run `python3 src/main.py`
3. Follow interactive menu to process and export trajectories
4. Generated sequences are saved to `image_description.json`

### Drawing Process

1. Launch system with `ros2 launch ur5_drawing ur5_drawing.launch.py`
2. Ensure robot is calibrated and positioned
3. Call `_START` to begin drawing
4. Monitor progress in RViz or robot interface

## Architecture

### Core Components

- **`ur5_drawing_node`**: Main ROS2 node for robot control
- **`image_processing/`**: Computer vision pipeline
- **`ur_robot_driver/`**: UR5 communication and control
- **`ur_moveit_config/`**: Motion planning configuration

## Configuration

### Key Parameters

- **Drawing Speed**: Adjust `drawing_speed` in config
- **Paper Size**: Modify `paper_width` and `paper_height`
- **Safety Limits**: Configure velocity and acceleration constraints
- **Planning**: Customize MoveIt parameters in `move_it/` directory

### Network Setup

- Ensure PC and UR5 are on same network
  -- [Router] -> [PC] -> [UR5]
- Configure static IPs for reliable communication
- Test connectivity: `ping <robot_ip>`

## Troubleshooting

### Common Issues

1. **Network Connection**

   - Verify IP addresses in config
   - Check firewall settings
   - Ensure direct network connection

2. **Calibration Problems**

   - Re-run joint calibration tool
   - Verify corner positions are reachable
   - Check for mechanical obstructions

3. **Drawing Quality**
   - Adjust drawing speed in configuration
   - Check tool mounting and offset
   - Verify paper surface is level

### Debug Mode

```bash
# Launch with debug output
ros2 launch ur5_drawing ur5_drawing.launch.py use_fake_hardware:=true --debug

# Check service status
ros2 service list
ros2 topic list
```

## Development

### Adding Features

- **New Robot Behaviors**: Extend `ur5_drawing_node.cpp`
- **Image Processing**: Modify `image_processing/src/`
- **Configuration**: Add parameters to `drawing_config.yaml`

### Building

```bash
# Build specific package
colcon build --packages-select ur5_drawing

# Build with debug info
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```
