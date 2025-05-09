# Turtle Work Project

This is a ROS (Robot Operating System) workspace for turtle simulation and control. The project is designed to run on Ubuntu with ROS environment.

## Project Structure

```
.
├── build/          # Build directory for ROS packages
├── install/        # Installation directory for ROS packages
├── log/           # ROS log files
└── src/           # Source code directory
    └── turtle_work/  # Main package directory
        ├── resource/  # Resource files
        ├── test/     # Test files
        ├── package.xml  # Package manifest
        ├── setup.cfg   # Setup configuration
        └── setup.py    # Python setup file
```

## Prerequisites

- Ubuntu 22.04


## Setup Instructions

1. Clone this repository to your workspace:
```bash
git clone [repository-url] turtle_homework
cd turtle_homework
```

2. Initialize the workspace:
```bash
source /opt/ros/humble/setup.bash  # For ROS 2 Humble
```

3. Build the workspace:
```bash
colcon build
```

4. Source the workspace:
```bash
source install/setup.bash
```

5. run node
```bash
ros2 run turtle_work turtle_node
```
