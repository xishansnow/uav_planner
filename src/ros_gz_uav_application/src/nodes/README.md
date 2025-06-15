# Nodes Directory

This directory contains ROS node executables for the UAV path planning application.

## Current Nodes

### test_3d_planning
- **File**: `test_3d_planning.cpp`, `test_3d_planning.hpp`
- **Purpose**: 3D path planning test node that demonstrates the usage of different path planning algorithms
- **Features**:
  - Tests A*, Theta*, and ARA* path planning algorithms
  - Demonstrates path smoothing with Bezier, B-spline, and MINVO curves
  - Provides visualization and testing capabilities
  - Uses the new smoother module for path post-processing

## Directory Structure

```
src/nodes/
├── README.md                    # This file
├── test_3d_planning.cpp         # Main test node implementation
└── test_3d_planning.hpp         # Test node header file
```

## Usage

To run the test node:

```bash
# Build the package
colcon build --packages-select ros_gz_uav_application

# Source the workspace
source install/setup.bash

# Run the test node
ros2 run ros_gz_uav_application test_3d_planning
```

## Future Nodes

Additional nodes can be added to this directory as the project grows, such as:
- `global_planner_node` - Main global path planning node
- `path_smoother_node` - Standalone path smoothing node
- `visualization_node` - Path visualization node
- `mission_planner_node` - High-level mission planning node

## Integration with Smoother Module

The nodes in this directory utilize the smoother module (`../smoother/`) for path post-processing, demonstrating the decoupled architecture where planning and smoothing are separate concerns. 