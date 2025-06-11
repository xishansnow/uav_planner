# UAV Global Path Planners

This package implements three different global path planning algorithms for UAV navigation in 3D voxel environments using octomap.

## Algorithms

### 1. A* (A-Star)
- **Type**: Optimal path planning
- **Features**: 
  - Guarantees optimal path (shortest distance)
  - Uses heuristic guidance for efficiency
  - 26-connected 3D grid search
  - Suitable for real-time planning in moderate-sized environments

### 2. Theta* (Theta-Star)
- **Type**: Any-angle path planning
- **Features**:
  - Finds shorter paths than grid-based A*
  - Uses line-of-sight optimization
  - Reduces path length by connecting non-adjacent nodes
  - Better for UAVs that can fly at any angle

### 3. ARA* (Anytime Repairing A*)
- **Type**: Anytime path planning
- **Features**:
  - Provides suboptimal solutions quickly
  - Improves solution quality over time
  - Adjustable inflation factor (epsilon)
  - Suitable for real-time applications with time constraints

## Usage

### Building
```bash
cd ~/uav_planner_ws
colcon build --packages-select ros_gz_uav_application
source install/setup.bash
```

### Running with Default Settings
```bash
# Run with A* planner (default)
ros2 launch ros_gz_uav_application global_planner.launch.py

# Run with Theta* planner
ros2 launch ros_gz_uav_application global_planner.launch.py planner_type:=thetastar

# Run with ARA* planner
ros2 launch ros_gz_uav_application global_planner.launch.py planner_type:=arastar
```

### Running with Custom Environment
```bash
ros2 launch ros_gz_uav_application global_planner.launch.py \
  planner_type:=astar \
  octomap_file:=/path/to/environment.bt \
  environment_file:=/path/to/environment.yaml
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `planner_type` | string | "astar" | Planner algorithm (astar, thetastar, arastar) |
| `octomap_file` | string | "" | Path to octomap file (.bt) |
| `environment_file` | string | "" | Path to environment file (.yaml) |
| `max_planning_time` | double | 5.0 | Maximum planning time in seconds |
| `publish_path` | bool | true | Whether to publish path |
| `publish_markers` | bool | true | Whether to publish visualization markers |

### Topics

#### Subscribed Topics
- `/goal_pose` (geometry_msgs/PoseStamped): Goal position for path planning

#### Published Topics
- `/global_path` (nav_msgs/Path): Planned path
- `/path_markers` (visualization_msgs/MarkerArray): Visualization markers

#### Services
- `/plan_path` (std_srvs/Trigger): Trigger path planning

## Algorithm Comparison

| Algorithm | Optimality | Speed | Path Quality | Memory Usage |
|-----------|------------|-------|--------------|--------------|
| A* | Optimal | Medium | High | Medium |
| Theta* | Suboptimal | Fast | Very High | Low |
| ARA* | Anytime | Fast → Slow | Medium → High | Medium |

## Configuration

### A* Parameters
- `heuristic_weight`: Weight for heuristic function (default: 1.0)
- `diagonal_cost`: Cost for diagonal moves (default: 1.414)
- `vertical_cost`: Cost for vertical moves (default: 1.0)

### Theta* Parameters
- `heuristic_weight`: Weight for heuristic function (default: 1.0)
- `diagonal_cost`: Cost for diagonal moves (default: 1.414)
- `vertical_cost`: Cost for vertical moves (default: 1.0)
- `use_lazy_thetastar`: Use lazy Theta* variant (default: false)

### ARA* Parameters
- `initial_epsilon`: Initial inflation factor (default: 3.0)
- `final_epsilon`: Final inflation factor (default: 1.0)
- `epsilon_decrease`: Epsilon decrease rate (default: 0.5)
- `heuristic_weight`: Weight for heuristic function (default: 1.0)
- `diagonal_cost`: Cost for diagonal moves (default: 1.414)
- `vertical_cost`: Cost for vertical moves (default: 1.0)

## Environment Format

### Octomap File (.bt)
Binary octomap file containing 3D occupancy information.

### Environment File (.yaml)
```yaml
resolution: 0.1
bounds:
  min_x: -10.0
  max_x: 10.0
  min_y: -10.0
  max_y: 10.0
  min_z: 0.0
  max_z: 5.0
```

## Visualization

The package includes RViz configuration for visualizing:
- Planned path (green line)
- Start position (blue sphere)
- Goal position (red sphere)
- Environment grid

## Testing

### Interactive Testing
1. Launch the planner:
   ```bash
   ros2 launch ros_gz_uav_application global_planner.launch.py
   ```

2. In RViz, use the "2D Goal Pose" tool to set goal positions

3. The planner will automatically compute and display the path

### Service Testing
```bash
# Trigger path planning via service
ros2 service call /plan_path std_srvs/srv/Trigger
```

### Topic Testing
```bash
# Publish a goal pose
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
  stamp:
    sec: 0
    nanosec: 0
pose:
  position:
    x: 5.0
    y: 3.0
    z: 1.0
  orientation:
    w: 1.0
"
```

## Performance Tips

1. **For Real-time Applications**: Use ARA* with high initial epsilon
2. **For Optimal Paths**: Use A* with admissible heuristic
3. **For Smooth Paths**: Use Theta* for better path quality
4. **For Large Environments**: Reduce resolution or use ARA* with time limits

## Troubleshooting

### Common Issues

1. **No path found**: Check if start/goal positions are occupied
2. **Slow planning**: Reduce environment resolution or use ARA*
3. **Memory issues**: Use Theta* or reduce environment size
4. **Path not smooth**: Use Theta* or enable path smoothing

### Debug Information
Enable debug output by setting log level:
```bash
ros2 run ros_gz_uav_application global_path_planner_node --ros-args --log-level DEBUG
```

## Contributing

To add new planning algorithms:
1. Inherit from `GlobalPlannerBase`
2. Implement required virtual functions
3. Add to `PlannerFactory`
4. Update documentation

## License

This package is part of the UAV Planner workspace and follows the same license terms. 

## File Structure

```
src/ros_gz_uav_application/src/
├── global_planner_base.hpp/cpp          # 基类
├── astar_planner.hpp/cpp                # A*算法
├── thetastar_planner.hpp/cpp            # Theta*算法
├── arastar_planner.hpp/cpp              # ARA*算法
├── planner_factory.hpp/cpp              # 工厂类
├── global_path_planner_node.cpp         # ROS2节点
└── environment_voxel3d.cpp              # 环境类

launch/
└── global_planner.launch.py             # 启动文件

config/
└── global_planner.rviz                  # RViz配置 