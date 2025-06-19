# ros_gz_uav_application

## 目录结构

```
ros_gz_uav_application/
├── src/                # 源码目录（包含各类算法、节点、环境等子模块）
│   ├── nodes/              # ROS2 节点实现（如 test_3d_planning、global_path_planner_node 等）
│   ├── single_global/      # 全局单机径规划算法实现
│   ├── smoother/           # 路径平滑器的相关模块
│   ├── voxelization/       # 体素化相关模块（暂时未开发调试）
│   └── env/                # 体素环境建模与处理
├── include/           # 头文件目录（与 src 结构对应）
├── launch/            # 启动文件（launch.py）
├── config/            # 配置文件与 RViz 配置
├── scripts/           # 辅助脚本（如性能测试等）
├── test/              # 单元测试与调试脚本
├── docs/              # 设计与开发文档
├── CMakeLists.txt     # 构建配置
├── package.xml        # ROS2 包描述文件
├── README.md          # 主说明文档
├── README_*.md        # 其他功能/算法说明文档
```

## 简介

`ros_gz_uav_application` 是一个基于 ROS 2 和 Octomap 的三维无人机路径规划与环境感知应用包，支持多种全局路径规划算法（A*、Theta*、ARA*、多尺度A*），并集成了丰富的可视化与调试工具，适用于复杂三维环境下的无人机导航、仿真与算法研究。

## 主要功能

- 支持 A*、Theta*、ARA*、多尺度A* 等多种三维路径规划算法
- 基于 Octomap 的三维环境建模与体素化
- 多种路径与环境可视化 Marker 支持，便于 RViz 调试
- 支持自定义环境、障碍物、起终点配置
- 丰富的参数与算法可调节性

## 依赖

- ROS 2 (rclcpp, geometry_msgs, nav_msgs, sensor_msgs, tf2, nav2 等)
- Octomap & octomap_msgs
- yaml-cpp
- pluginlib, angles
- ament_cmake

详见 `package.xml`。

## 安装与编译

```bash
cd ~/uav_planner_ws
colcon build --packages-select ros_gz_uav_application
source install/setup.bash
```

## 使用方法

### 启动全局路径规划器

注意：下面的方法并没有调试，建议先直接修改 src/nodes/test_3d_planning.hpp 和 .cpp 文件，并调试和编译运行。test_3d_planning 里定义了一个 ROS2 节点，可以完成环境初始化、障碍物生成、路径规划、体素环境更新、topic 发布等功能。

VSCode 下的软件断点调试需要配置 .vscode 目录下的 launch.json （已配置），并且编译要使用 `-DCMAKE_BUILD_TYPE=Debug`  选项。

```
 colcon build --packages-select ros_gz_uav_application  -DCMAKE_BUILD_TYPE=Debug 
```

直接按 F5 快捷键即可启动运行调试。

<!-- ```bash
# 默认使用 A* 算法
ros2 launch ros_gz_uav_application global_planner.launch.py

# 使用 Theta* 算法
ros2 launch ros_gz_uav_application global_planner.launch.py planner_type:=thetastar

# 使用 ARA* 算法
ros2 launch ros_gz_uav_application global_planner.launch.py planner_type:=arastar
``` -->

<!-- ### 启动 3D 规划测试与可视化

```bash
ros2 launch ros_gz_uav_application planning_test_separated.launch.py
``` -->

### RViz 可视化

- 根据 test_3d_planning 的发布内容定义的 RViz 配置，见 `config/planning_test_separated.rviz`
- 支持 Octomap 体素图、障碍物、体素网格线、起终点、路径等多种可视化元素

## 主要节点与 topic 

- `test_3d_planning`：三维路径规划测试节点
- `global_path_planner_node`：全局路径规划主节点（暂时未开发）
- `octomap_publisher_node`：Octomap 地图发布节点（暂时未开发）

**主要 topic ：**
- `/octomap_binary`：三维占用栅格地图
- `/environment_bounds`、`/obstacles`、`/voxel_grid`、`/start_goal_points`、`/paths`：可视化 Marker
- `/global_path`：规划路径
- `/goal_pose`：目标点输入

## 支持的算法

- **A\***：最优路径，适合中等规模环境
- **Theta\***：任意角度路径，路径更短更平滑
- **ARA\***：Anytime Repairing A*，可快速获得次优解并逐步优化
- **多尺度A\***：利用 Octomap 多分辨率结构，适合稀疏/大规模环境，显著提升效率

## 参数与配置

- `planner_type`：选择算法（astar, thetastar, arastar, multiscale_astar）
- `octomap_file`、`environment_file`：自定义环境与地图
- 详见各 launch 文件与算法 README

## 可视化与调试

- 支持分离/组合 Marker  topic ，便于 RViz 独立控制各类可视化元素
- 提供 TF 调试脚本与常见问题排查方法
- 支持路径、体素、障碍物、环境边界等多种可视化

## 常见问题

- "frame [map] does not exist"：请检查 TF 发布与 RViz Fixed Frame 设置
- 路径未显示/无解：检查起终点是否在障碍物内，环境配置是否正确
- 体素/障碍物不可见：检查 topic 订阅与 RViz 显示设置

## 参考文档

- [README_PLANNING_VISUALIZATION.md](./README_PLANNING_VISUALIZATION.md)
- [README_GLOBAL_PLANNERS.md](./README_GLOBAL_PLANNERS.md)
- [README_MULTISCALE_ASTAR.md](./README_MULTISCALE_ASTAR.md)
- [README_Z_RESOLUTION.md](./README_Z_RESOLUTION.md)
- [README_OCTOMAP_VISUALIZATION.md](./README_OCTOMAP_VISUALIZATION.md)
- [README_octomap_improvement.md](./README_octomap_improvement.md)

