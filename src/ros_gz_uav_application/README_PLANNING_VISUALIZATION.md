# PlanningTestNode 可视化配置

这个配置文件专门用于可视化 `PlanningTestNode` 发布的 octomap 和 markerarray 数据。

## 发布的话题

### PlanningTestNode 发布的话题：

1. **Octomap 话题**：
   - `/octomap_full` - 完整的 octomap 数据
   - `/octomap_binary` - 二进制 octomap 数据（用于 rviz2 显示）

2. **分离的 MarkerArray 话题**（推荐使用）：
   - `/environment_bounds` - 环境边界框和线框
   - `/obstacles` - 障碍物（立方体、圆柱体、球体）
   - `/voxel_grid` - 体素网格（占用体素和网格线）
   - `/start_goal_points` - 起点和目标点
   - `/paths` - 规划路径

3. **组合的 MarkerArray 话题**（向后兼容）：
   - `/planning_visualization` - 包含所有可视化标记的组合话题

4. **TF 变换**：
   - `world` -> `map` - 世界坐标系到地图坐标系的变换

## 使用方法

### 方法1：使用分离的可视化配置（推荐）

```bash
# 启动 PlanningTestNode 和 rviz2（分离话题）
ros2 launch ros_gz_uav_application planning_test_separated.launch.py
```

### 方法2：使用组合的可视化配置

```bash
# 启动 PlanningTestNode 和 rviz2（组合话题）
ros2 launch ros_gz_uav_application planning_test_visualization.launch.py
```

### 方法3：手动启动

```bash
# 终端1：启动 PlanningTestNode
ros2 run ros_gz_uav_application test_3d_planning

# 终端2：启动 rviz2（分离话题）
ros2 run rviz2 rviz2 -d src/ros_gz_uav_application/config/planning_test_separated.rviz

# 或者启动 rviz2（组合话题）
ros2 run rviz2 rviz2 -d src/ros_gz_uav_application/config/planning_test_simple.rviz
```

### 方法4：调试TF问题

如果遇到"frame [map] does not exist"错误，可以使用以下方法调试：

```bash
# 终端1：启动 PlanningTestNode
ros2 run ros_gz_uav_application test_3d_planning

# 终端2：检查TF变换
ros2 run tf2_tools view_frames

# 终端3：使用调试脚本检查TF
python3 src/ros_gz_uav_application/test_tf_debug.py

# 终端4：启动 rviz2
ros2 run rviz2 rviz2 -d src/ros_gz_uav_application/config/planning_test_separated.rviz
```

## 可视化内容

### 1. Octomap 显示
- **话题**：`/octomap_binary`
- **显示内容**：3D 占用栅格地图
- **颜色编码**：按 Z 轴高度着色
- **透明度**：0.8（可调整）
- **高度范围**：0.0 - 5000.0 米

### 2. 环境边界
- **话题**：`/environment_bounds`
- **显示内容**：半透明边界框和线框
- **颜色**：灰色

### 3. 障碍物
- **话题**：`/obstacles`
- **显示内容**：几何形状障碍物
- **类型和颜色**：
  - 立方体（BOX）：红色
  - 圆柱体（CYLINDER）：绿色
  - 球体（SPHERE）：蓝色
- **透明度**：0.7

### 4. 占用体素
- **话题**：`/voxel_grid`
- **显示内容**：红色立方体表示占用的体素
- **大小**：略小于实际体素尺寸（0.9倍）
- **透明度**：0.6

### 5. 起点和目标点
- **话题**：`/start_goal_points`
- **显示内容**：球体标记
- **起点颜色**：绿色
- **目标点颜色**：红色
- **大小**：50.0 米半径

### 6. 规划路径
- **话题**：`/paths`
- **显示内容**：线带（LINE_STRIP）
- **颜色编码**：
  - 红色：A* 算法
  - 绿色：Theta* 算法
  - 蓝色：ARA* 算法
  - 青色：JPS 算法
- **线宽**：10.0 米

### 7. 体素网格线框
- **话题**：`/voxel_grid`
- **显示内容**：浅灰色网格线
- **间距**：每10个体素一条线
- **透明度**：0.3

### 8. TF 变换
- **显示内容**：坐标轴和变换关系
- **帧**：`map` 和 `world`

## 分离可视化的优势

使用分离的话题有以下优势：

1. **独立控制**：可以在rviz2中独立开启/关闭每种类型的可视化
2. **性能优化**：只显示需要的内容，减少渲染负担
3. **调试便利**：可以单独查看障碍物、路径、体素等
4. **自定义显示**：可以根据需要组合不同的可视化元素

## 视图设置

### 默认视角
- **距离**：15000.0 米
- **焦点**：(5000.0, 5000.0, 2500.0)
- **俯仰角**：45° (0.7853981852531433)
- **偏航角**：45° (0.7853981852531433)

### 环境尺寸
- **世界尺寸**：10000.0 x 10000.0 x 5000.0 米
- **体素网格**：100 x 100 x 100
- **XY 分辨率**：100.0 米
- **Z 分辨率**：50.0 米

## 故障排除

### "frame [map] does not exist" 错误

这个错误通常是因为TF变换没有正确发布。解决方案：

1. **检查PlanningTestNode是否正在运行**：
   ```bash
   ros2 node list
   ```

2. **检查TF变换是否发布**：
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. **使用调试脚本检查TF**：
   ```bash
   python3 src/ros_gz_uav_application/test_tf_debug.py
   ```

4. **手动发布TF变换**（如果PlanningTestNode没有发布）：
   ```bash
   ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
   ```

5. **检查rviz2的Fixed Frame设置**：
   - 确保Fixed Frame设置为"world"
   - 如果仍然有问题，尝试设置为"map"

### 看不到障碍物体素表示

如果看不到障碍物的体素表示，请检查：

1. **确认占用体素已启用**：
   - 在rviz2中确保"Voxel Grid"显示已启用
   - 检查话题`/voxel_grid`是否正确订阅

2. **检查障碍物是否正确添加到环境**：
   - 查看PlanningTestNode的日志输出
   - 应该看到"Published voxel grid: X occupied voxels out of Y total"

3. **调整视角**：
   - 障碍物可能很小，需要放大查看
   - 尝试调整rviz2的缩放和视角

4. **检查体素大小**：
   - 当前体素大小：XY=100.0m, Z=50.0m
   - 如果体素太大，可能看不到细节

### 看不到数据
1. 检查 PlanningTestNode 是否正在运行
2. 使用 `ros2 topic list` 检查话题是否存在
3. 使用 `ros2 topic echo` 检查数据是否正在发布

### 坐标变换问题
1. 确保 TF 变换正确发布
2. 检查 Fixed Frame 是否设置为 "world"
3. 使用 `ros2 run tf2_tools view_frames` 查看变换树

### 性能问题
1. 减少 octomap 的最大深度（Max. Octree Depth）
2. 调整体素网格大小
3. 减少可视化更新频率

## 自定义配置

### 修改话题名称
如果需要修改话题名称，可以编辑 `planning_test_separated.rviz` 文件中的相应部分。

### 调整显示参数
- **Octomap 透明度**：修改 `Voxel Alpha` 值
- **高度显示范围**：修改 `Min. Height Display` 和 `Max. Height Display`
- **标记大小**：在 PlanningTestNode 代码中修改相应的 scale 值

### 添加新的可视化元素
1. 在 PlanningTestNode 中添加新的 marker 发布
2. 在 rviz 配置文件中添加对应的 MarkerArray 显示
3. 设置适当的命名空间和话题

## 示例输出

启动后，你应该能看到：
1. 3D 环境中的障碍物（octomap 显示）
2. 环境边界框
3. 几何形状障碍物（立方体、圆柱体、球体）
4. 红色占用体素（表示障碍物占用的体素）
5. 多个起点（绿色球体）和目标点（红色球体）
6. 不同颜色的规划路径
7. 浅灰色网格线框
8. 坐标轴显示

这个配置提供了完整的 3D 路径规划可视化，包括障碍物的体素表示，便于调试和分析算法性能。

---

# GlobalPathPlannerNode 更新说明

## 更新概述

参照 `PlanningTestNode` 的实现，对 `GlobalPathPlannerNode` 进行了以下重要更新：

### 1. 分离的可视化发布器

添加了多个独立的可视化发布器，可以分别控制不同类型的可视化：

- `/environment_bounds` - 环境边界
- `/voxel_grid` - 体素网格（占用体素）
- `/start_goal_points` - 起点和目标点
- `/paths` - 规划路径
- `/visualization_markers` - 组合的可视化标记（向后兼容）

### 2. Octomap 发布

添加了 octomap 发布功能：
- `/octomap` - 完整的 octomap 数据
- `/binary_octomap` - 二进制 octomap 数据（用于 rviz2 显示）

### 3. 静态TF发布

添加了静态TF变换发布器，发布 `world` 到 `map` 的变换。

### 4. 环境参数配置

增加了详细的环境参数配置：
- `env_width`, `env_height`, `env_depth` - 环境尺寸
- `voxel_width`, `voxel_height`, `voxel_depth` - 体素网格尺寸
- 自动计算 XY 和 Z 轴分辨率

### 5. 可视化定时器

添加了可视化定时器，每5秒发布一次可视化数据。

## 使用方法

### 启动 GlobalPathPlannerNode 和可视化

```bash
# 使用launch文件启动（推荐）
ros2 launch ros_gz_uav_application global_path_planner_visualization.launch.py

# 手动启动
# 终端1：启动 GlobalPathPlannerNode
ros2 run ros_gz_uav_application global_path_planner_node

# 终端2：启动 rviz2
ros2 run rviz2 rviz2 -d src/ros_gz_uav_application/config/global_path_planner.rviz
```

### 配置参数

可以通过launch文件或参数文件配置以下参数：

```yaml
global_path_planner_node:
  ros__parameters:
    planner_type: "astar"           # 规划器类型：astar, thetastar, arastar, jps
    planner_frequency: 1.0          # 规划频率
    max_planning_time: 5.0          # 最大规划时间
    goal_tolerance: 0.5             # 目标容差
    robot_radius: 0.3               # 机器人半径
    map_resolution: 0.1             # 地图分辨率
    origin_x: -100.0                # 原点X坐标
    origin_y: -100.0                # 原点Y坐标
    origin_z: 0.0                   # 原点Z坐标
    env_width: 200.0                # 环境宽度
    env_height: 200.0               # 环境高度
    env_depth: 100.0                # 环境深度
    voxel_width: 200                # 体素网格宽度
    voxel_height: 200               # 体素网格高度
    voxel_depth: 100                # 体素网格深度
```

## 可视化内容

### 1. 环境边界
- **话题**：`/environment_bounds`
- **显示内容**：半透明边界框
- **颜色**：灰色，透明度0.1

### 2. 占用体素
- **话题**：`/voxel_grid`
- **显示内容**：红色立方体表示占用的体素
- **大小**：略小于实际体素尺寸（0.9倍）
- **透明度**：0.6

### 3. 起点和目标点
- **话题**：`/start_goal_points`
- **显示内容**：球体标记
- **起点颜色**：绿色
- **目标点颜色**：红色
- **大小**：2.0 米半径

### 4. 规划路径
- **话题**：`/paths`
- **显示内容**：线带（LINE_STRIP）
- **颜色**：蓝色
- **线宽**：1.0 米

### 5. 计划路径（ROS Path）
- **话题**：`/planned_path`
- **显示内容**：标准的ROS Path消息
- **颜色**：绿色

### 6. Octomap
- **话题**：`/binary_octomap`
- **显示内容**：3D 占用栅格地图
- **颜色编码**：按 Z 轴高度着色

## 主要改进

### 1. 更好的可视化控制
- 可以在rviz2中独立控制每种可视化类型
- 支持实时显示机器人位置、目标位置和规划路径

### 2. 更灵活的环境配置
- 支持自定义环境尺寸和体素分辨率
- 支持不同的XY和Z轴分辨率

### 3. 更好的调试支持
- 实时显示体素网格状态
- 支持octomap可视化
- 自动发布TF变换

### 4. 向后兼容性
- 保留了原有的功能
- 添加了组合的可视化话题用于向后兼容

## 与 PlanningTestNode 的区别

| 特性 | PlanningTestNode | GlobalPathPlannerNode |
|------|------------------|----------------------|
| 节点类型 | 普通节点 | 生命周期节点 |
| 主要功能 | 测试规划算法 | 实际路径规划服务 |
| 障碍物生成 | 随机生成几何障碍物 | 从传感器数据更新 |
| 可视化频率 | 5秒 | 5秒 |
| 环境大小 | 10km x 10km x 5km | 200m x 200m x 100m |
| 体素分辨率 | 100m x 100m x 50m | 1m x 1m x 1m |

## 故障排除

### 常见问题

1. **看不到可视化数据**：
   - 确保节点已激活（使用 `ros2 lifecycle set /global_path_planner_node activate`）
   - 检查话题是否正确发布（使用 `ros2 topic list`）

2. **TF变换错误**：
   - 确保Fixed Frame设置为"world"
   - 检查TF变换是否正确发布

3. **体素显示问题**：
   - 调整rviz2的缩放和视角
   - 检查体素大小是否合适

4. **性能问题**：
   - 减少体素网格大小
   - 调整可视化更新频率

## 未来改进

1. **传感器数据集成**：
   - 从激光雷达和点云数据更新环境
   - 实时障碍物检测和更新

2. **动态可视化**：
   - 支持实时路径更新
   - 支持动态障碍物显示

3. **更多可视化选项**：
   - 支持不同的颜色主题
   - 支持自定义标记样式

4. **性能优化**：
   - 支持选择性可视化
   - 支持LOD（细节层次）显示 