# Octomap 可视化指南

本指南说明如何在 rviz2 中显示 octomap，包括环境、障碍物和路径规划结果。

## 功能概述

我们提供了两个主要的 octomap 发布器：

1. **`octomap_publisher_node`** - 简单的 octomap 发布器
2. **`test_3d_planning`** - 完整的 3D 路径规划测试，包含 octomap 发布功能

## 快速开始

### 1. 使用 3D 规划测试（推荐）

```bash
# 启动 3D 规划测试和 rviz2
ros2 launch ros_gz_uav_application test_3d_planning_with_octomap.launch.py
```

这将：
- 创建一个 10km x 10km x 2km 的大型环境
- 随机生成 3 个不同形状的障碍物（球体、圆柱体、正方体）
- 生成 10 个随机的起点-终点对
- 使用 A*、Theta* 和 ARA* 算法进行路径规划
- 在 rviz2 中显示环境、障碍物、起点终点和规划路径

### 2. 使用简单的 octomap 发布器

```bash
# 启动 octomap 发布器和 rviz2
ros2 launch ros_gz_uav_application octomap_visualization.launch.py
```

这将：
- 创建一个 1024x1024x200 的测试环境（分辨率 0.1m）
- 添加三个随机分布的障碍物（球体、圆柱体、正方体）
- 启动 rviz2 并显示 octomap

### 3. 从文件加载 octomap

```bash
# 启动并指定 octomap 文件
ros2 launch ros_gz_uav_application octomap_visualization.launch.py \
    octomap_file:=/path/to/your/octomap.bt
```

## 发布的话题

### 3D 规划测试节点发布的话题：

- `/octomap_full` - 完整的 octomap 消息
- `/octomap_binary` - 二进制 octomap 消息（推荐用于 rviz2）
- `/planning_visualization` - 可视化标记数组（环境、障碍物、路径等）

### 简单 octomap 发布器发布的话题：

- `/octomap_full` - 完整的 octomap 消息
- `/octomap_binary` - 二进制 octomap 消息（推荐用于 rviz2）

## RViz2 配置

启动后，rviz2 会自动加载配置，显示：

1. **Octomap** - 3D 占用栅格地图
2. **Grid** - 参考网格
3. **TF** - 坐标变换
4. **MarkerArray** - 可视化标记（仅 3D 规划测试）

### 手动配置 RViz2

如果自动配置不工作，可以手动添加：

1. 添加 **Octomap** 显示类型
2. 设置话题为 `/octomap_binary`
3. 设置 Fixed Frame 为 `map`
4. 调整渲染选项：
   - **Octree Rendering**: Occupied Cells
   - **Point Size**: 根据需要调整
   - **Alpha**: 透明度

## 3D 规划测试功能

### 环境设置

- **世界尺寸**: 10km x 10km x 2km
- **体素网格**: 100 x 100 x 100
- **XY 分辨率**: 100m
- **Z 分辨率**: 20m

### 障碍物生成

- **数量**: 3 个随机障碍物
- **类型**: 球体、圆柱体、正方体
- **尺寸**: 50-150m（XY），25-100m（Z）
- **分布**: 随机分布在环境中

### 路径规划

- **算法**: A*、Theta*、ARA*
- **起点-终点对**: 10 个随机生成的对
- **可视化**: 不同颜色表示不同算法
  - 红色: A*
  - 绿色: Theta*
  - 蓝色: ARA*

## 参数说明

### 3D 规划测试参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_sim_time` | `false` | 是否使用仿真时间 |

### 简单 octomap 发布器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `octomap_file` | `""` | Octomap .bt 文件路径 |
| `environment_file` | `""` | 环境配置文件路径（可选） |
| `publish_frequency` | `1.0` | 发布频率（Hz） |
| `frame_id` | `"map"` | 坐标系帧 ID |
| `octomap_topic` | `"octomap_full"` | 完整 octomap 话题名 |
| `binary_octomap_topic` | `"octomap_binary"` | 二进制 octomap 话题名 |

## 故障排除

### 1. 看不到 octomap

- 检查话题是否正确发布：`ros2 topic list | grep octomap`
- 检查消息是否发布：`ros2 topic echo /octomap_binary --once`
- 确认 Fixed Frame 设置正确

### 2. 坐标变换问题

- 检查 TF 树：`ros2 run tf2_tools view_frames`
- 确认 map 帧存在：`ros2 topic echo /tf_static`

### 3. 性能问题

- 降低发布频率：`publish_frequency:=0.5`
- 调整 rviz2 中的点大小和渲染选项

### 4. 内存问题

- 如果遇到内存不足，可以减小环境尺寸
- 调整体素分辨率以平衡精度和性能

## 集成到其他系统

要将 octomap 发布器集成到其他启动文件中：

```python
# 在其他 launch 文件中添加
planning_test_node = Node(
    package='ros_gz_uav_application',
    executable='test_3d_planning',
    name='planning_test_node',
    parameters=[{
        'use_sim_time': False
    }]
)
```

## 扩展功能

节点支持以下扩展：

1. **动态更新**: 可以通过服务或话题动态更新 octomap
2. **多分辨率**: 支持不同的 XY 和 Z 分辨率
3. **自定义环境**: 可以加载自定义的 octomap 文件
4. **实时可视化**: 支持实时传感器数据更新
5. **路径规划**: 集成多种路径规划算法
6. **3D 可视化**: 完整的 3D 环境可视化 