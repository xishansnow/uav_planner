# 3D Path Planning Test

这个测试程序演示了在大型3D体素环境中使用不同路径规划算法的性能。

## 测试环境

- **环境大小**: 10 km × 10 km × 2 km
- **体素网格**: 1024 × 1024 × 20
- **XY分辨率**: ~9.77 m
- **Z分辨率**: 100 m
- **障碍物**: 10个不同形状的随机障碍物（立方体、圆柱体、球体）
- **测试用例**: 10组随机起点-终点对

## 功能特性

1. **环境生成**: 创建大型3D体素环境，支持不同Z轴分辨率
2. **障碍物生成**: 随机生成10个不同形状的障碍物
3. **路径规划**: 使用A*、Theta*、ARA*三种算法进行路径规划
4. **性能测试**: 测量每种算法的规划时间和成功率
5. **3D可视化**: 在RViz中可视化环境、障碍物和路径

## 编译

```bash
# 在工作空间根目录
colcon build --cmake-args -DBUILD_TESTING=ON
```

## 运行测试

### 方法1: 使用启动文件（推荐）

```bash
# 启动测试程序和RViz可视化
ros2 launch ros_gz_uav_application test_3d_planning.launch.py
```

### 方法2: 分别运行

```bash
# 终端1: 运行测试程序
ros2 run ros_gz_uav_application test_3d_planning

# 终端2: 运行RViz
rviz2 -d src/ros_gz_uav_application/config/test_3d_planning.rviz
```

## 可视化说明

在RViz中，你可以看到：

- **灰色半透明立方体**: 环境边界
- **红色立方体**: 立方体障碍物
- **绿色圆柱体**: 圆柱体障碍物  
- **蓝色球体**: 球体障碍物
- **绿色小球**: 起点
- **红色小球**: 终点
- **彩色线条**: 路径
  - 红色: A*算法路径
  - 绿色: Theta*算法路径
  - 蓝色: ARA*算法路径

## 输出信息

程序会在控制台输出：

1. **环境设置信息**: 环境大小、分辨率等
2. **障碍物信息**: 位置、大小、形状
3. **规划结果**: 每种算法对每组起点-终点的规划结果
   - 成功/失败状态
   - 路径长度
   - 规划时间

## 算法比较

- **A***: 经典的最优路径搜索算法
- **Theta***: 基于视线检查的路径平滑算法
- **ARA***: 任意时间重规划算法，支持动态调整启发式权重

## 注意事项

1. **内存使用**: 大型环境可能需要较多内存
2. **计算时间**: 首次规划可能需要较长时间
3. **可视化**: 确保RViz正确订阅了`/planning_visualization`话题

## 故障排除

### RViz中看不到可视化内容

如果RViz中看不到环境、障碍物或路径，请按以下步骤检查：

#### 1. 检查话题是否正确发布
```bash
# 检查话题是否存在
ros2 topic list | grep planning_visualization

# 检查话题是否有数据
ros2 topic echo /planning_visualization --once

# 使用检查脚本
python3 src/ros_gz_uav_application/test/check_visualization.py
```

#### 2. 检查RViz配置
- 确保RViz订阅了 `/planning_visualization` 话题
- 确保所有命名空间都已启用（environment, obstacles, start_points, goal_points, paths, test）
- 确保Fixed Frame设置为 `map`

#### 3. 检查程序输出
运行测试程序时，应该看到类似输出：
```
[INFO] [planning_test_node]: Published visualization markers: X markers
[INFO] [planning_test_node]:   - Environment bounds: 1 marker
[INFO] [planning_test_node]:   - Obstacles: 10 markers
[INFO] [planning_test_node]:   - Start/Goal points: 20 markers
[INFO] [planning_test_node]:   - Paths: Y markers
```

#### 4. 手动测试可视化
```bash
# 终端1：运行测试程序
ros2 run ros_gz_uav_application test_3d_planning

# 终端2：检查话题
ros2 topic echo /planning_visualization --once

# 终端3：运行RViz
rviz2 -d src/ros_gz_uav_application/config/test_3d_planning.rviz
```

#### 5. 常见问题
- **看不到任何内容**：检查Fixed Frame是否为 `map`
- **只看到部分内容**：检查RViz中的命名空间是否全部启用
- **路径不显示**：确保路径规划成功，检查控制台输出
- **障碍物不显示**：检查障碍物生成是否成功

## 自定义参数

你可以修改`test_3d_planning.cpp`中的以下参数：

```cpp
// 环境大小
env_width_ = 10000.0;  // 10 km
env_height_ = 10000.0; // 10 km  
env_depth_ = 2000.0;   // 2 km

// 体素网格大小
voxel_width_ = 1024;
voxel_height_ = 1024;
voxel_depth_ = 20;

// 障碍物数量
for (int i = 0; i < 10; ++i)  // 10个障碍物

// 起点-终点对数量
for (int i = 0; i < 10; ++i)  // 10组测试
``` 