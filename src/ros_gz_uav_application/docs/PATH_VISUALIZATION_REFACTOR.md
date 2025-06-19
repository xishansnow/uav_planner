# 路径可视化重构总结

## 概述
本次重构实现了同时发布平滑前和平滑后路径的功能，让用户可以直观地比较路径平滑的效果。

## 主要改动

### 1. MultiScaleAStarPlanner 类改进

#### 新增成员变量
- `original_path_`: 存储平滑前的原始路径

#### 新增函数
- `getOriginalPath()`: 获取原始路径的公共接口

#### 修改的函数
- `planPath()`: 在应用平滑前保存原始路径，并添加详细的日志输出

### 2. Path 数据结构重构

#### 修改前
```cpp
struct Path {
    int planner_type;
    int pair_index;
    std::vector<geometry_msgs::msg::PoseStamped> path;  // 单一路径
};
```

#### 修改后
```cpp
struct Path {
    int planner_type;
    int pair_index;
    std::vector<geometry_msgs::msg::PoseStamped> original_path;  // 平滑前的原始路径
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;  // 平滑后的路径
};
```

### 3. 新增发布器

#### 新增的 ROS 主题
- `/original_paths`: 发布平滑前的原始路径
- `/smoothed_paths`: 发布平滑后的路径

#### 新增的可视化函数
- `publishOriginalPaths()`: 发布原始路径，使用较细的线条和较暗的颜色
- `publishSmoothedPaths()`: 发布平滑路径，使用较粗的线条和较亮的颜色

### 4. 路径存储逻辑改进

#### MultiScale A* 规划器
- 自动获取并存储原始路径和平滑路径
- 提供 `getOriginalPath()` 接口供外部访问

#### 其他规划器
- 将平滑后的路径同时存储为原始路径和平滑路径
- 保持向后兼容性

## 可视化效果

### 原始路径 (`/original_paths`)
- 线条宽度: 5.0
- 颜色: 较暗版本 (70% 亮度)
- 透明度: 0.6
- 用途: 显示规划算法的原始输出

### 平滑路径 (`/smoothed_paths`)
- 线条宽度: 15.0
- 颜色: 标准亮度
- 透明度: 1.0
- 用途: 显示经过平滑处理后的路径

### 兼容性路径 (`/paths`)
- 保持原有功能，发布平滑后的路径
- 确保向后兼容性

## 使用方法

### 在 RViz 中查看
1. 添加 MarkerArray 显示
2. 订阅 `/original_paths` 查看原始路径
3. 订阅 `/smoothed_paths` 查看平滑路径
4. 同时显示两个主题以比较效果

### 程序化访问
```cpp
// 获取原始路径
auto original_path = multiscale_planner->getOriginalPath();

// 获取平滑路径
std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
bool success = planner->planPath(start_x, start_y, start_z,
                                goal_x, goal_y, goal_z,
                                smoothed_path, SmootherType::BEZIER);
```

## 技术细节

### 路径获取机制
- MultiScale A* 规划器在 `planPath()` 中自动保存原始路径
- 其他规划器通过动态类型转换获取原始路径
- 如果无法获取原始路径，则使用平滑路径作为原始路径

### 性能考虑
- 原始路径存储不会显著影响性能
- 可视化函数使用不同的线条样式区分路径类型
- 保持原有的发布频率和数据结构

## 未来扩展

### 可能的改进
1. 为其他规划器添加原始路径获取接口
2. 添加路径质量评估指标
3. 支持多种平滑算法的比较
4. 添加路径动画播放功能

### 接口标准化
- 考虑在基类中添加 `getOriginalPath()` 虚函数
- 统一所有规划器的原始路径访问接口
- 添加路径版本管理功能 