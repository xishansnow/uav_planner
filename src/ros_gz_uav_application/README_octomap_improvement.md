# Octomap-based Environment Implementation

## 问题描述

原始的哈希表实现在大型环境中存在严重的内存问题：

- 哈希表大小 = width × height × depth
- 对于 1000×1000×100 的环境，哈希表大小达到 100,000,000
- 每个哈希桶占用内存，即使为空
- 导致巨大的内存消耗，可能超出系统限制

## 解决方案

### 1. 移除哈希表
- 删除了 `Coord2StateIDHashTable` 哈希表
- 删除了 `HashTableSize` 字段
- 删除了相关的哈希函数和查找方法

### 2. 使用 Octomap 直接映射
- 利用 octomap 的稀疏特性
- 只在需要时创建状态条目
- 使用 `std::unordered_map<int, EnvVoxel3DHashEntry_t*>` 存储状态映射

### 3. 懒加载策略
- 状态 ID 按需分配
- 坐标到状态 ID 的映射在首次访问时创建
- 显著减少内存使用

## 主要改进

### 内存使用
- **之前**: O(width × height × depth) 固定内存分配
- **现在**: O(实际使用的坐标数量) 动态内存分配

### 性能
- **之前**: 哈希表查找 + 碰撞处理
- **现在**: 直接 octomap 查询 + 线性搜索（仅对已使用的坐标）

### 代码简化
- 移除了复杂的哈希表管理代码
- 简化了内存管理
- 更清晰的接口设计

## 新的 API

### 核心方法
```cpp
// 获取或创建状态 ID
int GetOrCreateStateID(int X, int Y, int Z);

// 获取状态条目
EnvVoxel3DHashEntry_t* GetStateEntry(int stateID) const;

// 从坐标获取状态 ID
int GetStateIDFromCoord(int X, int Y, int Z) const;
```

### 使用示例
```cpp
EnvironmentVoxel3D env;
env.InitializeEnv(1000, 1000, 100, 0.1, -50.0, -50.0, 0.0);

// 设置起点和终点
int start_state = env.SetStart(100, 100, 50);
int goal_state = env.SetGoal(900, 900, 50);

// 获取坐标
int x, y, z;
env.GetCoordFromState(start_state, x, y, z);
```

## 测试

运行测试程序验证实现：
```bash
mkdir build && cd build
cmake -f ../CMakeLists_test.txt ..
make
./test_octomap_env
```

## 兼容性

- 保持了原有的公共 API 接口
- 现有的路径规划算法无需修改
- 向后兼容现有代码

## 优势

1. **内存效率**: 支持任意大小的环境，内存使用与实际占用空间成正比
2. **性能**: 避免了哈希表的内存分配和查找开销
3. **简单性**: 代码更简洁，易于维护
4. **扩展性**: 可以轻松扩展到更大的环境 