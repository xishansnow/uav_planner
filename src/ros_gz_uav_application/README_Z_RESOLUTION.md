# Z轴分辨率支持

## 概述

为了支持不同Z轴分辨率的3D体素环境，我们对 `EnvironmentVoxel3D` 类进行了扩展。

## 主要变更

### 1. 配置结构体更新

在 `EnvVoxel3DConfig_t` 结构体中添加了 `ResolutionZ` 字段：

```cpp
typedef struct ENV_VOXEL3D_CONFIG
{
    int Env_x;   // X dimension in voxels
    int Env_y;  // Y dimension in voxels
    int Env_z;   // Z dimension in voxels
    double ResolutionXY; // Size of each voxel in meters (X and Y)
    double ResolutionZ; // Size of each voxel in Z direction (can be different from X/Y)
    double OriginX;   // Origin X coordinate in world frame
    double OriginY;   // Origin Y coordinate in world frame
    double OriginZ;   // Origin Z coordinate in world frame
    // ... other fields
} EnvVoxel3DConfig_t;
```

### 2. 初始化方法扩展

新增了支持不同Z轴分辨率的初始化方法：

```cpp
bool InitializeEnv(int env_width, int env_height, int env_depth, 
                  double resolution_xy, double resolution_z,
                  double origin_x, double origin_y, double origin_z);
```

### 3. 坐标转换更新

`WorldToVoxel` 和 `VoxelToWorld` 方法现在使用不同的XY和Z分辨率：

```cpp
// WorldToVoxel
voxel_x = (world_x - origin_x) / resolution_xy;
voxel_y = (world_y - origin_y) / resolution_xy;
voxel_z = (world_z - origin_z) / resolution_z;

// VoxelToWorld
world_x = origin_x + (voxel_x + 0.5) * resolution_xy;
world_y = origin_y + (voxel_y + 0.5) * resolution_xy;
world_z = origin_z + (voxel_z + 0.5) * resolution_z;
```

## 使用示例

```cpp
// 创建环境，XY分辨率为0.1m，Z分辨率为0.05m
EnvironmentVoxel3D env;
env.InitializeEnv(1000, 1000, 100, 0.1, 0.05, -50.0, -50.0, 0.0);

// 坐标转换
int voxel_x, voxel_y, voxel_z;
env.WorldToVoxel(0.0, 0.0, 2.5, voxel_x, voxel_y, voxel_z);
// 结果: voxel_x=500, voxel_y=500, voxel_z=50

double world_x, world_y, world_z;
env.VoxelToWorld(500, 500, 50, world_x, world_y, world_z);
// 结果: world_x=0.05, world_y=0.05, world_z=2.525
```

## 向后兼容性

4. **向后兼容性**: 原有的 `InitializeEnv` 方法仍然可用，会自动设置 `ResolutionZ = ResolutionXY`。

## 应用场景

- **精确垂直导航**: 在需要精确控制高度的应用中，可以使用较小的 Z 轴分辨率
- **快速规划**: 在实时性要求高的场景中，可以使用较大的 Z 轴分辨率来减少计算量
- **地形适应**: 根据地形复杂度动态调整 Z 轴分辨率
- **传感器融合**: 结合不同传感器的精度要求设置合适的分辨率 

UAV_Sbpl_Planner planner(0.1, 0.0, 0.0, 0.0); // XY=Z=0.1m 