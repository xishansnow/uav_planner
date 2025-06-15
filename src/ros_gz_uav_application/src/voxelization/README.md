# Voxelization Module

A modular and extensible 3D voxelization library for spatial entities, supporting multiple algorithms and entity types.

## Features

- **Multiple Entity Types**: Box, Cylinder, Sphere, Ellipsoid, Cone, Mesh, and Composite entities
- **Multiple Algorithms**: CPU Sequential, CPU Parallel (OpenMP), GPU (CUDA), and Hybrid
- **Extensible Design**: Easy to add new entity types and algorithms
- **File I/O**: Save and load voxel grids
- **Performance Optimized**: Type-specific optimizations and parallel processing

## Architecture

### Core Components

1. **SpatialEntity**: Base class for all spatial entities
2. **VoxelizationBase**: Base class for voxelization algorithms
3. **VoxelizationFactory**: Factory for creating algorithms
4. **SpatialEntityFactory**: Factory for creating entities

### Entity Types

- **BoxEntity**: Axis-aligned bounding boxes
- **CylinderEntity**: Cylinders with specified radius and height
- **SphereEntity**: Spheres with specified radius
- **EllipsoidEntity**: Ellipsoids with different radii per axis
- **ConeEntity**: Cones with specified radius and height
- **MeshEntity**: Complex shapes defined by vertices and faces
- **CompositeEntity**: Union of multiple entities

### Algorithms

- **CPUSequentialVoxelization**: Basic CPU implementation
- **CPUParallelVoxelization**: OpenMP-parallelized CPU implementation
- **GPUCudaVoxelization**: GPU implementation using CUDA
- **HybridVoxelization**: Adaptive CPU/GPU implementation

## Usage

### Basic Usage

```cpp
#include "voxelization_base.hpp"
#include "spatial_entities.hpp"
#include "voxelization_algorithms.hpp"

using namespace voxelization;

// Create voxelization algorithm
auto voxelizer = VoxelizationFactory::createAlgorithm(
    VoxelizationFactory::AlgorithmType::CPU_PARALLEL
);

// Initialize with grid parameters
voxelizer->initialize(256, 256, 256, 1.0, 1.0); // 256x256x256 grid, 1m resolution

// Create spatial entities
auto box = std::make_shared<BoxEntity>(100, 100, 100, 50, 50, 50);
auto sphere = std::make_shared<SphereEntity>(200, 200, 200, 30);

// Voxelize entities
std::vector<std::shared_ptr<SpatialEntity>> entities = {box, sphere};
int marked_voxels = voxelizer->voxelizeEntities(entities, 5.0, 255);

// Access voxel grid
unsigned char* grid_data = voxelizer->getVoxelGrid();
```

### Creating Custom Entities

```cpp
class CustomEntity : public SpatialEntity {
public:
    CustomEntity(double center_x, double center_y, double center_z, double size)
        : center_x_(center_x), center_y_(center_y), center_z_(center_z), size_(size) {}
    
    std::vector<double> getBoundingBox() const override {
        return {
            center_x_ - size_, center_y_ - size_, center_z_ - size_,
            center_x_ + size_, center_y_ + size_, center_z_ + size_
        };
    }
    
    bool isPointInside(double x, double y, double z) const override {
        // Implement your custom geometry check
        double dx = x - center_x_;
        double dy = y - center_y_;
        double dz = z - center_z_;
        return (dx*dx + dy*dy + dz*dz) <= size_*size_;
    }
    
    std::string getType() const override { return "custom"; }
    
    std::map<std::string, double> getProperties() const override {
        return {
            {"center_x", center_x_},
            {"center_y", center_y_},
            {"center_z", center_z_},
            {"size", size_}
        };
    }
    
private:
    double center_x_, center_y_, center_z_, size_;
};
```

### File I/O

```cpp
// Save voxel grid
voxelizer->saveToFile("voxel_grid.bin");

// Load voxel grid
auto new_voxelizer = VoxelizationFactory::createAlgorithm(
    VoxelizationFactory::AlgorithmType::CPU_SEQUENTIAL
);
new_voxelizer->loadFromFile("voxel_grid.bin");
```

### Coordinate Conversion

```cpp
// World to voxel coordinates
int voxel_x, voxel_y, voxel_z;
if (voxelizer->worldToVoxel(100.5, 200.3, 300.7, voxel_x, voxel_y, voxel_z)) {
    // Coordinates are valid
}

// Voxel to world coordinates
double world_x, world_y, world_z;
voxelizer->voxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
```

## Building

### Prerequisites

- CMake 3.8+
- C++17 compiler
- Eigen3
- OpenMP (optional, for parallel algorithms)
- CUDA (optional, for GPU algorithms)

### Build Instructions

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake ..

# Build
make -j$(nproc)

# Run tests
./test_voxelization CPU_PARALLEL 100 256
```

### CUDA Support

To enable CUDA support:

```bash
cmake -DENABLE_CUDA=ON ..
```

## Performance

### Algorithm Comparison

| Algorithm | Pros | Cons | Best For |
|-----------|------|------|----------|
| CPU Sequential | Simple, reliable | Slow | Small grids, debugging |
| CPU Parallel | Good performance, portable | Memory bandwidth limited | Medium grids, multi-core systems |
| GPU CUDA | Very fast for large grids | Complex setup, memory transfer | Large grids, many entities |
| Hybrid | Adaptive, best of both worlds | More complex | Variable workloads |

### Performance Tips

1. **Choose the right algorithm**: Use GPU for large grids (>512³), CPU parallel for medium grids
2. **Batch processing**: Process multiple entities together for better performance
3. **Memory layout**: Voxel grid uses Z-major layout for better cache performance
4. **Resolution**: Higher resolution = more accurate but slower processing

## Extending the Module

### Adding New Entity Types

1. Inherit from `SpatialEntity`
2. Implement required virtual methods
3. Add to `SpatialEntityFactory` if needed

### Adding New Algorithms

1. Inherit from `VoxelizationBase`
2. Implement required virtual methods
3. Add to `VoxelizationFactory`

### Adding File Format Support

1. Implement format-specific reader/writer
2. Add to entity factory or create specialized loader

## Integration with ROS

This module can be easily integrated with ROS packages:

```cpp
// In your ROS node
#include "voxelization_base.hpp"

class MyPlanningNode : public rclcpp::Node {
private:
    std::unique_ptr<voxelization::VoxelizationBase> voxelizer_;
    
public:
    MyPlanningNode() : Node("my_planning_node") {
        // Initialize voxelizer
        voxelizer_ = voxelization::VoxelizationFactory::createAlgorithm(
            voxelization::VoxelizationFactory::AlgorithmType::CPU_PARALLEL
        );
        voxelizer_->initialize(1024, 1024, 512, 0.5, 0.5);
    }
    
    void processObstacles(const std::vector<Obstacle>& obstacles) {
        std::vector<std::shared_ptr<voxelization::SpatialEntity>> entities;
        
        for (const auto& obs : obstacles) {
            auto entity = std::make_shared<voxelization::BoxEntity>(
                obs.center_x, obs.center_y, obs.center_z,
                obs.size_x, obs.size_y, obs.size_z
            );
            entities.push_back(entity);
        }
        
        voxelizer_->voxelizeEntities(entities, obs.buffer_size, 255);
    }
};
```

## Examples

See `test/test_voxelization.cpp` for a complete example demonstrating:

- Creating different entity types
- Using different algorithms
- Performance measurement
- File I/O operations
- Coordinate conversion

## License

This module is part of the UAV Planning package and follows the same license terms. 