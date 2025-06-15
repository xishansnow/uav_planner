#include <iostream>
#include <memory>
#include <vector>
#include "voxelization_base.hpp"
#include "spatial_entities.hpp"
#include "voxelization_algorithms.hpp"

// Original obstacle structure (from existing code)
struct Obstacle {
    double center_x, center_y, center_z;
    double size_x, size_y, size_z;
    enum Type { BOX, CYLINDER, SPHERE } type;
    double buffer_size;
};

// Migration example: Convert existing obstacles to spatial entities
std::vector<std::shared_ptr<voxelization::SpatialEntity>> 
convertObstaclesToEntities(const std::vector<Obstacle>& obstacles) {
    std::vector<std::shared_ptr<voxelization::SpatialEntity>> entities;
    
    for (const auto& obs : obstacles) {
        std::shared_ptr<voxelization::SpatialEntity> entity;
        
        switch (obs.type) {
            case Obstacle::BOX:
                entity = std::make_shared<voxelization::BoxEntity>(
                    obs.center_x, obs.center_y, obs.center_z,
                    obs.size_x, obs.size_y, obs.size_z
                );
                break;
                
            case Obstacle::CYLINDER:
                entity = std::make_shared<voxelization::CylinderEntity>(
                    obs.center_x, obs.center_y, obs.center_z,
                    obs.size_x / 2, obs.size_z  // radius = size_x/2, height = size_z
                );
                break;
                
            case Obstacle::SPHERE:
                entity = std::make_shared<voxelization::SphereEntity>(
                    obs.center_x, obs.center_y, obs.center_z,
                    obs.size_x / 2  // radius = size_x/2
                );
                break;
        }
        
        if (entity) {
            entities.push_back(entity);
        }
    }
    
    return entities;
}

// Example: Replace existing addObstacleToVoxelEnvironment function
class MigratedPlanningNode {
private:
    std::unique_ptr<voxelization::VoxelizationBase> voxelizer_;
    
public:
    MigratedPlanningNode() {
        // Initialize voxelizer with same parameters as original code
        voxelizer_ = voxelization::VoxelizationFactory::createAlgorithm(
            voxelization::VoxelizationFactory::AlgorithmType::CPU_PARALLEL
        );
        
        // Use same grid parameters as original
        int grid_x = 2048, grid_y = 2048, grid_z = 1024;
        double resolution_xy = 100000.0 / grid_x;  // 100km / grid_size
        double resolution_z = 5000.0 / grid_z;     // 5km / grid_size
        
        voxelizer_->initialize(grid_x, grid_y, grid_z, resolution_xy, resolution_z);
    }
    
    // Replace the original addObstacleToVoxelEnvironment function
    void addObstaclesToVoxelEnvironment(const std::vector<Obstacle>& obstacles) {
        // Convert obstacles to spatial entities
        auto entities = convertObstaclesToEntities(obstacles);
        
        // Use the buffer size from the first obstacle (assuming all have same buffer)
        double buffer_size = obstacles.empty() ? 1000.0 : obstacles[0].buffer_size;
        
        // Voxelize all entities at once (much more efficient than original approach)
        int marked_voxels = voxelizer_->voxelizeEntities(entities, buffer_size, 255);
        
        std::cout << "Marked " << marked_voxels << " voxels for " << obstacles.size() << " obstacles" << std::endl;
    }
    
    // Replace the original batchAddObstaclesToVoxelEnvironment function
    void batchAddObstaclesToVoxelEnvironment(const std::vector<Obstacle>& obstacles, double buffer_size) {
        // This is now the same as addObstaclesToVoxelEnvironment since the new module
        // automatically handles batching and optimization
        addObstaclesToVoxelEnvironment(obstacles);
    }
    
    // Get voxel grid data (compatible with existing environment)
    unsigned char* getVoxelGrid() const {
        return voxelizer_->getVoxelGrid();
    }
    
    // Coordinate conversion (compatible with existing code)
    bool worldToVoxel(double world_x, double world_y, double world_z,
                     int& voxel_x, int& voxel_y, int& voxel_z) const {
        return voxelizer_->worldToVoxel(world_x, world_y, world_z, voxel_x, voxel_y, voxel_z);
    }
    
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                     double& world_x, double& world_y, double& world_z) const {
        voxelizer_->voxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
    }
    
    // Check if voxel is occupied (compatible with existing code)
    bool isVoxelOccupied(int voxel_x, int voxel_y, int voxel_z) const {
        return voxelizer_->getVoxelCost(voxel_x, voxel_y, voxel_z) > 0;
    }
    
    // Update voxel cost (compatible with existing code)
    void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) {
        voxelizer_->updateVoxelCost(voxel_x, voxel_y, voxel_z, cost_value);
    }
};

// Example: Performance comparison
void performanceComparison() {
    std::cout << "=== Performance Comparison ===" << std::endl;
    
    // Create test obstacles
    std::vector<Obstacle> test_obstacles;
    for (int i = 0; i < 50; ++i) {
        Obstacle obs;
        obs.center_x = i * 1000.0;
        obs.center_y = i * 1000.0;
        obs.center_z = 2500.0;
        obs.size_x = 5000.0;
        obs.size_y = 5000.0;
        obs.size_z = 5000.0;
        obs.type = static_cast<Obstacle::Type>(i % 3);
        obs.buffer_size = 1000.0;
        test_obstacles.push_back(obs);
    }
    
    // Test different algorithms
    std::vector<voxelization::VoxelizationFactory::AlgorithmType> algorithms = {
        voxelization::VoxelizationFactory::AlgorithmType::CPU_SEQUENTIAL,
        voxelization::VoxelizationFactory::AlgorithmType::CPU_PARALLEL,
        voxelization::VoxelizationFactory::AlgorithmType::GPU_CUDA
    };
    
    std::vector<std::string> algorithm_names = {"CPU Sequential", "CPU Parallel", "GPU CUDA"};
    
    for (size_t i = 0; i < algorithms.size(); ++i) {
        auto voxelizer = voxelization::VoxelizationFactory::createAlgorithm(algorithms[i]);
        if (!voxelizer) {
            std::cout << algorithm_names[i] << ": Not available" << std::endl;
            continue;
        }
        
        voxelizer->initialize(1024, 1024, 512, 50.0, 10.0);
        
        auto entities = convertObstaclesToEntities(test_obstacles);
        
        auto start = std::chrono::high_resolution_clock::now();
        int marked_voxels = voxelizer->voxelizeEntities(entities, 1000.0, 255);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << algorithm_names[i] << ": " << marked_voxels << " voxels in " 
                  << duration.count() << " ms" << std::endl;
    }
}

// Example: Advanced usage with different entity types
void advancedUsage() {
    std::cout << "\n=== Advanced Usage ===" << std::endl;
    
    auto voxelizer = voxelization::VoxelizationFactory::createAlgorithm(
        voxelization::VoxelizationFactory::AlgorithmType::CPU_PARALLEL
    );
    voxelizer->initialize(512, 512, 256, 10.0, 5.0);
    
    std::vector<std::shared_ptr<voxelization::SpatialEntity>> entities;
    
    // Add different types of entities
    entities.push_back(std::make_shared<voxelization::BoxEntity>(100, 100, 50, 80, 60, 40));
    entities.push_back(std::make_shared<voxelization::CylinderEntity>(300, 300, 50, 40, 100));
    entities.push_back(std::make_shared<voxelization::SphereEntity>(500, 500, 50, 30));
    entities.push_back(std::make_shared<voxelization::EllipsoidEntity>(700, 700, 50, 50, 30, 40));
    entities.push_back(std::make_shared<voxelization::ConeEntity>(900, 900, 50, 25, 80));
    
    // Create a composite entity
    std::vector<std::shared_ptr<voxelization::SpatialEntity>> composite_parts;
    composite_parts.push_back(std::make_shared<voxelization::BoxEntity>(1100, 1100, 50, 40, 40, 40));
    composite_parts.push_back(std::make_shared<voxelization::SphereEntity>(1100, 1100, 70, 20));
    auto composite = std::make_shared<voxelization::CompositeEntity>(composite_parts);
    entities.push_back(composite);
    
    // Voxelize all entities
    int marked_voxels = voxelizer->voxelizeEntities(entities, 5.0, 255);
    std::cout << "Advanced example: " << marked_voxels << " voxels marked" << std::endl;
    
    // Save result
    voxelizer->saveToFile("advanced_example.bin");
    std::cout << "Saved voxel grid to advanced_example.bin" << std::endl;
}

int main() {
    std::cout << "=== Obstacle Migration Example ===" << std::endl;
    
    // Create test obstacles
    std::vector<Obstacle> obstacles;
    for (int i = 0; i < 10; ++i) {
        Obstacle obs;
        obs.center_x = i * 5000.0;
        obs.center_y = i * 5000.0;
        obs.center_z = 2500.0;
        obs.size_x = 3000.0;
        obs.size_y = 3000.0;
        obs.size_z = 5000.0;
        obs.type = static_cast<Obstacle::Type>(i % 3);
        obs.buffer_size = 1000.0;
        obstacles.push_back(obs);
    }
    
    // Test migrated node
    MigratedPlanningNode node;
    node.addObstaclesToVoxelEnvironment(obstacles);
    
    // Performance comparison
    performanceComparison();
    
    // Advanced usage
    advancedUsage();
    
    std::cout << "\n=== Migration Complete ===" << std::endl;
    std::cout << "The new voxelization module provides:" << std::endl;
    std::cout << "- Better performance through optimized algorithms" << std::endl;
    std::cout << "- Support for more entity types" << std::endl;
    std::cout << "- Easier extension and maintenance" << std::endl;
    std::cout << "- Better memory management" << std::endl;
    std::cout << "- File I/O capabilities" << std::endl;
    
    return 0;
} 