#include <iostream>
#include <memory>
#include <chrono>
#include <random>
#include "voxelization_base.hpp"
#include "spatial_entities.hpp"
#include "voxelization_algorithms.hpp"

using namespace voxelization;

void printUsage() {
    std::cout << "Usage: test_voxelization <algorithm_type> <num_entities> <grid_size>" << std::endl;
    std::cout << "Algorithm types: CPU_SEQUENTIAL, CPU_PARALLEL, GPU_CUDA, HYBRID" << std::endl;
    std::cout << "Example: test_voxelization CPU_PARALLEL 100 256" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        printUsage();
        return 1;
    }
    
    std::string algorithm_type = argv[1];
    int num_entities = std::stoi(argv[2]);
    int grid_size = std::stoi(argv[3]);
    
    std::cout << "=== Voxelization Test ===" << std::endl;
    std::cout << "Algorithm: " << algorithm_type << std::endl;
    std::cout << "Number of entities: " << num_entities << std::endl;
    std::cout << "Grid size: " << grid_size << "x" << grid_size << "x" << grid_size << std::endl;
    
    // Create voxelization algorithm
    VoxelizationFactory::AlgorithmType alg_type;
    if (algorithm_type == "CPU_SEQUENTIAL") {
        alg_type = VoxelizationFactory::AlgorithmType::CPU_SEQUENTIAL;
    } else if (algorithm_type == "CPU_PARALLEL") {
        alg_type = VoxelizationFactory::AlgorithmType::CPU_PARALLEL;
    } else if (algorithm_type == "GPU_CUDA") {
        alg_type = VoxelizationFactory::AlgorithmType::GPU_CUDA;
    } else if (algorithm_type == "HYBRID") {
        alg_type = VoxelizationFactory::AlgorithmType::HYBRID;
    } else {
        std::cerr << "Unknown algorithm type: " << algorithm_type << std::endl;
        return 1;
    }
    
    auto voxelizer = VoxelizationFactory::createAlgorithm(alg_type);
    if (!voxelizer) {
        std::cerr << "Failed to create voxelization algorithm" << std::endl;
        return 1;
    }
    
    // Initialize voxelizer
    double world_size = 1000.0; // 1km world
    double resolution = world_size / grid_size;
    
    voxelizer->initialize(grid_size, grid_size, grid_size, resolution, resolution);
    std::cout << "Voxelizer initialized with resolution: " << resolution << " m" << std::endl;
    
    // Create random entities
    std::vector<std::shared_ptr<SpatialEntity>> entities;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(0, world_size);
    std::uniform_real_distribution<> size_dist(10, 100);
    std::uniform_int_distribution<> type_dist(0, 2);
    
    std::cout << "Creating " << num_entities << " random entities..." << std::endl;
    
    for (int i = 0; i < num_entities; ++i) {
        double center_x = pos_dist(gen);
        double center_y = pos_dist(gen);
        double center_z = pos_dist(gen);
        
        int entity_type = type_dist(gen);
        std::shared_ptr<SpatialEntity> entity;
        
        switch (entity_type) {
            case 0: { // Box
                double size_x = size_dist(gen);
                double size_y = size_dist(gen);
                double size_z = size_dist(gen);
                entity = std::make_shared<BoxEntity>(center_x, center_y, center_z, size_x, size_y, size_z);
                break;
            }
            case 1: { // Cylinder
                double radius = size_dist(gen) / 2;
                double height = size_dist(gen);
                entity = std::make_shared<CylinderEntity>(center_x, center_y, center_z, radius, height);
                break;
            }
            case 2: { // Sphere
                double radius = size_dist(gen) / 2;
                entity = std::make_shared<SphereEntity>(center_x, center_y, center_z, radius);
                break;
            }
        }
        
        entities.push_back(entity);
    }
    
    std::cout << "Created entities: " << entities.size() << std::endl;
    
    // Perform voxelization
    double buffer_size = 5.0; // 5m buffer
    unsigned char cost_value = 255;
    
    std::cout << "Starting voxelization..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int marked_voxels = voxelizer->voxelizeEntities(entities, buffer_size, cost_value);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Voxelization completed in " << duration.count() << " ms" << std::endl;
    std::cout << "Marked voxels: " << marked_voxels << std::endl;
    
    // Calculate statistics
    auto grid_dims = voxelizer->getGridDimensions();
    int total_voxels = grid_dims[0] * grid_dims[1] * grid_dims[2];
    double occupancy_rate = static_cast<double>(marked_voxels) / total_voxels * 100.0;
    
    std::cout << "Grid dimensions: " << grid_dims[0] << "x" << grid_dims[1] << "x" << grid_dims[2] << std::endl;
    std::cout << "Total voxels: " << total_voxels << std::endl;
    std::cout << "Occupancy rate: " << occupancy_rate << "%" << std::endl;
    
    // Test individual entity voxelization
    std::cout << "\n=== Testing Individual Entity Voxelization ===" << std::endl;
    
    // Create a test box
    auto test_box = std::make_shared<BoxEntity>(world_size/2, world_size/2, world_size/2, 50, 50, 50);
    
    // Clear voxelizer
    voxelizer->clear();
    
    start_time = std::chrono::high_resolution_clock::now();
    int box_voxels = voxelizer->voxelizeEntity(test_box, buffer_size, cost_value);
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Single box voxelization: " << box_voxels << " voxels in " << duration.count() << " ms" << std::endl;
    
    // Test coordinate conversion
    std::cout << "\n=== Testing Coordinate Conversion ===" << std::endl;
    
    double test_world_x = 100.0, test_world_y = 200.0, test_world_z = 300.0;
    int test_voxel_x, test_voxel_y, test_voxel_z;
    
    if (voxelizer->worldToVoxel(test_world_x, test_world_y, test_world_z, test_voxel_x, test_voxel_y, test_voxel_z)) {
        std::cout << "World (" << test_world_x << ", " << test_world_y << ", " << test_world_z 
                  << ") -> Voxel (" << test_voxel_x << ", " << test_voxel_y << ", " << test_voxel_z << ")" << std::endl;
        
        double back_world_x, back_world_y, back_world_z;
        voxelizer->voxelToWorld(test_voxel_x, test_voxel_y, test_voxel_z, back_world_x, back_world_y, back_world_z);
        
        std::cout << "Voxel (" << test_voxel_x << ", " << test_voxel_y << ", " << test_voxel_z 
                  << ") -> World (" << back_world_x << ", " << back_world_y << ", " << back_world_z << ")" << std::endl;
    }
    
    // Test file I/O
    std::cout << "\n=== Testing File I/O ===" << std::endl;
    
    std::string filename = "test_voxel_grid.bin";
    if (voxelizer->saveToFile(filename)) {
        std::cout << "Successfully saved voxel grid to " << filename << std::endl;
        
        // Create new voxelizer and load
        auto new_voxelizer = VoxelizationFactory::createAlgorithm(alg_type);
        if (new_voxelizer->loadFromFile(filename)) {
            std::cout << "Successfully loaded voxel grid from " << filename << std::endl;
            
            // Compare voxel counts
            int loaded_voxels = 0;
            auto loaded_dims = new_voxelizer->getGridDimensions();
            for (int x = 0; x < loaded_dims[0]; ++x) {
                for (int y = 0; y < loaded_dims[1]; ++y) {
                    for (int z = 0; z < loaded_dims[2]; ++z) {
                        if (new_voxelizer->getVoxelCost(x, y, z) > 0) {
                            loaded_voxels++;
                        }
                    }
                }
            }
            std::cout << "Loaded voxel count: " << loaded_voxels << " (original: " << box_voxels << ")" << std::endl;
        } else {
            std::cout << "Failed to load voxel grid from " << filename << std::endl;
        }
    } else {
        std::cout << "Failed to save voxel grid to " << filename << std::endl;
    }
    
    std::cout << "\n=== Test Complete ===" << std::endl;
    
    return 0;
} 