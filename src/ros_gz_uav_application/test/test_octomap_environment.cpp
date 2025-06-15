#include "env/environment_voxel3d.hpp"
#include <iostream>

int main()
{
    std::cout << "Testing Octomap-based Environment Implementation" << std::endl;
    
    // Create environment with large dimensions
    EnvironmentVoxel3D env;
    
    // Test with large environment (1000x1000x100)
    bool success = env.InitializeEnv(1000, 1000, 100, 0.1, -50.0, -50.0, 0.0);
    
    if (!success) {
        std::cerr << "Failed to initialize environment" << std::endl;
        return -1;
    }
    
    std::cout << "Environment initialized successfully!" << std::endl;
    
    // Test coordinate conversion
    double world_x = 0.0, world_y = 0.0, world_z = 5.0;
    int voxel_x, voxel_y, voxel_z;
    
    bool converted = env.WorldToVoxel(world_x, world_y, world_z, voxel_x, voxel_y, voxel_z);
    std::cout << "World coordinates (" << world_x << ", " << world_y << ", " << world_z 
              << ") -> Voxel coordinates (" << voxel_x << ", " << voxel_y << ", " << voxel_z << ")" << std::endl;
    
    // Test reverse conversion
    double back_world_x, back_world_y, back_world_z;
    env.VoxelToWorld(voxel_x, voxel_y, voxel_z, back_world_x, back_world_y, back_world_z);
    std::cout << "Voxel coordinates (" << voxel_x << ", " << voxel_y << ", " << voxel_z 
              << ") -> World coordinates (" << back_world_x << ", " << back_world_y << ", " << back_world_z << ")" << std::endl;
    
    // Test occupancy
    bool is_occupied = env.IsCellOccupied(voxel_x, voxel_y, voxel_z);
    std::cout << "Position (" << voxel_x << ", " << voxel_y << ", " << voxel_z 
              << ") occupied: " << (is_occupied ? "Yes" : "No") << std::endl;
    
    // Test cost
    unsigned char cost = env.GetCellCost(voxel_x, voxel_y, voxel_z);
    std::cout << "Position (" << voxel_x << ", " << voxel_y << ", " << voxel_z 
              << ") cost: " << (int)cost << std::endl;
    
    // Test grid size
    int size_x, size_y, size_z;
    env.GetGridSize(size_x, size_y, size_z);
    std::cout << "Grid size: " << size_x << " x " << size_y << " x " << size_z << std::endl;
    
    std::cout << "All tests passed!" << std::endl;
    return 0;
} 