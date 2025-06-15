#include <iostream>
#include <memory>
#include <vector>
#include <chrono>

// Local includes
#include "env/environment_voxel3d.hpp"
#include "single_global/multiscale_astar_planner.hpp"

int main()
{
    std::cout << "=== MultiScale A* Debug Test ===" << std::endl;
    
    // Create environment
    auto env = std::make_unique<EnvironmentVoxel3D>();
    
    // Initialize environment with small size for debugging
    bool success = env->InitializeEnv(
        50, 50, 25,  // 50x50x25 voxels
        1.0, 1.0,    // 1m resolution
        0.0, 0.0, 0.0  // origin at (0,0,0)
    );
    
    if (!success) {
        std::cout << "Failed to initialize environment!" << std::endl;
        return -1;
    }
    
    std::cout << "Environment initialized successfully" << std::endl;
    
    // Create a simple obstacle in the middle
    for (int x = 20; x < 30; ++x) {
        for (int y = 20; y < 30; ++y) {
            for (int z = 10; z < 15; ++z) {
                env->UpdateCellCost(x, y, z, 255);  // Occupied
            }
        }
    }
    
    std::cout << "Simple obstacle created in the middle" << std::endl;
    
    // Test start and goal positions (should be able to find a path around the obstacle)
    double start_x = 10.0, start_y = 10.0, start_z = 12.0;
    double goal_x = 40.0, goal_y = 40.0, goal_z = 12.0;
    
    std::cout << "Start: (" << start_x << ", " << start_y << ", " << start_z << ")" << std::endl;
    std::cout << "Goal:  (" << goal_x << ", " << goal_y << ", " << goal_z << ")" << std::endl;
    
    // Create MultiScale A* planner
    auto multiscale_planner = std::make_shared<MultiScaleAStarPlanner>(env.get());
    
    // Test path planning
    std::vector<geometry_msgs::msg::PoseStamped> multiscale_path;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    bool multiscale_success = multiscale_planner->planPath(
        start_x, start_y, start_z,
        goal_x, goal_y, goal_z,
        multiscale_path,
        SmootherType::NONE,
        10.0  // 10 second timeout
    );
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (multiscale_success) {
        std::cout << "MultiScale A* SUCCESS! Path length: " << multiscale_path.size() 
                  << " points, Time: " << duration.count() << "ms" << std::endl;
        
        // Print path details
        for (size_t i = 0; i < multiscale_path.size(); ++i) {
            const auto& pose = multiscale_path[i];
            std::cout << "  Point " << i << ": (" 
                      << pose.pose.position.x << ", " 
                      << pose.pose.position.y << ", " 
                      << pose.pose.position.z << ")" << std::endl;
        }
    } else {
        std::cout << "MultiScale A* FAILED! Time: " << duration.count() << "ms" << std::endl;
    }
    
    std::cout << "=== Debug Test Complete ===" << std::endl;
    return 0;
} 