#include <iostream>
#include <memory>
#include <vector>
#include <chrono>

// Local includes
#include "env/environment_voxel3d.hpp"
#include "single_global/multiscale_astar_planner.hpp"
#include "single_global/astar_planner.hpp"
#include "single_global/planner_factory.hpp"

int main()
{
    std::cout << "=== MultiScale A* Planner Test ===" << std::endl;
    
    // Create environment
    auto env = std::make_unique<EnvironmentVoxel3D>();
    
    // Initialize environment with smaller size for testing
    bool success = env->InitializeEnv(
        100, 100, 50,  // 100x100x50 voxels
        1.0, 1.0,      // 1m resolution
        0.0, 0.0, 0.0  // origin at (0,0,0)
    );
    
    if (!success) {
        std::cout << "Failed to initialize environment!" << std::endl;
        return -1;
    }
    
    std::cout << "Environment initialized successfully" << std::endl;
    
    // Create some obstacles
    for (int x = 20; x < 80; ++x) {
        for (int y = 20; y < 80; ++y) {
            for (int z = 10; z < 40; ++z) {
                if (x >= 30 && x < 70 && y >= 30 && y < 70) {
                    env->UpdateCellCost(x, y, z, 255);  // Occupied
                }
            }
        }
    }
    
    std::cout << "Obstacles created" << std::endl;
    
    // Test start and goal positions
    double start_x = 10.0, start_y = 10.0, start_z = 25.0;
    double goal_x = 90.0, goal_y = 90.0, goal_z = 25.0;
    
    std::cout << "Start: (" << start_x << ", " << start_y << ", " << start_z << ")" << std::endl;
    std::cout << "Goal:  (" << goal_x << ", " << goal_y << ", " << goal_z << ")" << std::endl;
    
    // Test MultiScale A* using factory
    try {
        auto multiscale_planner = PlannerFactory::createPlanner("multiscale_astar", env.get());
        std::cout << "Created MultiScale A* planner: " << multiscale_planner->getAlgorithmName() << std::endl;
        
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
        } else {
            std::cout << "MultiScale A* FAILED! Time: " << duration.count() << "ms" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error creating MultiScale A* planner: " << e.what() << std::endl;
    }
    
    // Test regular A* for comparison
    try {
        auto astar_planner = PlannerFactory::createPlanner("astar", env.get());
        std::cout << "Created A* planner: " << astar_planner->getAlgorithmName() << std::endl;
        
        // Test path planning
        std::vector<geometry_msgs::msg::PoseStamped> astar_path;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        bool astar_success = astar_planner->planPath(
            start_x, start_y, start_z,
            goal_x, goal_y, goal_z,
            astar_path,
            SmootherType::NONE,
            10.0  // 10 second timeout
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (astar_success) {
            std::cout << "A* SUCCESS! Path length: " << astar_path.size() 
                      << " points, Time: " << duration.count() << "ms" << std::endl;
        } else {
            std::cout << "A* FAILED! Time: " << duration.count() << "ms" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error creating A* planner: " << e.what() << std::endl;
    }
    
    std::cout << "=== Test Complete ===" << std::endl;
    return 0;
} 