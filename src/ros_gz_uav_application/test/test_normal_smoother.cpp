#include <iostream>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "smoother/smoother_normal.hpp"

int main()
{
    std::cout << "Testing Normal Smoother..." << std::endl;
    
    // Create a normal smoother
    SmootherNormal smoother;
    
    // Create a test path with unnecessary waypoints
    std::vector<geometry_msgs::msg::PoseStamped> raw_path;
    
    // Add waypoints
    for (int i = 0; i < 10; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = i * 1.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        raw_path.push_back(pose);
    }
    
    std::cout << "Original path has " << raw_path.size() << " waypoints" << std::endl;
    
    // Smooth the path
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
    smoother.smoothPath(raw_path, smoothed_path);
    
    std::cout << "Smoothed path has " << smoothed_path.size() << " waypoints" << std::endl;
    
    // Print the smoothed path
    std::cout << "Smoothed path waypoints:" << std::endl;
    for (size_t i = 0; i < smoothed_path.size(); ++i) {
        std::cout << "  " << i << ": (" 
                  << smoothed_path[i].pose.position.x << ", "
                  << smoothed_path[i].pose.position.y << ", "
                  << smoothed_path[i].pose.position.z << ")" << std::endl;
    }
    
    std::cout << "Normal smoother test completed successfully!" << std::endl;
    return 0;
} 