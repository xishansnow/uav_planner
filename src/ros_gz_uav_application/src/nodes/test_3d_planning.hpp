#ifndef TEST_3D_PLANNING_HPP
#define TEST_3D_PLANNING_HPP

#include <iostream>
#include <vector>
#include <random>
#include <memory>
#include <chrono>
#include <fstream>
#include <sstream>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Local includes
#include "env/environment_voxel3d.hpp"
#include "single_global/astar_planner.hpp"
#include "single_global/thetastar_planner.hpp"
#include "single_global/arastar_planner.hpp"
#include "single_global/multiscale_astar_planner.hpp"
#include "single_global/planner_factory.hpp"

// Data structures for planning test
enum class ObstacleType { BOX, CYLINDER, SPHERE };

struct Obstacle {
    double center_x, center_y, center_z;
    double size_x, size_y, size_z;
    ObstacleType type;
};

struct StartGoalPair {
    double start_x, start_y, start_z;
    double goal_x, goal_y, goal_z;
};

struct Path {
    int planner_type;  // 0: A*, 1: Theta*, 2: ARA*, 3: JPS, 4: MultiScale A*
    int pair_index;
    std::vector<geometry_msgs::msg::PoseStamped> original_path;  // 平滑前的原始路径
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;  // 平滑后的路径
};

class PlanningTestNode : public rclcpp::Node
{
public:
    PlanningTestNode();

    

private:
    // Environment initialization
    void initializeEnvironment();
    void createObstacles(int num_obstacles);
    // 新增：将障碍物添加到体素环境
    void addObstacleToVoxelEnvironment(const Obstacle& obs, double buffer_size = 0.0);
    // 新增：批量添加障碍物到体素环境（优化版本）
    void batchAddObstaclesToVoxelEnvironment(const std::vector<Obstacle>& obstacles, double buffer_size);
    // 新增：将路径添加到体素环境
    void addPathsToVoxelEnvironment();
    // 新增：验证顶层体素标记
    void verifyTopLayerVoxelMarking();
    bool isPointInObstacle(int voxel_x, int voxel_y, int voxel_z, const Obstacle& obs);
    bool isPointInObstacleWithBuffer(int voxel_x, int voxel_y, int voxel_z, const Obstacle& obs, double buffer_size);
    void generateStartGoalPairs();
    bool isPointOccupied(double x, double y, double z);
    
    // Planning tests
    void runPlanningTests();
    
    // Visualization methods
    void publishVisualization();    
    void publishVoxelGrid(visualization_msgs::msg::MarkerArray& marker_array);
    void addLine(visualization_msgs::msg::Marker& marker, double x1, double y1, double z1, 
                 double x2, double y2, double z2);
    void publishEnvironmentBounds(visualization_msgs::msg::MarkerArray& marker_array);
    void publishObstacles(visualization_msgs::msg::MarkerArray& marker_array);
    void publishStartGoalPoints(visualization_msgs::msg::MarkerArray& marker_array);
    void publishPaths(visualization_msgs::msg::MarkerArray& marker_array);
    void publishOriginalPaths(visualization_msgs::msg::MarkerArray& marker_array);
    void publishSmoothedPaths(visualization_msgs::msg::MarkerArray& marker_array);
    
    // Transform and octomap publishing
    void publishMapTransform();
    void publishOctomap();
    
    // Member variables
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Separate publishers for different visualization types
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_bounds_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr original_paths_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr smoothed_paths_pub_;
    
    // Publishers for octomap
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_octomap_pub_;
    
    // Create static transform broadcaster
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    
    std::unique_ptr<EnvironmentVoxel3D> env_;
    std::vector<Obstacle> obstacles_;
    std::vector<StartGoalPair> start_goal_pairs_;
    std::vector<Path> paths_;
    
    // Environment parameters
    double world_x_, world_y_, world_z_;
    int env_x_, env_y_, env_z_;
    double resolution_xy_, resolution_z_;
    
    // Random number generation
    std::mt19937 gen_;
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;

    // 优化：针对不同障碍物类型的专用标记函数
    int markBoxObstacle(const Obstacle& obs, double buffer_size, 
                       int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);
    int markCylinderObstacle(const Obstacle& obs, double buffer_size,
                            int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);
    int markSphereObstacle(const Obstacle& obs, double buffer_size,
                          int min_x, int max_x, int min_y, int max_y, int min_z, int max_z);
};

#endif // TEST_3D_PLANNING_HPP 