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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Local includes
#include "env/environment_voxel3d.hpp"
#include "single_global/global_planner_base.hpp"
#include "single_global/astar_planner.hpp"
#include "single_global/thetastar_planner.hpp"
#include "single_global/arastar_planner.hpp"
#include "single_global/jps_planner.hpp"
#include "single_global/multiscale_astar_planner.hpp"
#include "single_global/planner_factory.hpp"
#include "smoother/smoother_factory.hpp"
#include "test_3d_planning.hpp"


PlanningTestNode::PlanningTestNode() : Node("planning_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting planning test node initialization...");
        
        // Publishers for visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planning_visualization", 10);
        
        // Separate publishers for different visualization types
        environment_bounds_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/environment_bounds", 10);
        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacles", 10);
        voxel_grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/voxel_grid", 10);
        start_goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/start_goal_points", 10);
        paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/paths", 10);
        original_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/original_paths", 10);
        smoothed_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/smoothed_paths", 10);
        
        // Publishers for octomap
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_full", 10);
        binary_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", 10);
        
        // Create static transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "Publishers created");
        
        // Initialize random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());
        
        RCLCPP_INFO(this->get_logger(), "Random number generator initialized");
        
        // Environment parameters - use smaller size for testing
        world_x_ = 100000.0;   // 300 km instead of 10 km
        world_y_ = 100000.0;  // 300 km instead of 10 km
        world_z_ = 5000.0;    // 5 km instead of 2 km
        
        env_x_ = 2048;    // Smaller grid for testing
        env_y_ = 2048;   // Smaller grid for testing
        env_z_ = 1024;     // Smaller grid for testing
        
        // Calculate resolutions
        resolution_xy_ = world_x_ / env_x_;
        resolution_z_ = world_z_ / env_z_;
        
        std::cout << "Environment setup:" << std::endl;
        std::cout << "  World size: " << world_x_ << " x " << world_y_ << " x " << world_z_ << " m" << std::endl;
        std::cout << "  Voxel grid: " << env_x_ << " x " << env_y_ << " x " << env_z_ << std::endl;
        std::cout << "  XY resolution: " << resolution_xy_ << " m" << std::endl;
        std::cout << "  Z resolution: " << resolution_z_ << " m" << std::endl;
        
        RCLCPP_INFO(this->get_logger(), "Environment parameters set");
        
        // Initialize environment
        RCLCPP_INFO(this->get_logger(), "Initializing environment...");
        initializeEnvironment();
        
        // 创建障碍物，并将其添加到体素环境中
        RCLCPP_INFO(this->get_logger(), "Creating obstacles...");
        createObstacles(20);  
        
        // 验证顶层体素标记
        verifyTopLayerVoxelMarking();
        
        // 生成起点/终点对
        RCLCPP_INFO(this->get_logger(), "Generating start-goal pairs...");
        generateStartGoalPairs();

        // 运行规划测试
        RCLCPP_INFO(this->get_logger(), "Running planning tests...");
        runPlanningTests();

        // 将路径填充至体素环境，以体素方式显示路径
        // RCLCPP_INFO(this->get_logger(), "Adding paths to environment...");
        addPathsToVoxelEnvironment();  
        
        // 发布地图坐标系变换
        publishMapTransform();
       
        // 发布主题
        // 绘制体素环境（占有体素和线框形式），并以 marker 的形式发布障碍物、起点、终点、路径        
        visualization_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Publishing marker visualization...");
                this->publishVisualization();
                RCLCPP_INFO(this->get_logger(), "Publishing octomap...");
                this->publishOctomap();
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Planning test node initialization complete!");
    }

    void PlanningTestNode::addPathsToVoxelEnvironment()
    {
        if (!env_) return;
        for (const auto& path : paths_) {
            // 使用平滑前的原始路径填充体素环境
            for (const auto& pose_stamped : path.original_path) {
                double x = pose_stamped.pose.position.x;
                double y = pose_stamped.pose.position.y;
                double z = pose_stamped.pose.position.z;
                int vx, vy, vz;
                if (env_->WorldToVoxel(x, y, z, vx, vy, vz)) {
                    // 这里你可以选择 UpdateCellCost 的值，比如128表示路径
                    env_->UpdateCellCost(vx, vy, vz, 128);
                }
            }
        }
    }

    void PlanningTestNode::initializeEnvironment()
    {
        env_ = std::make_unique<EnvironmentVoxel3D>();
        
        bool success = env_->InitializeEnv(
            env_x_, env_y_, env_z_,
            resolution_xy_, resolution_z_,
            0.0, 0.0, 0.0  // origin at (0,0,0)
        );
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize environment!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Environment initialized successfully");
    }
    
    void PlanningTestNode::createObstacles(int num_obstacles)
    {
        RCLCPP_INFO(this->get_logger(), "Starting obstacle creation...");
        obstacles_.clear();
        
        // 设置缓冲半径（可以根据需要调整）
        double buffer_size = 1000.0;  // 1km 缓冲半径
        
        // Create fewer obstacles for testing
        for (int i = 0; i < num_obstacles; ++i) {  // Only 3 obstacles instead of 10
            // RCLCPP_INFO(this->get_logger(), "Creating obstacle %d...", i+1);
            
            Obstacle obs;
            
            // Random position (X和Y随机，Z设为环境中心)
            obs.center_x = uniform_dist_(gen_) * world_x_;
            obs.center_y = uniform_dist_(gen_) * world_y_;
            obs.center_z = world_z_ / 2.0;  // 固定在环境Z轴中心
            
            // Random size - smaller for testing
            obs.size_x = 5000.0 + uniform_dist_(gen_) * 2000.0;  // 10000-15000m
            obs.size_y = 5000.0 + uniform_dist_(gen_) * 2000.0;  // 10000-15000m
            // 改造：让障碍物在Z轴上占据完整空间
            obs.size_z = world_z_;  // 占据整个Z轴空间
            
            // Random shape type
            obs.type = static_cast<ObstacleType>(i % 3);  // 0: box, 1: cylinder, 2: sphere
            
            obstacles_.push_back(obs);
            
            RCLCPP_INFO(this->get_logger(), "Obstacle %d: type=%d, pos=(%.1f,%.1f,%.1f), size=(%.1f,%.1f,%.1f), buffer=%.1f", 
                       i+1, static_cast<int>(obs.type), obs.center_x, obs.center_y, obs.center_z,
                       obs.size_x, obs.size_y, obs.size_z, buffer_size);
        }
        
        // 优化：批量处理所有障碍物
        RCLCPP_INFO(this->get_logger(), "Batch processing %zu obstacles...", obstacles_.size());
        batchAddObstaclesToVoxelEnvironment(obstacles_, buffer_size);
        
        // RCLCPP_INFO(this->get_logger(), "Created %zu obstacles", obstacles_.size());
    }
    
    void PlanningTestNode::batchAddObstaclesToVoxelEnvironment(const std::vector<Obstacle>& obstacles, double buffer_size)
    {
        if (!env_) return;
        
        // 预计算所有障碍物的边界
        std::vector<std::tuple<int, int, int, int, int, int>> obstacle_bounds;
        obstacle_bounds.reserve(obstacles.size());
        
        for (const auto& obs : obstacles) {
            int min_x, min_y, min_z, max_x, max_y, max_z;
            
            double expanded_size_x = obs.size_x + 2 * buffer_size;
            double expanded_size_y = obs.size_y + 2 * buffer_size;
            double expanded_size_z = obs.size_z + 2 * buffer_size;
            
            env_->WorldToVoxel(obs.center_x - expanded_size_x/2, obs.center_y - expanded_size_y/2, obs.center_z - expanded_size_z/2,
                              min_x, min_y, min_z);
            env_->WorldToVoxel(obs.center_x + expanded_size_x/2, obs.center_y + expanded_size_y/2, obs.center_z + expanded_size_z/2,
                              max_x, max_y, max_z);
            
            // 边界检查
            if(min_x<0) min_x = 0;
            if(min_y<0) min_y = 0;
            if(min_z<0) min_z = 0;
            if(max_x>=env_x_) max_x = env_x_ - 1;
            if(max_y>=env_y_) max_y = env_y_ - 1;
            if(max_z>=env_z_) max_z = env_z_ - 1;
            
            obstacle_bounds.emplace_back(min_x, max_x, min_y, max_y, min_z, max_z);
        }
        
        // 计算全局边界
        int global_min_x = env_x_, global_max_x = 0;
        int global_min_y = env_y_, global_max_y = 0;
        int global_min_z = env_z_, global_max_z = 0;
        
        for (const auto& bounds : obstacle_bounds) {
            int min_x, max_x, min_y, max_y, min_z, max_z;
            std::tie(min_x, max_x, min_y, max_y, min_z, max_z) = bounds;
            
            global_min_x = std::min(global_min_x, min_x);
            global_max_x = std::max(global_max_x, max_x);
            global_min_y = std::min(global_min_y, min_y);
            global_max_y = std::max(global_max_y, max_y);
            global_min_z = std::min(global_min_z, min_z);
            global_max_z = std::max(global_max_z, max_z);
        }
        
        RCLCPP_INFO(this->get_logger(), "Global bounds: x[%d,%d], y[%d,%d], z[%d,%d]", 
                   global_min_x, global_max_x, global_min_y, global_max_y, global_min_z, global_max_z);
        
        // 批量标记体素
        int total_marked = 0;
        
        #pragma omp parallel for collapse(3) reduction(+:total_marked)
        for (int x = global_min_x; x <= global_max_x; ++x) {
            for (int y = global_min_y; y <= global_max_y; ++y) {
                for (int z = global_min_z; z <= global_max_z; ++z) {
                    // 检查当前体素是否被任何障碍物占用
                    for (size_t i = 0; i < obstacles.size(); ++i) {
                        const auto& obs = obstacles[i];
                        const auto& bounds = obstacle_bounds[i];
                        
                        int min_x, max_x, min_y, max_y, min_z, max_z;
                        std::tie(min_x, max_x, min_y, max_y, min_z, max_z) = bounds;
                        
                        // 快速边界检查
                        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z) {
                            // 详细几何检查
                            if (isPointInObstacleWithBuffer(x, y, z, obs, buffer_size)) {
                                env_->UpdateCellCost(x, y, z, 255);
                                total_marked++;
                                break;  // 一旦被标记，就不需要检查其他障碍物
                            }
                        }
                    }
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Batch marked %d voxels for %zu obstacles", total_marked, obstacles.size());
    }
    
    void PlanningTestNode::addObstacleToVoxelEnvironment(const Obstacle& obs, double buffer_size)
    {
        // Convert obstacle bounds to voxel coordinates with buffer expansion
        int min_x, min_y, min_z, max_x, max_y, max_z;
        
        // Expand obstacle bounds by buffer_size
        double expanded_size_x = obs.size_x + 2 * buffer_size;
        double expanded_size_y = obs.size_y + 2 * buffer_size;
        double expanded_size_z = obs.size_z + 2 * buffer_size;
        
        env_->WorldToVoxel(obs.center_x - expanded_size_x/2, obs.center_y - expanded_size_y/2, obs.center_z - expanded_size_z/2,
                          min_x, min_y, min_z);
        env_->WorldToVoxel(obs.center_x + expanded_size_x/2, obs.center_y + expanded_size_y/2, obs.center_z + expanded_size_z/2,
                          max_x, max_y, max_z);

        // 修复边界检查，确保障碍物能够正确标记到顶层体素
        if(min_x<0) min_x = 0;
        if(min_y<0) min_y = 0;
        // 改造：对于占据完整Z轴的障碍物，确保覆盖从底层到顶层
        if(min_z<0) min_z = 0;
        if(max_x>=env_x_) max_x = env_x_ - 1;
        if(max_y>=env_y_) max_y = env_y_ - 1;
        // 确保max_z能够到达顶层体素（索引env_z_-1）
        if(max_z>=env_z_) max_z = env_z_ - 1;
        
        // 添加调试信息
        RCLCPP_DEBUG(this->get_logger(), "Obstacle voxel bounds: x[%d,%d], y[%d,%d], z[%d,%d]", 
                    min_x, max_x, min_y, max_y, min_z, max_z);
        
        // 检查是否包含顶层体素
        if (max_z == env_z_ - 1) {
            RCLCPP_INFO(this->get_logger(), "Obstacle includes top layer voxels (z=%d)", max_z);
        }
        
        // 检查是否包含底层体素
        if (min_z == 0) {
            RCLCPP_INFO(this->get_logger(), "Obstacle includes bottom layer voxels (z=%d)", min_z);
        }
        
        // 优化：根据障碍物类型使用不同的标记策略
        int marked_count = 0;
        
        switch (obs.type) {
            case ObstacleType::BOX:
                marked_count = markBoxObstacle(obs, buffer_size, min_x, max_x, min_y, max_y, min_z, max_z);
                break;
            case ObstacleType::CYLINDER:
                marked_count = markCylinderObstacle(obs, buffer_size, min_x, max_x, min_y, max_y, min_z, max_z);
                break;
            case ObstacleType::SPHERE:
                marked_count = markSphereObstacle(obs, buffer_size, min_x, max_x, min_y, max_y, min_z, max_z);
                break;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Marked %d voxels for obstacle", marked_count);
    }
    
    int PlanningTestNode::markBoxObstacle(const Obstacle& obs, double buffer_size, 
                                         int min_x, int max_x, int min_y, int max_y, int min_z, int max_z)
    {
        int marked_count = 0;
        
        // 对于长方体，可以直接计算边界，无需逐点检查
        double obs_min_x = obs.center_x - obs.size_x/2 - buffer_size;
        double obs_max_x = obs.center_x + obs.size_x/2 + buffer_size;
        double obs_min_y = obs.center_y - obs.size_y/2 - buffer_size;
        double obs_max_y = obs.center_y + obs.size_y/2 + buffer_size;
        double obs_min_z = obs.center_z - obs.size_z/2 - buffer_size;
        double obs_max_z = obs.center_z + obs.size_z/2 + buffer_size;
        
        // 使用OpenMP并行化
        #pragma omp parallel for collapse(3) reduction(+:marked_count)
        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    // 快速边界检查
                    double world_x, world_y, world_z;
                    env_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
                    
                    if (world_x >= obs_min_x && world_x <= obs_max_x &&
                        world_y >= obs_min_y && world_y <= obs_max_y &&
                        world_z >= obs_min_z && world_z <= obs_max_z) {
                        env_->UpdateCellCost(x, y, z, 255);
                        marked_count++;
                    }
                }
            }
        }
        
        return marked_count;
    }
    
    int PlanningTestNode::markCylinderObstacle(const Obstacle& obs, double buffer_size,
                                              int min_x, int max_x, int min_y, int max_y, int min_z, int max_z)
    {
        int marked_count = 0;
        double radius_sq = (obs.size_x/2 + buffer_size) * (obs.size_x/2 + buffer_size);
        double obs_min_z = obs.center_z - obs.size_z/2 - buffer_size;
        double obs_max_z = obs.center_z + obs.size_z/2 + buffer_size;
        
        // 使用OpenMP并行化
        #pragma omp parallel for collapse(2) reduction(+:marked_count)
        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                double world_x, world_y, world_z;
                env_->VoxelToWorld(x, y, min_z, world_x, world_y, world_z);
                
                // 检查XY平面上的距离
                double dx = world_x - obs.center_x;
                double dy = world_y - obs.center_y;
                double dist_sq = dx*dx + dy*dy;
                
                if (dist_sq <= radius_sq) {
                    // 在圆柱范围内，标记整个Z轴
                    for (int z = min_z; z <= max_z; ++z) {
                        env_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
                        if (world_z >= obs_min_z && world_z <= obs_max_z) {
                            env_->UpdateCellCost(x, y, z, 255);
                            marked_count++;
                        }
                    }
                }
            }
        }
        
        return marked_count;
    }
    
    int PlanningTestNode::markSphereObstacle(const Obstacle& obs, double buffer_size,
                                            int min_x, int max_x, int min_y, int max_y, int min_z, int max_z)
    {
        int marked_count = 0;
        double radius_sq = (obs.size_x/2 + buffer_size) * (obs.size_x/2 + buffer_size);
        
        // 使用OpenMP并行化
        #pragma omp parallel for collapse(3) reduction(+:marked_count)
        for (int x = min_x; x <= max_x; ++x) {
            for (int y = min_y; y <= max_y; ++y) {
                for (int z = min_z; z <= max_z; ++z) {
                    double world_x, world_y, world_z;
                    env_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
                    
                    double dx = world_x - obs.center_x;
                    double dy = world_y - obs.center_y;
                    double dz = world_z - obs.center_z;
                    double dist_sq = dx*dx + dy*dy + dz*dz;
                    
                    if (dist_sq <= radius_sq) {
                        env_->UpdateCellCost(x, y, z, 255);
                        marked_count++;
                    }
                }
            }
        }
        
        return marked_count;
    }
    
    bool PlanningTestNode::isPointInObstacle(int voxel_x, int voxel_y, int voxel_z, const Obstacle& obs)
    {
        // Convert voxel coordinates to world coordinates
        double world_x, world_y, world_z;
        env_->VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
        
        // Check if point is inside obstacle based on type
        switch (obs.type) {
            case ObstacleType::BOX:
                return (world_x >= obs.center_x - obs.size_x/2 && world_x <= obs.center_x + obs.size_x/2 &&
                        world_y >= obs.center_y - obs.size_y/2 && world_y <= obs.center_y + obs.size_y/2 &&
                        world_z >= obs.center_z - obs.size_z/2 && world_z <= obs.center_z + obs.size_z/2);
                        
            case ObstacleType::CYLINDER:
                {
                    double dx = world_x - obs.center_x;
                    double dy = world_y - obs.center_y;
                    double radius_sq = (obs.size_x/2) * (obs.size_x/2);
                    return (dx*dx + dy*dy <= radius_sq &&
                            world_z >= obs.center_z - obs.size_z/2 && world_z <= obs.center_z + obs.size_z/2);
                }
                
            case ObstacleType::SPHERE:
                {
                    double dx = world_x - obs.center_x;
                    double dy = world_y - obs.center_y;
                    double dz = world_z - obs.center_z;
                    double radius_sq = (obs.size_x/2) * (obs.size_x/2);
                    return (dx*dx + dy*dy + dz*dz <= radius_sq);
                }
        }
        return false;
    }
    
    bool PlanningTestNode::isPointInObstacleWithBuffer(int voxel_x, int voxel_y, int voxel_z, const Obstacle& obs, double buffer_size)
    {
        // Convert voxel coordinates to world coordinates
        double world_x, world_y, world_z;
        env_->VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
        
        // Check if point is inside obstacle based on type with buffer
        switch (obs.type) {
            case ObstacleType::BOX:
                return (world_x >= obs.center_x - obs.size_x/2 - buffer_size && world_x <= obs.center_x + obs.size_x/2 + buffer_size &&
                        world_y >= obs.center_y - obs.size_y/2 - buffer_size && world_y <= obs.center_y + obs.size_y/2 + buffer_size &&
                        world_z >= obs.center_z - obs.size_z/2 - buffer_size && world_z <= obs.center_z + obs.size_z/2 + buffer_size);
                        
            case ObstacleType::CYLINDER:
                {
                    double dx = world_x - obs.center_x;
                    double dy = world_y - obs.center_y;
                    double radius_sq = (obs.size_x/2 + buffer_size) * (obs.size_x/2 + buffer_size);
                    return (dx*dx + dy*dy <= radius_sq &&
                            world_z >= obs.center_z - obs.size_z/2 - buffer_size && world_z <= obs.center_z + obs.size_z/2 + buffer_size);
                }
                
            case ObstacleType::SPHERE:
                {
                    double dx = world_x - obs.center_x;
                    double dy = world_y - obs.center_y;
                    double dz = world_z - obs.center_z;
                    double radius_sq = (obs.size_x/2 + buffer_size) * (obs.size_x/2 + buffer_size);
                    return (dx*dx + dy*dy + dz*dz <= radius_sq);
                }
        }
        return false;
    }
    
    void PlanningTestNode::generateStartGoalPairs()
    {
        RCLCPP_INFO(this->get_logger(), "Starting start-goal pair generation...");
        start_goal_pairs_.clear();
        
        // 定义起点和终点的范围
        const double start_x_min = 0.0;
        const double start_x_max = 20000.0;  // 左侧20km
        const double start_y_min = 0.0; 
        const double start_y_max = world_y_;  
        
        const double goal_x_min = world_x_ - 20000.0;  // 右侧20km带状区域
        const double goal_x_max = world_x_;
        const double goal_y_min = 0.0;
        const double goal_y_max = world_y_;
        
        for (int i = 0; i < 10; ++i) {
            StartGoalPair pair;
            
            // 生成起点 - 在左下角10km x 10km范围内
            int attempts = 0;
            const int max_attempts = 100;
            do {
                pair.start_x = start_x_min + uniform_dist_(gen_) * (start_x_max - start_x_min);
                pair.start_y = start_y_min + uniform_dist_(gen_) * (start_y_max - start_y_min);
                pair.start_z = uniform_dist_(gen_) * world_z_;
                attempts++;
                if (attempts >= max_attempts) {
                    RCLCPP_WARN(this->get_logger(), "Could not find free start point after %d attempts, using current point", max_attempts);
                    break;
                }
            } while (isPointOccupied(pair.start_x, pair.start_y, pair.start_z));
            
            // 生成终点 - 在右侧带状区域内
            attempts = 0;
            do {
                pair.goal_x = goal_x_min + uniform_dist_(gen_) * (goal_x_max - goal_x_min);
                pair.goal_y = goal_y_min + uniform_dist_(gen_) * (goal_y_max - goal_y_min);
                pair.goal_z = uniform_dist_(gen_) * world_z_;
                attempts++;
                if (attempts >= max_attempts) {
                    RCLCPP_WARN(this->get_logger(), "Could not find free goal point after %d attempts, using current point", max_attempts);
                    break;
                }
            } while (isPointOccupied(pair.goal_x, pair.goal_y, pair.goal_z));
            
            start_goal_pairs_.push_back(pair);
            
            RCLCPP_INFO(this->get_logger(), "Pair %d: start=(%.1f,%.1f,%.1f), goal=(%.1f,%.1f,%.1f)", 
                       i+1, pair.start_x, pair.start_y, pair.start_z, pair.goal_x, pair.goal_y, pair.goal_z);
        }
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu start-goal pairs", start_goal_pairs_.size());
    }
    
    bool PlanningTestNode::isPointOccupied(double x, double y, double z)
    {
        int voxel_x, voxel_y, voxel_z;
        if (env_->WorldToVoxel(x, y, z, voxel_x, voxel_y, voxel_z)) {
            return env_->IsCellOccupied(voxel_x, voxel_y, voxel_z);
        }
        return true;  // Out of bounds
    }
    
    void PlanningTestNode::runPlanningTests()
    {
        // Create planners using factory
        std::vector<std::shared_ptr<GlobalPlannerBase>> planners;
        std::vector<std::string> planner_names;
        
        // Create planners using factory
        try {
            // planners.push_back(PlannerFactory::createPlanner(PlannerFactory::PlannerType::ASTAR, env_.get()));
            // planner_names.push_back("A*");

            // 创建多分辨率A*规划器，指定3个分辨率层级
            auto planner = PlannerFactory::createPlanner(PlannerFactory::PlannerType::MULTISCALE_ASTAR, env_.get());
            std::static_pointer_cast<MultiScaleAStarPlanner>(planner)->initializeResolutionLevels(8);
            planners.push_back(planner);
            planner_names.push_back("MultiScale A* (4 levels)");
            
            // planners.push_back(PlannerFactory::createPlanner(PlannerFactory::PlannerType::THETASTAR, env_.get()));
            // planner_names.push_back("Theta*");
            
            // planners.push_back(PlannerFactory::createPlanner(PlannerFactory::PlannerType::ARASTAR, env_.get()));
            // planner_names.push_back("ARA*");
            
            // planners.push_back(PlannerFactory::createPlanner(PlannerFactory::PlannerType::JPS, env_.get()));
            // planner_names.push_back("JPS");
            
            RCLCPP_INFO(this->get_logger(), "Successfully created %zu planners using factory", planners.size());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create planners: %s", e.what());
            return;
        }
        
        // Test each planner with each start-goal pair
        for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
            
            const auto& pair = start_goal_pairs_[i];
            
            std::cout << "\n=== Testing Start-Goal Pair " << (i+1) << " ===" << std::endl;
            std::cout << "Start: (" << pair.start_x << ", " << pair.start_y << ", " << pair.start_z << ")" << std::endl;
            std::cout << "Goal:  (" << pair.goal_x << ", " << pair.goal_y << ", " << pair.goal_z << ")" << std::endl;
            
            // Test all planners
            for (size_t j = 0; j < planners.size(); ++j) {
                

                auto& planner = planners[j];
                
                if (!planner) {
                    RCLCPP_WARN(this->get_logger(), "Planner %zu is null, skipping", j);
                    continue;
                }
                
                std::cout << "\n--- " << planner_names[j] << " ---" << std::endl;
                
                // Plan path
                std::vector<geometry_msgs::msg::PoseStamped> path_msgs;

                bool success = planner->planPath(
                    pair.start_x, pair.start_y, pair.start_z,
                    pair.goal_x, pair.goal_y, pair.goal_z,
                    path_msgs, 
                    SmootherType::NONE,  // 不在规划器内部进行平滑
                    5.0  // 15 second timeout
                );
                
                if (success) {
                    std::cout << "  Success! Path length: " << path_msgs.size() << " points" << std::endl;
                    
                    // Store path for visualization
                    Path path;
                    path.planner_type = j;
                    path.pair_index = i;
                    
                    // 获取原始路径（所有规划器现在都支持 getOriginalPath）
                    path.original_path = planner->getOriginalPath();
                    std::cout << "  Original path length: " << path.original_path.size() << " points" << std::endl;
                    
                    // 在测试节点中进行路径平滑
                    SmootherType smooth_type = SmootherType::BSPLINE;  // 可以根据需要修改
                    if (smooth_type != SmootherType::NONE) {
                        auto smoother = SmootherFactory::createSmoother(smooth_type, 3, 32);
                        if (smoother) {
                            std::vector<geometry_msgs::msg::PoseStamped> smoothed_path, smoothed_path_2;
                            if (smoother->smoothPath(path.original_path, smoothed_path)) {
                                smoother->smoothPath(smoothed_path, smoothed_path_2);
                                path.smoothed_path = smoothed_path_2;
                                std::cout << "  Smoothed path length: " << path.smoothed_path.size() << " points" << std::endl;
                            } else {
                                std::cout << "  Smoothing failed, using original path" << std::endl;
                                path.smoothed_path = path.original_path;
                            }
                        } else {
                            std::cout << "  Failed to create smoother, using original path" << std::endl;
                            path.smoothed_path = path.original_path;
                        }
                    } else {
                        // 不进行平滑，使用原始路径
                        path.smoothed_path = path.original_path;
                    }
                    
                    paths_.push_back(path);
                } else {
                    std::cout << "  Failed to find path" << std::endl;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning tests completed. Generated %zu paths", paths_.size());
    }
    
    void PlanningTestNode::publishVisualization()
    {
        // Publish environment bounds
        auto bounds_array = visualization_msgs::msg::MarkerArray();
        publishEnvironmentBounds(bounds_array);
        environment_bounds_pub_->publish(bounds_array);
        
        // Publish obstacles
        auto obstacles_array = visualization_msgs::msg::MarkerArray();
        publishObstacles(obstacles_array);
        obstacles_pub_->publish(obstacles_array);
        
        // Publish voxel grid
        auto voxel_array = visualization_msgs::msg::MarkerArray();
        // publishVoxelGrid(voxel_array);
        // voxel_grid_pub_->publish(voxel_array);
        
        // Publish start-goal points
        auto start_goal_array = visualization_msgs::msg::MarkerArray();
        publishStartGoalPoints(start_goal_array);
        start_goal_pub_->publish(start_goal_array);
        
        // Publish paths
        auto paths_array = visualization_msgs::msg::MarkerArray();
        publishPaths(paths_array);
        paths_pub_->publish(paths_array);
        
        // Publish original paths
        auto original_paths_array = visualization_msgs::msg::MarkerArray();
        publishOriginalPaths(original_paths_array);
        original_paths_pub_->publish(original_paths_array);
        
        // Publish smoothed paths
        auto smoothed_paths_array = visualization_msgs::msg::MarkerArray();
        publishSmoothedPaths(smoothed_paths_array);
        smoothed_paths_pub_->publish(smoothed_paths_array);
        
        // Also publish everything to the original topic for backward compatibility
        auto combined_array = visualization_msgs::msg::MarkerArray();
        // publishEnvironmentBounds(combined_array);
        // publishObstacles(combined_array);
        // publishVoxelGrid(combined_array);
        // publishStartGoalPoints(combined_array);
        // publishPaths(combined_array);
        // marker_pub_->publish(combined_array);
        
        RCLCPP_INFO(this->get_logger(), "Published visualization markers:");
        RCLCPP_INFO(this->get_logger(), "  - Environment bounds: %zu markers", bounds_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Obstacles: %zu markers", obstacles_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Voxel grid: %zu markers", voxel_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Start/Goal points: %zu markers", start_goal_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Paths: %zu markers", paths_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Original paths: %zu markers", original_paths_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Smoothed paths: %zu markers", smoothed_paths_array.markers.size());
        RCLCPP_INFO(this->get_logger(), "  - Combined: %zu markers", combined_array.markers.size());
    }
    
   
    
    void PlanningTestNode::publishVoxelGrid(visualization_msgs::msg::MarkerArray& marker_array)
    {
        if (!env_) {
            RCLCPP_WARN(this->get_logger(), "Environment not initialized, skipping voxel grid visualization");
            return;
        }
        
        // 创建占用体素的标记
        auto occupied_voxels_marker = visualization_msgs::msg::Marker();
        occupied_voxels_marker.header.frame_id = "world";
        occupied_voxels_marker.header.stamp = this->now();
        occupied_voxels_marker.ns = "voxel_grid";
        occupied_voxels_marker.id = 0;
        occupied_voxels_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_voxels_marker.action = visualization_msgs::msg::Marker::ADD;
        
        occupied_voxels_marker.pose.orientation.w = 1.0;
        
        // 设置体素大小（略小于实际体素尺寸以避免重叠）
        occupied_voxels_marker.scale.x = resolution_xy_ * 0.9;
        occupied_voxels_marker.scale.y = resolution_xy_ * 0.9;
        occupied_voxels_marker.scale.z = resolution_z_ * 0.9;
        
        // 设置占用体素颜色（红色）
        occupied_voxels_marker.color.r = 1.0;
        occupied_voxels_marker.color.g = 0.0;
        occupied_voxels_marker.color.b = 0.0;
        occupied_voxels_marker.color.a = 0.6;
        
        // 遍历所有体素，添加占用的体素
        int occupied_count = 0;
        for (int x = 0; x < env_x_; ++x) {
            for (int y = 0; y < env_y_; ++y) {
                for (int z = 0; z < env_z_; ++z) {
                    if (env_->IsCellOccupied(x, y, z)) {
                        geometry_msgs::msg::Point point;
                        double world_x, world_y, world_z;
                        env_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
                        point.x = world_x;
                        point.y = world_y;
                        point.z = world_z;
                        occupied_voxels_marker.points.push_back(point);
                        occupied_count++;
                    }
                }
            }
        }
        
        marker_array.markers.push_back(occupied_voxels_marker);
        
        // 创建体素网格线框标记
        auto grid_lines_marker = visualization_msgs::msg::Marker();
        grid_lines_marker.header.frame_id = "world";
        grid_lines_marker.header.stamp = this->now();
        grid_lines_marker.ns = "voxel_grid";
        grid_lines_marker.id = 1;
        grid_lines_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        grid_lines_marker.action = visualization_msgs::msg::Marker::ADD;
        
        grid_lines_marker.pose.orientation.w = 1.0;
        grid_lines_marker.scale.x = 1.0;  // 线宽
        
        // 设置网格线颜色（浅灰色）
        grid_lines_marker.color.r = 0.5;
        grid_lines_marker.color.g = 0.5;
        grid_lines_marker.color.b = 0.5;
        grid_lines_marker.color.a = 0.3;
        
        // 添加主要的网格线（每隔10个体素添加一条线以减少视觉混乱）
        int grid_step = 10;
        
        // X方向的网格线
        for (int y = 0; y <= env_y_; y += grid_step) {
            for (int z = 0; z <= env_z_; z += grid_step) {
                    double world_x1, world_y1, world_z1, world_x2, world_y2, world_z2;
                env_->VoxelToWorld(0, y, z, world_x1, world_y1, world_z1);
                env_->VoxelToWorld(env_x_, y, z, world_x2, world_y2, world_z2);
                addLine(grid_lines_marker, world_x1, world_y1, world_z1, world_x2, world_y2, world_z2);
            }
        }
        
        // Y方向的网格线
        for (int x = 0; x <= env_x_; x += grid_step) {
            for (int z = 0; z <= env_z_; z += grid_step) {
                double world_x1, world_y1, world_z1, world_x2, world_y2, world_z2;
                env_->VoxelToWorld(x, 0, z, world_x1, world_y1, world_z1);
                env_->VoxelToWorld(x, env_y_, z, world_x2, world_y2, world_z2);
                addLine(grid_lines_marker, world_x1, world_y1, world_z1, world_x2, world_y2, world_z2);
            }
        }
        
        // Z方向的网格线
        for (int x = 0; x <= env_x_; x += grid_step) {
            for (int y = 0; y <= env_y_; y += grid_step) {
                double world_x1, world_y1, world_z1, world_x2, world_y2, world_z2;
                env_->VoxelToWorld(x, y, 0, world_x1, world_y1, world_z1);
                env_->VoxelToWorld(x, y, env_z_, world_x2, world_y2, world_z2);
                addLine(grid_lines_marker, world_x1, world_y1, world_z1, world_x2, world_y2, world_z2);
            }
        }
        
        marker_array.markers.push_back(grid_lines_marker);
        
        RCLCPP_INFO(this->get_logger(), "Published voxel grid: %d occupied voxels out of %d total", 
                   occupied_count, env_x_ * env_y_ * env_z_);
    }
    
    void PlanningTestNode::addLine(visualization_msgs::msg::Marker& marker, double x1, double y1, double z1, 
                 double x2, double y2, double z2)
    {
        geometry_msgs::msg::Point p1, p2;
        p1.x = x1; p1.y = y1; p1.z = z1;
        p2.x = x2; p2.y = y2; p2.z = z2;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    
    void PlanningTestNode::publishEnvironmentBounds(visualization_msgs::msg::MarkerArray& marker_array)
    {
        // 创建环境边界框标记
        auto bounds_marker = visualization_msgs::msg::Marker();
        bounds_marker.header.frame_id = "world";
        bounds_marker.header.stamp = this->now();
        bounds_marker.ns = "environment_bounds";
        bounds_marker.id = 0;
        bounds_marker.type = visualization_msgs::msg::Marker::CUBE;
        bounds_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置边界框位置（环境中心）
        bounds_marker.pose.position.x = world_x_ / 2.0;
        bounds_marker.pose.position.y = world_y_ / 2.0;
        bounds_marker.pose.position.z = world_z_ / 2.0;
        bounds_marker.pose.orientation.w = 1.0;
        
        // 设置边界框尺寸
        bounds_marker.scale.x = world_x_;
        bounds_marker.scale.y = world_y_;
        bounds_marker.scale.z = world_z_;
        
        // 设置边界框颜色（半透明灰色）
        bounds_marker.color.r = 0.7;
        bounds_marker.color.g = 0.7;
        bounds_marker.color.b = 0.7;
        bounds_marker.color.a = 0.1;
        
        marker_array.markers.push_back(bounds_marker);
        
        // 创建边界线框标记
        auto wireframe_marker = visualization_msgs::msg::Marker();
        wireframe_marker.header.frame_id = "world";
        wireframe_marker.header.stamp = this->now();
        wireframe_marker.ns = "environment_bounds";
        wireframe_marker.id = 1;
        wireframe_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        wireframe_marker.action = visualization_msgs::msg::Marker::ADD;
        
        wireframe_marker.pose.orientation.w = 1.0;
        wireframe_marker.scale.x = 5.0;  // 线宽
        
        // 设置线框颜色（深灰色）
        wireframe_marker.color.r = 0.3;
        wireframe_marker.color.g = 0.3;
        wireframe_marker.color.b = 0.3;
        wireframe_marker.color.a = 0.8;
        
        // 添加边界线框的12条边
        double min_x = 0.0, min_y = 0.0, min_z = 0.0;
        double max_x = world_x_, max_y = world_y_, max_z = world_z_;
        
        // 底面四条边
        addLine(wireframe_marker, min_x, min_y, min_z, max_x, min_y, min_z);
        addLine(wireframe_marker, min_x, min_y, min_z, min_x, max_y, min_z);
        addLine(wireframe_marker, max_x, min_y, min_z, max_x, max_y, min_z);
        addLine(wireframe_marker, min_x, max_y, min_z, max_x, max_y, min_z);
        
        // 顶面四条边
        addLine(wireframe_marker, min_x, min_y, max_z, max_x, min_y, max_z);
        addLine(wireframe_marker, min_x, min_y, max_z, min_x, max_y, max_z);
        addLine(wireframe_marker, max_x, min_y, max_z, max_x, max_y, max_z);
        addLine(wireframe_marker, min_x, max_y, max_z, max_x, max_y, max_z);
        
        // 四条垂直边
        addLine(wireframe_marker, min_x, min_y, min_z, min_x, min_y, max_z);
        addLine(wireframe_marker, max_x, min_y, min_z, max_x, min_y, max_z);
        addLine(wireframe_marker, min_x, max_y, min_z, min_x, max_y, max_z);
        addLine(wireframe_marker, max_x, max_y, min_z, max_x, max_y, max_z);
        
        marker_array.markers.push_back(wireframe_marker);
        
        // 创建坐标轴标记
        auto axis_marker = visualization_msgs::msg::Marker();
        axis_marker.header.frame_id = "world";
        axis_marker.header.stamp = this->now();
        axis_marker.ns = "environment_bounds";
        axis_marker.id = 2;
        axis_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        axis_marker.action = visualization_msgs::msg::Marker::ADD;
        
        axis_marker.pose.orientation.w = 1.0;
        axis_marker.scale.x = 10.0;  // 线宽
        
        // 添加坐标轴
        // X轴（红色）
        addLine(axis_marker, 0, 0, 0, world_x_ * 0.1, 0, 0);
        // Y轴（绿色）
        addLine(axis_marker, 0, 0, 0, 0, world_y_ * 0.1, 0);
        // Z轴（蓝色）
        addLine(axis_marker, 0, 0, 0, 0, 0, world_z_ * 0.1);
        
        // 设置坐标轴颜色
        axis_marker.color.r = 1.0;
        axis_marker.color.g = 0.0;
        axis_marker.color.b = 0.0;
        axis_marker.color.a = 1.0;
        
        marker_array.markers.push_back(axis_marker);
        
        RCLCPP_INFO(this->get_logger(), "Published environment bounds: %.1f x %.1f x %.1f m", 
                   world_x_, world_y_, world_z_);
    }
    
    void PlanningTestNode::publishObstacles(visualization_msgs::msg::MarkerArray& marker_array)
    {
        for (size_t i = 0; i < obstacles_.size(); ++i) {
            const auto& obs = obstacles_[i];
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = i;
            
            // Set marker type based on obstacle type
            switch (obs.type) {
                case ObstacleType::BOX:
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    break;
                case ObstacleType::CYLINDER:
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    break;
                case ObstacleType::SPHERE:
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    break;
            }
            
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = obs.center_x;
            marker.pose.position.y = obs.center_y;
            marker.pose.position.z = obs.center_z;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = obs.size_x;
            marker.scale.y = obs.size_y;
            marker.scale.z = obs.size_z;
            
            // Different colors for different obstacle types
            switch (obs.type) {
                case ObstacleType::BOX:
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    break;
                case ObstacleType::CYLINDER:
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    break;
                case ObstacleType::SPHERE:
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    break;
            }
            marker.color.a = 0.7;
            
            marker_array.markers.push_back(marker);
        }
    }
    
    void PlanningTestNode::publishStartGoalPoints(visualization_msgs::msg::MarkerArray& marker_array)
    {
        for (size_t i = 0; i < start_goal_pairs_.size(); ++i) {
            const auto& pair = start_goal_pairs_[i];
            
            // Start point (green sphere)
            auto start_marker = visualization_msgs::msg::Marker();
            start_marker.header.frame_id = "world";
            start_marker.header.stamp = this->now();
            start_marker.ns = "start_points";
            start_marker.id = i;
            start_marker.type = visualization_msgs::msg::Marker::SPHERE;
            start_marker.action = visualization_msgs::msg::Marker::ADD;
            
            start_marker.pose.position.x = pair.start_x;
            start_marker.pose.position.y = pair.start_y;
            start_marker.pose.position.z = pair.start_z;
            start_marker.pose.orientation.w = 1.0;
            
            start_marker.scale.x = 50.0;
            start_marker.scale.y = 50.0;
            start_marker.scale.z = 50.0;
            
            start_marker.color.r = 0.0;
            start_marker.color.g = 1.0;
            start_marker.color.b = 0.0;
            start_marker.color.a = 1.0;
            
            marker_array.markers.push_back(start_marker);
            
            // Goal point (red sphere)
            auto goal_marker = visualization_msgs::msg::Marker();
            goal_marker.header.frame_id = "world";
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "goal_points";
            goal_marker.id = i;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            
            goal_marker.pose.position.x = pair.goal_x;
            goal_marker.pose.position.y = pair.goal_y;
            goal_marker.pose.position.z = pair.goal_z;
            goal_marker.pose.orientation.w = 1.0;
            
            goal_marker.scale.x = 50.0;
            goal_marker.scale.y = 50.0;
            goal_marker.scale.z = 50.0;
            
            goal_marker.color.r = 1.0;
            goal_marker.color.g = 0.0;
            goal_marker.color.b = 0.0;
            goal_marker.color.a = 1.0;
            
            marker_array.markers.push_back(goal_marker);
        }
    }
    
    void PlanningTestNode::publishPaths(visualization_msgs::msg::MarkerArray& marker_array)
    {
        std::vector<std::vector<double>> colors = {
            {1.0, 0.0, 0.0},  // Red for A*
            {0.0, 1.0, 0.0},  // Green for Theta*
            {0.0, 0.0, 1.0},  // Blue for ARA*
            {0.0, 1.0, 1.0},  // Cyan for JPS
            {1.0, 0.0, 1.0}   // Magenta for MultiScale A*
        };
        
        std::vector<std::string> planner_names = {"A*", "Theta*", "ARA*", "JPS", "MultiScale A*"};
        
        for (const auto& path : paths_) {
            if (path.smoothed_path.empty()) continue;
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "paths";
            marker.id = marker_array.markers.size();
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 10.0;  // Line width
            
            // Set color based on planner type
            if (path.planner_type < colors.size()) {
                marker.color.r = colors[path.planner_type][0];
                marker.color.g = colors[path.planner_type][1];
                marker.color.b = colors[path.planner_type][2];
            } else {
                // Default color for unknown planner types
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }
            marker.color.a = 0.8;
            
            // Add path points
            for (size_t i = 0; i < path.smoothed_path.size(); ++i) {
                geometry_msgs::msg::Point point;
                point.x = path.smoothed_path[i].pose.position.x;
                point.y = path.smoothed_path[i].pose.position.y;
                point.z = path.smoothed_path[i].pose.position.z;
                marker.points.push_back(point);
            }
            
            marker_array.markers.push_back(marker);
        }
    }
    
    void PlanningTestNode::publishOriginalPaths(visualization_msgs::msg::MarkerArray& marker_array)
    {
        for (const auto& path : paths_) {
            if (path.original_path.empty()) continue;
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "original_paths";
            marker.id = marker_array.markers.size();
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 15.0;  // Thinner line for original paths
            
            // 平滑前的路径使用黄色
            marker.color.r = 1.0;  // 黄色：红色分量
            marker.color.g = 1.0;  // 黄色：绿色分量
            marker.color.b = 0.0;  // 黄色：蓝色分量
            marker.color.a = 0.8;  // 透明度
            
            // Add path points
            for (size_t i = 0; i < path.original_path.size(); ++i) {
                geometry_msgs::msg::Point point;
                point.x = path.original_path[i].pose.position.x;
                point.y = path.original_path[i].pose.position.y;
                point.z = path.original_path[i].pose.position.z;
                marker.points.push_back(point);
            }
            
            marker_array.markers.push_back(marker);
        }
    }
    
    void PlanningTestNode::publishSmoothedPaths(visualization_msgs::msg::MarkerArray& marker_array)
    {
        for (const auto& path : paths_) {
            if (path.smoothed_path.empty()) continue;
            
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "smoothed_paths";
            marker.id = marker_array.markers.size();
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 25.0;  // Thicker line for smoothed paths
            
            // 平滑后的路径使用蓝色
            marker.color.r = 1.0;  // 蓝色：红色分量
            marker.color.g = 0.0;  // 蓝色：绿色分量
            marker.color.b = 0.0;  // 蓝色：蓝色分量
            marker.color.a = 0.2;  // 透明度
            
            // Add path points
            for (size_t i = 0; i < path.smoothed_path.size(); ++i) {
                geometry_msgs::msg::Point point;
                point.x = path.smoothed_path[i].pose.position.x;
                point.y = path.smoothed_path[i].pose.position.y;
                point.z = path.smoothed_path[i].pose.position.z;
                marker.points.push_back(point);
            }
            
            marker_array.markers.push_back(marker);
        }
    }
    
    void PlanningTestNode::publishMapTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = "map";
        
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Published map transform");
    }
    
    void PlanningTestNode::publishOctomap()
    {
        auto octomap = env_->GetOctomap();
        if (!octomap) {
            RCLCPP_WARN(this->get_logger(), "No octomap available");
            return;
        }
        
        // Create octomap message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = "world";
        octomap_msg.header.stamp = this->now();
        octomap_msg.binary = false;
        octomap_msg.id = "OcTree";
        
        // Convert full octomap to message
        if (octomap_msgs::fullMapToMsg(*octomap, octomap_msg)) {
            octomap_pub_->publish(octomap_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to message");
        }
        
        // Create binary octomap message
        octomap_msgs::msg::Octomap binary_octomap_msg;
        binary_octomap_msg.header.frame_id = "world";
        binary_octomap_msg.header.stamp = this->now();
        binary_octomap_msg.binary = true;
        binary_octomap_msg.id = "OcTree";
        
        // Convert octomap to binary message
        if (octomap_msgs::binaryMapToMsg(*octomap, binary_octomap_msg)) {
            binary_octomap_pub_->publish(binary_octomap_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to binary message");
        }
    }
    
    void PlanningTestNode::verifyTopLayerVoxelMarking()
    {
        if (!env_) {
            RCLCPP_WARN(this->get_logger(), "Environment not initialized, cannot verify top layer marking");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Verifying top layer voxel marking...");
        
        int top_layer_occupied = 0;
        int top_layer_total = env_x_ * env_y_;
        
        // 检查顶层体素（z = env_z_ - 1）
        for (int x = 0; x < env_x_; ++x) {
            for (int y = 0; y < env_y_; ++y) {
                int z = env_z_ - 1;  // 顶层
                if (env_->IsCellOccupied(x, y, z)) {
                    top_layer_occupied++;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Top layer voxel status: %d/%d occupied (%.1f%%)", 
                   top_layer_occupied, top_layer_total, 
                   (double)top_layer_occupied / top_layer_total * 100.0);
        
        // 检查是否有障碍物延伸到顶层
        bool has_top_layer_obstacles = false;
        for (const auto& obs : obstacles_) {
            double expanded_size_z = obs.size_z + 2 * 1000.0;  // 使用相同的缓冲大小
            double obs_min_z = obs.center_z - expanded_size_z/2;
            double obs_max_z = obs.center_z + expanded_size_z/2;
            
            if (obs_max_z >= world_z_) {
                has_top_layer_obstacles = true;
                RCLCPP_INFO(this->get_logger(), "Obstacle extends to top layer: center_z=%.1f, size_z=%.1f, max_z=%.1f", 
                           obs.center_z, obs.size_z, obs_max_z);
            }
        }
        
        if (!has_top_layer_obstacles) {
            RCLCPP_WARN(this->get_logger(), "No obstacles extend to the top layer - this may be expected behavior");
        }
        
        if (top_layer_occupied == 0) {
            RCLCPP_WARN(this->get_logger(), "No voxels marked in top layer - this may indicate an issue");
        } else {
            RCLCPP_INFO(this->get_logger(), "Top layer voxel marking verification complete - %d voxels marked", top_layer_occupied);
        }
    }


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PlanningTestNode>();
    
    // Keep the node running to publish visualization
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 