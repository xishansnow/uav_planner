/*
 * Multi-Scale A* Path Planner Implementation
 * Leverages Octomap's multi-resolution structure for efficient path planning
 * Uses hierarchical search from coarse to fine resolution
 */

#ifndef MULTISCALE_ASTAR_PLANNER_HPP
#define MULTISCALE_ASTAR_PLANNER_HPP

#include "global_planner_base.hpp"
#include "common_types.hpp"
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>
#include <memory>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>

// 多尺度A*节点结构
struct MultiScaleAStarNode
{
    int x, y, z;
    int resolution_level;  // 分辨率层级 (0=最粗, max_depth=最细)
    double g_cost;         // 从起点到当前节点的代价
    double h_cost;         // 启发式代价
    double f_cost;         // 总代价 (g + h)
    int parent_x, parent_y, parent_z;
    int parent_resolution_level;
    
    // 默认构造函数
    MultiScaleAStarNode() : x(0), y(0), z(0), resolution_level(0), g_cost(0.0), 
                           h_cost(0.0), f_cost(0.0), parent_x(0), parent_y(0), 
                           parent_z(0), parent_resolution_level(0) {}
    
    MultiScaleAStarNode(int x, int y, int z, int level, double g, double h, 
                       int px, int py, int pz, int p_level)
        : x(x), y(y), z(z), resolution_level(level), g_cost(g), h_cost(h), 
          f_cost(g + h), parent_x(px), parent_y(py), parent_z(pz), 
          parent_resolution_level(p_level) {}
    
    // 优先级队列比较
    bool operator>(const MultiScaleAStarNode& other) const {
        return f_cost > other.f_cost;
    }
    
    // 集合比较
    bool operator<(const MultiScaleAStarNode& other) const {
        if (resolution_level != other.resolution_level) 
            return resolution_level < other.resolution_level;
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

// 分辨率层级配置
struct ResolutionLevel
{
    int level;           // 层级 (0=最粗)
    double resolution;   // 该层级的分辨率
    int step_size;       // 该层级的步长
    double cost_factor;  // 该层级的代价因子
    
    ResolutionLevel(int l, double res, int step, double cost) 
        : level(l), resolution(res), step_size(step), cost_factor(cost) {}
};

/**
 * \brief Multi-Scale A* path planning algorithm implementation
 * Leverages Octomap's hierarchical structure for efficient path planning
 * Uses coarse-to-fine resolution strategy
 */
class MultiScaleAStarPlanner : public GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    MultiScaleAStarPlanner(EnvironmentVoxel3D* env);
    
    /**
     * \brief Destructor
     */
    ~MultiScaleAStarPlanner();
    
    /**
     * \brief Initialize resolution levels
     */
    void initializeResolutionLevels(int max_resolution_levels=4);
    /**
     * \brief Plan path using multi-scale A* algorithm
     */
    bool planPath(double start_x, double start_y, double start_z,
                  double goal_x, double goal_y, double goal_z,
                  std::vector<geometry_msgs::msg::PoseStamped>& path,
                  SmootherType smooth_type,
                  double max_time = 5.0) override;
    
    /**
     * \brief Replan path
     */
    bool replanPath(const geometry_msgs::msg::PoseStamped& current_position,
                   const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
                   std::vector<geometry_msgs::msg::PoseStamped>& new_path,
                   SmootherType smooth_type = SmootherType::BSPLINE,
                   double max_time = 5.0) override;
    
    /**
     * \brief Set algorithm parameters
     */
    void setParameters(const std::string& param_name, double value) override;
    
    /**
     * \brief Get algorithm name
     */
    std::string getAlgorithmName() const override { return "MultiScale A*"; }
    
    /**
     * \brief Get the original path before smoothing
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOriginalPath() const { return original_path_; }
    
    /**
     * \brief Set maximum resolution levels for search
     */
    void setMaxResolutionLevels(int max_levels);
    
    /**
     * \brief Set minimum resolution for fine search
     */
    void setMinResolution(double min_res);
    
    /**
     * \brief Enable/disable adaptive resolution
     */
    void setAdaptiveResolution(bool enable);

private:
    // 算法参数
    double heuristic_weight_;
    double diagonal_cost_;
    double vertical_cost_;
    int max_resolution_levels_;
    double min_resolution_;
    bool adaptive_resolution_;
    
    // 路径存储
    std::vector<geometry_msgs::msg::PoseStamped> original_path_;
    
    // 分辨率层级配置
    std::vector<ResolutionLevel> resolution_levels_;
    
    // 邻居方向 (26连通)
    static const int num_neighbors_ = 26;
    static const int neighbor_dirs_[26][3];
    
    // 核心算法函数
    bool planPathMultiScale(double start_x, double start_y, double start_z,
                           double goal_x, double goal_y, double goal_z,
                           std::vector<geometry_msgs::msg::PoseStamped>& path,
                           double max_time);
    
    bool planPathAtResolution(int resolution_level,
                             const MultiScaleAStarNode& start_node,
                             const MultiScaleAStarNode& goal_node,
                             std::vector<MultiScaleAStarNode>& path,
                             double max_time);
    
    // 分辨率管理

    int getOptimalStartLevel(double start_x, double start_y, double start_z,
                            double goal_x, double goal_y, double goal_z) const;
    bool isValidPositionAtResolution(int x, int y, int z, int resolution_level) const;
    
    // 坐标转换
    void worldToResolutionCoords(double world_x, double world_y, double world_z,
                                int resolution_level,
                                int& res_x, int& res_y, int& res_z) const;
    void resolutionCoordsToWorld(int res_x, int res_y, int res_z,
                                int resolution_level,
                                double& world_x, double& world_y, double& world_z) const;
    void resolutionCoordsToResolutionCoords(int from_x, int from_y, int from_z, int from_level,
                                           int to_level, int& to_x, int& to_y, int& to_z) const;
    
    // 路径处理
    std::vector<geometry_msgs::msg::PoseStamped> convertToWorldPath(
        const std::vector<MultiScaleAStarNode>& resolution_path) const;
    std::vector<MultiScaleAStarNode> refinePathAtHigherResolution(
        const std::vector<MultiScaleAStarNode>& coarse_path,
        int target_resolution_level);
    
    // 启发式和代价计算
    double calculateHeuristic(int x1, int y1, int z1, int level1,
                             int x2, int y2, int z2, int level2) const;
    double calculateCost(int x1, int y1, int z1, int level1,
                        int x2, int y2, int z2, int level2) const;
    
    // 邻居生成
    std::vector<MultiScaleAStarNode> getNeighbors(const MultiScaleAStarNode& node) const;
    
    // 路径重建
    std::vector<MultiScaleAStarNode> reconstructPath(
        const std::unordered_map<std::string, MultiScaleAStarNode>& came_from,
        const MultiScaleAStarNode& goal_node) const;
    std::vector<MultiScaleAStarNode> reconstructPathImproved(
        const std::unordered_map<Coord3D, MultiScaleAStarNode>& came_from,
        const MultiScaleAStarNode& goal_node) const;
    
    // 辅助函数
    std::string nodeToKey(const MultiScaleAStarNode& node) const;
    bool isPathValid(const std::vector<MultiScaleAStarNode>& path) const;
    double estimatePathComplexity(double start_x, double start_y, double start_z,
                                 double goal_x, double goal_y, double goal_z) const;
    
    // 科学的分辨率选择函数
    double calculateSearchComplexity(double start_x, double start_y, double start_z,
                                    double goal_x, double goal_y, double goal_z,
                                    double distance) const;
    double calculateObstacleDensity(double start_x, double start_y, double start_z,
                                   double goal_x, double goal_y, double goal_z,
                                   double distance) const;
    int calculateOptimalResolutionLevel(double distance, double search_complexity,
                                       double obstacle_density, int grid_size_x, int grid_size_y, int grid_size_z,
                                       double base_resolution) const;
    double calculateEfficiencyScore(int level, double search_space_size,
                                   double obstacle_density, double search_complexity,
                                   double distance) const;
};

#endif // MULTISCALE_ASTAR_PLANNER_HPP 