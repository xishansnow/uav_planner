/*
 * A* Path Planner Implementation
 * Standard A* algorithm for 3D path planning
 */

#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include "single_global/global_planner_base.hpp"
#include "common_types.hpp"
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>

// 用于保存 A* 算法中的节点信息
struct AStarNode
{
    int x, y, z;    // voxel coordinates
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current to goal
    double f_cost;  // Total cost (g + h)
    int parent_x, parent_y, parent_z;
    
    // Default constructor for unordered_map compatibility
    AStarNode() : x(0), y(0), z(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), 
                  parent_x(0), parent_y(0), parent_z(0) {}
    
    AStarNode(int x, int y, int z, double g, double h, int px, int py, int pz)
        : x(x), y(y), z(z), g_cost(g), h_cost(h), f_cost(g + h),
          parent_x(px), parent_y(py), parent_z(pz) {}
    
    // For priority queue comparison
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
    
    // For priority queue comparison
    bool operator<(const AStarNode& other) const {
        return f_cost < other.f_cost;
    }
};

/**
 * \brief A* path planning algorithm implementation
 * Standard A* algorithm for 3D grid-based path planning
 */
class AStarPlanner : public GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    AStarPlanner(EnvironmentVoxel3D* env);
    
    /**
     * \brief Destructor
     */
    ~AStarPlanner();
    
    /**
     * \brief Plan path using A* algorithm
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
    std::string getAlgorithmName() const override { return "A*"; }
    
    /**
     * \brief Get original path before smoothing
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOriginalPath() const override { return original_path_; }

private:
    // Algorithm parameters
    double heuristic_weight_;
    double diagonal_cost_;
    double vertical_cost_;
    
    // Original path before smoothing
    std::vector<geometry_msgs::msg::PoseStamped> original_path_;
    
    // Neighbor directions (26-connected)
    static const int num_neighbors_ = 26;
    static const int neighbor_dirs_[26][3];
    
    // Helper functions
    double calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const;
    double calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<AStarNode> getNeighbors(const AStarNode& node) const;
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const std::unordered_map<Coord3D, AStarNode>& came_from,
        int goal_x, int goal_y, int goal_z) const;
    
    // A* specific functions
    // bool isValidPosition(int x, int y, int z) const;
};

#endif // ASTAR_PLANNER_HPP 