/*
 * JPS (Jump Point Search) Path Planner Implementation
 * Optimized A* algorithm using jump points to reduce search space
 */

#ifndef JPS_PLANNER_HPP
#define JPS_PLANNER_HPP

#include "single_global/global_planner_base.hpp"
#include "common_types.hpp"
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>

struct JPSNode
{
    int x, y, z;
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current to goal
    double f_cost;  // Total cost (g + h)
    int parent_x, parent_y, parent_z;
    int jump_direction[3];  // Direction of the jump that led to this node
    
    // Default constructor for unordered_map compatibility
    JPSNode() : x(0), y(0), z(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), 
                parent_x(0), parent_y(0), parent_z(0) {
        jump_direction[0] = 0;
        jump_direction[1] = 0;
        jump_direction[2] = 0;
    }
    
    JPSNode(int x, int y, int z, double g, double h, int px, int py, int pz)
        : x(x), y(y), z(z), g_cost(g), h_cost(h), f_cost(g + h),
          parent_x(px), parent_y(py), parent_z(pz) {
        jump_direction[0] = 0;
        jump_direction[1] = 0;
        jump_direction[2] = 0;
    }
    
    // For priority queue comparison
    bool operator>(const JPSNode& other) const {
        return f_cost > other.f_cost;
    }
    
    // For set comparison
    bool operator<(const JPSNode& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

/**
 * \brief JPS path planning algorithm implementation
 * JPS is an optimized A* algorithm that uses jump points to reduce
 * the number of nodes explored during path planning
 */
class JPSPlanner : public GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    JPSPlanner(EnvironmentVoxel3D* env);
    
    /**
     * \brief Destructor
     */
    ~JPSPlanner();
    
    /**
     * \brief Plan path using JPS algorithm
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
    std::string getAlgorithmName() const override { return "JPS"; }
    
    /**
     * \brief Get original path before smoothing
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOriginalPath() const override { return original_path_; }

private:
    // Algorithm parameters
    double heuristic_weight_;
    double diagonal_cost_;
    double vertical_cost_;
    int max_jump_distance_;  // Maximum distance to jump in one step
    
    // Original path before smoothing
    std::vector<geometry_msgs::msg::PoseStamped> original_path_;
    
    // Neighbor directions (26-connected)
    static const int num_neighbors_ = 26;
    static const int neighbor_dirs_[26][3];
    
    // Helper functions
    double calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const;
    double calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<JPSNode> getJumpPoints(const JPSNode& node, int goal_x, int goal_y, int goal_z) const;
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const std::unordered_map<Coord3D, JPSNode>& came_from,
        int goal_x, int goal_y, int goal_z) const;
    
    // JPS specific functions
    bool jump(int x, int y, int z, int dx, int dy, int dz, 
              int goal_x, int goal_y, int goal_z,
              int& jump_x, int& jump_y, int& jump_z, double& jump_cost) const;
    bool hasForcedNeighbor(int x, int y, int z, int dx, int dy, int dz) const;
    bool isJumpPoint(int x, int y, int z, int dx, int dy, int dz) const;
    bool isValidPosition(int x, int y, int z) const;
    void getStraightLinePath(int x1, int y1, int z1, int x2, int y2, int z2,
                            std::vector<geometry_msgs::msg::PoseStamped>& path) const;
};

#endif // JPS_PLANNER_HPP 