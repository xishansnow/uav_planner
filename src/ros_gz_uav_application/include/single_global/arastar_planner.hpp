/*
 * ARA* (Anytime Repairing A*) Path Planner Implementation
 * Anytime path planning algorithm that provides solutions with bounded suboptimality
 */

#ifndef ARASTAR_PLANNER_HPP
#define ARASTAR_PLANNER_HPP

#include "single_global/global_planner_base.hpp"
#include "common_types.hpp"
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>

struct ARAStarNode
{
    int x, y, z;
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current to goal
    double f_cost;  // Total cost (g + h)
    double key_value;  // Key value for priority queue (f + epsilon * h)
    int parent_x, parent_y, parent_z;
    bool in_open_set;   // Whether node is in open set
    bool in_closed_set; // Whether node is in closed set
    double epsilon;     // Current inflation factor
    
    // Default constructor for unordered_map compatibility
    ARAStarNode() : x(0), y(0), z(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), 
                    key_value(0.0), parent_x(0), parent_y(0), parent_z(0), 
                    in_open_set(false), in_closed_set(false), epsilon(1.0) {}
    
    ARAStarNode(int x, int y, int z, double g, double h, int px, int py, int pz, double eps = 1.0)
        : x(x), y(y), z(z), g_cost(g), h_cost(h), f_cost(g + h), key_value(g + eps * h),
          parent_x(px), parent_y(py), parent_z(pz), in_open_set(false), in_closed_set(false), epsilon(eps) {}
    
    // For priority queue comparison
    bool operator>(const ARAStarNode& other) const {
        return key_value > other.key_value;
    }
    
    // For set comparison
    bool operator<(const ARAStarNode& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
    
    // Update key value with new epsilon
    void updateKey(double new_epsilon) {
        epsilon = new_epsilon;
        key_value = g_cost + epsilon * h_cost;
    }
};

/**
 * \brief ARA* path planning algorithm implementation
 * ARA* is an anytime algorithm that provides suboptimal solutions quickly
 * and improves them over time by decreasing the inflation factor
 */
class ARAStarPlanner : public GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    ARAStarPlanner(EnvironmentVoxel3D* env);
    
    /**
     * \brief Destructor
     */
    ~ARAStarPlanner();
    
    /**
     * \brief Plan path using ARA* algorithm
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
    std::string getAlgorithmName() const override { return "ARA*"; }
    
    /**
     * \brief Get current solution cost
     */
    double getCurrentSolutionCost() const { return current_solution_cost_; }
    
    /**
     * \brief Get current epsilon value
     */
    double getCurrentEpsilon() const { return current_epsilon_; }
    
    /**
     * \brief Get original path before smoothing
     */
    std::vector<geometry_msgs::msg::PoseStamped> getOriginalPath() const override { return original_path_; }
    
private:
    // Algorithm parameters
    double initial_epsilon_;      // Initial inflation factor
    double final_epsilon_;        // Final inflation factor
    double epsilon_decrease_;     // How much to decrease epsilon each iteration
    double heuristic_weight_;     // Heuristic weight
    double diagonal_cost_;        // Cost for diagonal moves
    double vertical_cost_;        // Cost for vertical moves
    
    // Current state
    double current_epsilon_;      // Current inflation factor
    double current_solution_cost_; // Cost of current solution
    bool has_solution_;
    
    // Original path before smoothing
    std::vector<geometry_msgs::msg::PoseStamped> original_path_;
    
    // Neighbor directions (26-connected)
    static const int num_neighbors_ = 26;
    static const int neighbor_dirs_[26][3];
    
    // Helper functions
    double calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const;
    double calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<ARAStarNode> getNeighbors(const ARAStarNode& node) const;
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const std::unordered_map<Coord3D, ARAStarNode>& came_from,
        int goal_x, int goal_y, int goal_z) const;
    
    // ARA* specific functions
    void improvePath(double max_time);
    void updateVertex(ARAStarNode& node, const ARAStarNode& parent,
                     std::priority_queue<ARAStarNode, std::vector<ARAStarNode>, std::greater<ARAStarNode>>& open_set,
                     std::unordered_map<Coord3D, ARAStarNode>& nodes,
                     std::unordered_map<Coord3D, double>& g_scores) const;
    void computeShortestPath(double max_time);
    bool isConsistent() const;
    void decreaseEpsilon();
};

#endif // ARASTAR_PLANNER_HPP 