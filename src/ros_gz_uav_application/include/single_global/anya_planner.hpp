/*
 * ANYA (Anytime A*) Path Planner Implementation
 * Anytime path planning algorithm that provides solutions with bounded suboptimality
 */

#ifndef ANYA_PLANNER_HPP
#define ANYA_PLANNER_HPP

#include "global_planner_base.hpp"
#include "common_types.hpp"
#include <queue>
#include <unordered_map>
#include <set>
#include <vector>

struct ANYANode
{
    int x, y, z;
    double g_cost;  // Cost from start to current node
    double h_cost;  // Heuristic cost from current to goal
    double f_cost;  // Total cost (g + h)
    int parent_x, parent_y, parent_z;
    
    // Default constructor for unordered_map compatibility
    ANYANode() : x(0), y(0), z(0), g_cost(0.0), h_cost(0.0), f_cost(0.0), 
                 parent_x(0), parent_y(0), parent_z(0) {}
    
    ANYANode(int x, int y, int z, double g, double h, int px, int py, int pz)
        : x(x), y(y), z(z), g_cost(g), h_cost(h), f_cost(g + h),
          parent_x(px), parent_y(py), parent_z(pz) {}
    
    // For priority queue comparison
    bool operator>(const ANYANode& other) const {
        return f_cost > other.f_cost;
    }
    
    // For set comparison
    bool operator<(const ANYANode& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

/**
 * \brief ANYA path planning algorithm implementation
 * ANYA is an anytime path planning algorithm that provides solutions
 * with bounded suboptimality and can improve solutions over time
 */
class ANYAPlanner : public GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    ANYAPlanner(EnvironmentVoxel3D* env);
    
    /**
     * \brief Destructor
     */
    ~ANYAPlanner();
    
    /**
     * \brief Plan path using ANYA algorithm
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
    std::string getAlgorithmName() const override { return "ANYA"; }
    
private:
    // Algorithm parameters
    double initial_epsilon_;      // Initial inflation factor
    double final_epsilon_;        // Final inflation factor
    double epsilon_decrease_;     // Rate at which epsilon decreases
    double heuristic_weight_;     // Weight for heuristic function
    double diagonal_cost_;        // Cost for diagonal movement
    double vertical_cost_;        // Cost for vertical movement
    double current_epsilon_;      // Current inflation factor
    double current_solution_cost_; // Cost of current best solution
    bool has_solution_;           // Whether we have a solution
    
    // Neighbor directions (26-connected)
    static const int num_neighbors_ = 26;
    static const int neighbor_dirs_[26][3];
    
    // Helper functions
    double calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const;
    double calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const;
    std::vector<ANYANode> getNeighbors(const ANYANode& node) const;
    std::vector<geometry_msgs::msg::PoseStamped> reconstructPath(
        const std::unordered_map<Coord3D, ANYANode>& came_from,
        int goal_x, int goal_y, int goal_z) const;
    
    // ANYA specific functions
    bool isValidPosition(int x, int y, int z) const;
    void updateEpsilon();
    bool improveSolution();
};

#endif // ANYA_PLANNER_HPP 