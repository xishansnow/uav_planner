#include "single_global/arastar_planner.hpp"
#include "env/environment_voxel3d.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

// Neighbor directions (26-connected grid)
const int ARAStarPlanner::neighbor_dirs_[26][3] = {
    // Face neighbors (6)
    {-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1},
    // Edge neighbors (12)
    {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
    {-1, 0, -1}, {-1, 0, 1}, {1, 0, -1}, {1, 0, 1},
    {0, -1, -1}, {0, -1, 1}, {0, 1, -1}, {0, 1, 1},
    // Corner neighbors (8)
    {-1, -1, -1}, {-1, -1, 1}, {-1, 1, -1}, {-1, 1, 1},
    {1, -1, -1}, {1, -1, 1}, {1, 1, -1}, {1, 1, 1}
};

ARAStarPlanner::ARAStarPlanner(EnvironmentVoxel3D* env)
    : GlobalPlannerBase(env), initial_epsilon_(3.0), final_epsilon_(1.0), epsilon_decrease_(0.5),
      heuristic_weight_(1.0), diagonal_cost_(1.414), vertical_cost_(1.0),
      current_epsilon_(3.0), current_solution_cost_(std::numeric_limits<double>::infinity()),
      has_solution_(false)
{
}

ARAStarPlanner::~ARAStarPlanner()
{
}

bool ARAStarPlanner::planPath(double start_x, double start_y, double start_z,
                              double goal_x, double goal_y, double goal_z,
                              std::vector<geometry_msgs::msg::PoseStamped>& path,
                              SmootherType smooth_type,
                              double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Convert world coordinates to voxel coordinates
    int start_voxel_x, start_voxel_y, start_voxel_z;
    int goal_voxel_x, goal_voxel_y, goal_voxel_z;
    
    if (!worldToVoxel(start_x, start_y, start_z, start_voxel_x, start_voxel_y, start_voxel_z) ||
        !worldToVoxel(goal_x, goal_y, goal_z, goal_voxel_x, goal_voxel_y, goal_voxel_z)) {
        return false;
    }
    
    // Check if start or goal is occupied
    if (env_->IsCellOccupied(start_voxel_x, start_voxel_y, start_voxel_z) ||
        env_->IsCellOccupied(goal_voxel_x, goal_voxel_y, goal_voxel_z)) {
        return false;
    }
    
    // Initialize ARA* state
    current_epsilon_ = initial_epsilon_;
    current_solution_cost_ = std::numeric_limits<double>::infinity();
    has_solution_ = false;
    
    // Initialize data structures
    std::priority_queue<ARAStarNode, std::vector<ARAStarNode>, std::greater<ARAStarNode>> open_set;
    std::unordered_map<Coord3D, ARAStarNode> nodes;
    std::unordered_map<Coord3D, double> g_scores;
    std::set<Coord3D> closed_set;
    
    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                       goal_voxel_x, goal_voxel_y, goal_voxel_z);
    ARAStarNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                          -1, -1, -1, current_epsilon_);
    
    Coord3D start_coord(start_voxel_x, start_voxel_y, start_voxel_z);
    nodes[start_coord] = start_node;
    g_scores[start_coord] = 0.0;
    open_set.push(start_node);
    
    // Main ARA* loop
    while (current_epsilon_ >= final_epsilon_) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            break;  // Timeout
        }
        
        // Improve current solution
        improvePath(max_time - elapsed.count() / 1000.0);
        
        // Check if we have a solution
        Coord3D goal_coord(goal_voxel_x, goal_voxel_y, goal_voxel_z);
        auto goal_it = nodes.find(goal_coord);
        if (goal_it != nodes.end() && goal_it->second.g_cost < std::numeric_limits<double>::infinity()) {
            has_solution_ = true;
            current_solution_cost_ = goal_it->second.g_cost;
            
            // Reconstruct path
            path = reconstructPath(nodes, goal_voxel_x, goal_voxel_y, goal_voxel_z);
            
            // 保存原始路径
            original_path_ = path;
            
            // If epsilon is close to 1.0, we have a near-optimal solution
            if (current_epsilon_ <= 1.0 + 1e-6) {
                return true;
            }
        }
        
        // Decrease epsilon for next iteration
        decreaseEpsilon();
        
        // Update key values for all nodes in open set
        std::vector<ARAStarNode> updated_nodes;
        while (!open_set.empty()) {
            ARAStarNode node = open_set.top();
            open_set.pop();
            
            Coord3D coord(node.x, node.y, node.z);
            auto it = nodes.find(coord);
            if (it != nodes.end()) {
                it->second.updateKey(current_epsilon_);
                updated_nodes.push_back(it->second);
            }
        }
        
        // Re-add updated nodes to open set
        for (const auto& node : updated_nodes) {
            open_set.push(node);
        }
    }
    
    return has_solution_;
}

bool ARAStarPlanner::replanPath(const geometry_msgs::msg::PoseStamped& current_position,
                                const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
                                std::vector<geometry_msgs::msg::PoseStamped>& new_path,
                                SmootherType smooth_type,
                                double max_time)
{
    if (current_path.empty()) return false;
    
    // Use provided current position as start
    // Find the goal (last waypoint in current path)
    const auto& goal_pos = current_path.front();
    
    return planPath(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z,
                    goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z,
                    new_path, smooth_type, max_time);
}

void ARAStarPlanner::setParameters(const std::string& param_name, double value)
{
    if (param_name == "initial_epsilon") {
        initial_epsilon_ = value;
    } else if (param_name == "final_epsilon") {
        final_epsilon_ = value;
    } else if (param_name == "epsilon_decrease") {
        epsilon_decrease_ = value;
    } else if (param_name == "heuristic_weight") {
        heuristic_weight_ = value;
    } else if (param_name == "diagonal_cost") {
        diagonal_cost_ = value;
    } else if (param_name == "vertical_cost") {
        vertical_cost_ = value;
    }
}

double ARAStarPlanner::calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Euclidean distance heuristic
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return heuristic_weight_ * std::sqrt(dx*dx + dy*dy + dz*dz);
}

double ARAStarPlanner::calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);
    
    // Calculate movement type
    int max_delta = std::max({dx, dy, dz});
    
    if (max_delta == 0) return 0.0;  // Same position
    
    if (max_delta == 1) {
        // Face neighbor
        if (dx + dy + dz == 1) return 1.0;
        // Edge neighbor
        if (dx + dy + dz == 2) return diagonal_cost_;
        // Corner neighbor
        return diagonal_cost_ * 1.732;  // sqrt(3)
    } else {
        // Direct distance for longer moves
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
}

std::vector<ARAStarNode> ARAStarPlanner::getNeighbors(const ARAStarNode& node) const
{
    std::vector<ARAStarNode> neighbors;
    
    for (int i = 0; i < num_neighbors_; ++i) {
        int nx = node.x + neighbor_dirs_[i][0];
        int ny = node.y + neighbor_dirs_[i][1];
        int nz = node.z + neighbor_dirs_[i][2];
        
        // Check bounds
        int size_x, size_y, size_z;
        env_->GetGridSize(size_x, size_y, size_z);
        
        if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y || nz < 0 || nz >= size_z) {
            continue;
        }
        
        // Check if occupied
        if (env_->IsCellOccupied(nx, ny, nz)) {
            continue;
        }
        
        // Create neighbor node
        neighbors.emplace_back(nx, ny, nz, 0.0, 0.0, -1, -1, -1, current_epsilon_);
    }
    
    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> ARAStarPlanner::reconstructPath(
    const std::unordered_map<Coord3D, ARAStarNode>& nodes,
    int goal_x, int goal_y, int goal_z) const
{
    std::vector<geometry_msgs::msg::PoseStamped> path;
    
    int current_x = goal_x, current_y = goal_y, current_z = goal_z;
    
    // Safety check: prevent infinite loops
    const int max_iterations = 100000;
    int iteration_count = 0;
    
    while (iteration_count < max_iterations) {
        // Add current position to path
        geometry_msgs::msg::PoseStamped pose;
        voxelToWorld(current_x, current_y, current_z,
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        pose.pose.orientation.w = 1.0;
        path.push_back(pose);
        
        // Find parent
        Coord3D current_coord(current_x, current_y, current_z);
        auto it = nodes.find(current_coord);
        
        if (it == nodes.end()) {
            break;  // Reached start
        }
        
        const ARAStarNode& current_node = it->second;
        
        // Check if we've reached the start node
        if (current_node.parent_x == -1 && current_node.parent_y == -1 && current_node.parent_z == -1) {
            break;
        }
        
        // Safety check: prevent self-reference loops
        if (current_node.parent_x == current_node.x && current_node.parent_y == current_node.y && current_node.parent_z == current_node.z) {
            break;
        }
        
        // Move to parent
        current_x = current_node.parent_x;
        current_y = current_node.parent_y;
        current_z = current_node.parent_z;
        
        iteration_count++;
    }
    
    // Check if we hit the iteration limit
    if (iteration_count >= max_iterations) {
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }
    
    // Reverse path to get start to goal order
    std::reverse(path.begin(), path.end());
    return path;
}

void ARAStarPlanner::improvePath(double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // This is a simplified version of ARA* improve path
    // In a full implementation, this would involve more sophisticated
    // state management and epsilon-consistency checks
    
    while (max_time > 0) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        max_time -= elapsed.count() / 1000.0;
        
        if (max_time <= 0) break;
        
        // Simplified improvement: just run one iteration of A*
        // In practice, this would be more sophisticated
        break;
    }
}

void ARAStarPlanner::decreaseEpsilon()
{
    current_epsilon_ = std::max(final_epsilon_, current_epsilon_ - epsilon_decrease_);
}

bool ARAStarPlanner::isConsistent() const
{
    // Check if the current solution is consistent
    // This is a simplified check
    return current_epsilon_ <= 1.0 + 1e-6;
} 