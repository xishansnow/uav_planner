#include "arastar_planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

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
    : GlobalPlannerBase(env), initial_epsilon_(3.0), final_epsilon_(1.0), 
      epsilon_decrease_(0.5), heuristic_weight_(1.0), diagonal_cost_(1.414), 
      vertical_cost_(1.0), current_epsilon_(3.0), current_solution_cost_(std::numeric_limits<double>::infinity()),
      has_solution_(false)
{
}

ARAStarPlanner::~ARAStarPlanner()
{
}

bool ARAStarPlanner::planPath(double start_x, double start_y, double start_z,
                              double goal_x, double goal_y, double goal_z,
                              std::vector<geometry_msgs::msg::PoseStamped>& path,
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
    if (env_->isOccupied(start_voxel_x, start_voxel_y, start_voxel_z) ||
        env_->isOccupied(goal_voxel_x, goal_voxel_y, goal_voxel_z)) {
        return false;
    }
    
    // Initialize ARA* state
    current_epsilon_ = initial_epsilon_;
    current_solution_cost_ = std::numeric_limits<double>::infinity();
    has_solution_ = false;
    
    // Initialize data structures
    std::priority_queue<ARAStarNode, std::vector<ARAStarNode>, std::greater<ARAStarNode>> open_set;
    std::unordered_map<int, ARAStarNode> nodes;
    std::unordered_map<int, double> g_scores;
    
    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                       goal_voxel_x, goal_voxel_y, goal_voxel_z);
    ARAStarNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                           start_voxel_x, start_voxel_y, start_voxel_z);
    start_node.key_value = start_node.g_cost + current_epsilon_ * start_node.h_cost;
    start_node.in_open_set = true;
    
    int start_hash = hashNode(start_voxel_x, start_voxel_y, start_voxel_z);
    nodes[start_hash] = start_node;
    g_scores[start_hash] = 0.0;
    open_set.push(start_node);
    
    // Main ARA* loop
    while (current_epsilon_ >= final_epsilon_) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            break;  // Timeout
        }
        
        // Compute shortest path with current epsilon
        computeShortestPath(max_time - elapsed.count() / 1000.0);
        
        // Check if we found a solution
        int goal_hash = hashNode(goal_voxel_x, goal_voxel_y, goal_voxel_z);
        auto goal_it = nodes.find(goal_hash);
        
        if (goal_it != nodes.end() && goal_it->second.g_cost < std::numeric_limits<double>::infinity()) {
            has_solution_ = true;
            current_solution_cost_ = goal_it->second.g_cost;
            
            // Reconstruct path
            path = reconstructPath(nodes, goal_voxel_x, goal_voxel_y, goal_voxel_z);
            
            // If we have a solution and epsilon is close to 1, we can stop
            if (current_epsilon_ <= 1.1) {
                break;
            }
        }
        
        // Decrease epsilon for next iteration
        current_epsilon_ = std::max(final_epsilon_, current_epsilon_ - epsilon_decrease_);
        
        // Update key values for all nodes in open set
        std::vector<ARAStarNode> temp_nodes;
        while (!open_set.empty()) {
            ARAStarNode node = open_set.top();
            open_set.pop();
            
            int node_hash = hashNode(node.x, node.y, node.z);
            auto it = nodes.find(node_hash);
            if (it != nodes.end()) {
                it->second.key_value = it->second.g_cost + current_epsilon_ * it->second.h_cost;
                temp_nodes.push_back(it->second);
            }
        }
        
        // Re-add nodes to open set with updated key values
        for (const auto& node : temp_nodes) {
            open_set.push(node);
        }
    }
    
    if (has_solution_) {
        // Smooth the path
        path = smoothPath(path);
        return true;
    }
    
    return false;  // No path found
}

bool ARAStarPlanner::replanPath(const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
                                std::vector<geometry_msgs::msg::PoseStamped>& new_path,
                                double max_time)
{
    if (current_path.empty()) return false;
    
    // Use current position as start
    const auto& current_pos = current_path.back();
    
    // Find the goal (last waypoint in current path)
    const auto& goal_pos = current_path.front();
    
    return planPath(current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z,
                    goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z,
                    new_path, max_time);
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
        env_->getGridSize(size_x, size_y, size_z);
        
        if (nx < 0 || nx >= size_x || ny < 0 || ny >= size_y || nz < 0 || nz >= size_z) {
            continue;
        }
        
        // Check if occupied
        if (env_->isOccupied(nx, ny, nz)) {
            continue;
        }
        
        // Create neighbor node
        neighbors.emplace_back(nx, ny, nz, 0.0, 0.0, node.x, node.y, node.z);
    }
    
    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> ARAStarPlanner::reconstructPath(
    const std::unordered_map<int, ARAStarNode>& nodes,
    int goal_x, int goal_y, int goal_z) const
{
    std::vector<geometry_msgs::msg::PoseStamped> path;
    
    int current_x = goal_x, current_y = goal_y, current_z = goal_z;
    
    while (true) {
        // Add current position to path
        geometry_msgs::msg::PoseStamped pose;
        voxelToWorld(current_x, current_y, current_z,
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        pose.pose.orientation.w = 1.0;  // Default orientation
        path.push_back(pose);
        
        // Find parent
        int current_hash = hashNode(current_x, current_y, current_z);
        auto it = nodes.find(current_hash);
        
        if (it == nodes.end()) {
            break;  // Reached start
        }
        
        const ARAStarNode& node = it->second;
        current_x = node.parent_x;
        current_y = node.parent_y;
        current_z = node.parent_z;
    }
    
    // Reverse path to get start to goal order
    std::reverse(path.begin(), path.end());
    return path;
}

void ARAStarPlanner::computeShortestPath(double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // This is a simplified version - in practice, you would need to implement
    // the full ARA* algorithm with proper state management
    
    // For now, we'll use a basic A* with inflated heuristic
    std::priority_queue<ARAStarNode, std::vector<ARAStarNode>, std::greater<ARAStarNode>> open_set;
    std::unordered_map<int, ARAStarNode> nodes;
    std::unordered_map<int, double> g_scores;
    
    // Find start node
    for (const auto& pair : nodes) {
        if (!pair.second.in_closed_set) {
            open_set.push(pair.second);
        }
    }
    
    while (!open_set.empty()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            break;  // Timeout
        }
        
        ARAStarNode current = open_set.top();
        open_set.pop();
        
        if (current.in_closed_set) {
            continue;
        }
        
        current.in_closed_set = true;
        current.in_open_set = false;
        
        int current_hash = hashNode(current.x, current.y, current.z);
        nodes[current_hash] = current;
        
        // Get neighbors
        std::vector<ARAStarNode> neighbors = getNeighbors(current);
        
        for (auto& neighbor : neighbors) {
            int neighbor_hash = hashNode(neighbor.x, neighbor.y, neighbor.z);
            
            // Calculate tentative g cost
            double tentative_g = current.g_cost + calculateCost(current.x, current.y, current.z,
                                                               neighbor.x, neighbor.y, neighbor.z);
            
            // Check if this path is better
            auto g_it = g_scores.find(neighbor_hash);
            if (g_it == g_scores.end() || tentative_g < g_it->second) {
                // This path is better
                neighbor.g_cost = tentative_g;
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;
                neighbor.parent_z = current.z;
                neighbor.f_cost = tentative_g + neighbor.h_cost;
                neighbor.key_value = tentative_g + current_epsilon_ * neighbor.h_cost;
                neighbor.in_open_set = true;
                neighbor.in_closed_set = false;
                
                g_scores[neighbor_hash] = tentative_g;
                nodes[neighbor_hash] = neighbor;
                open_set.push(neighbor);
            }
        }
    }
}

int ARAStarPlanner::hashNode(int x, int y, int z) const
{
    // Simple hash function for 3D coordinates
    return x * 73856093 ^ y * 19349663 ^ z * 83492791;
} 