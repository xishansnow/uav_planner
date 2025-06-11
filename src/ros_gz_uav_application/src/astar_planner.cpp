#include "astar_planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

// Neighbor directions (26-connected grid)
const int AStarPlanner::neighbor_dirs_[26][3] = {
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

AStarPlanner::AStarPlanner(EnvironmentVoxel3D* env)
    : GlobalPlannerBase(env), heuristic_weight_(1.0), diagonal_cost_(1.414), vertical_cost_(1.0)
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::planPath(double start_x, double start_y, double start_z,
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
    
    // Initialize data structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_map<int, double> g_scores;
    std::unordered_map<int, AStarNode> came_from;
    std::set<AStarNode> closed_set;
    
    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                       goal_voxel_x, goal_voxel_y, goal_voxel_z);
    AStarNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                         start_voxel_x, start_voxel_y, start_voxel_z);
    
    open_set.push(start_node);
    g_scores[hashNode(start_voxel_x, start_voxel_y, start_voxel_z)] = 0.0;
    
    while (!open_set.empty()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            return false;  // Timeout
        }
        
        AStarNode current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current.x == goal_voxel_x && current.y == goal_voxel_y && current.z == goal_voxel_z) {
            // Reconstruct path
            path = reconstructPath(came_from, goal_voxel_x, goal_voxel_y, goal_voxel_z);
            
            // Smooth the path
            path = smoothPath(path);
            return true;
        }
        
        // Add to closed set
        closed_set.insert(current);
        
        // Get neighbors
        std::vector<AStarNode> neighbors = getNeighbors(current);
        
        for (const auto& neighbor : neighbors) {
            // Skip if already explored
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            int neighbor_hash = hashNode(neighbor.x, neighbor.y, neighbor.z);
            double tentative_g = current.g_cost + calculateCost(current.x, current.y, current.z,
                                                               neighbor.x, neighbor.y, neighbor.z);
            
            // Check if this path is better
            auto g_it = g_scores.find(neighbor_hash);
            if (g_it == g_scores.end() || tentative_g < g_it->second) {
                // This path is better
                came_from[neighbor_hash] = current;
                g_scores[neighbor_hash] = tentative_g;
                
                // Create new neighbor node with updated costs
                double h_cost = calculateHeuristic(neighbor.x, neighbor.y, neighbor.z,
                                                  goal_voxel_x, goal_voxel_y, goal_voxel_z);
                AStarNode new_neighbor(neighbor.x, neighbor.y, neighbor.z, tentative_g, h_cost,
                                      current.x, current.y, current.z);
                
                open_set.push(new_neighbor);
            }
        }
    }
    
    return false;  // No path found
}

bool AStarPlanner::replanPath(const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
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

void AStarPlanner::setParameters(const std::string& param_name, double value)
{
    if (param_name == "heuristic_weight") {
        heuristic_weight_ = value;
    } else if (param_name == "diagonal_cost") {
        diagonal_cost_ = value;
    } else if (param_name == "vertical_cost") {
        vertical_cost_ = value;
    }
}

double AStarPlanner::calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Euclidean distance heuristic
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return heuristic_weight_ * std::sqrt(dx*dx + dy*dy + dz*dz);
}

double AStarPlanner::calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const
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

std::vector<AStarNode> AStarPlanner::getNeighbors(const AStarNode& node) const
{
    std::vector<AStarNode> neighbors;
    
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

std::vector<geometry_msgs::msg::PoseStamped> AStarPlanner::reconstructPath(
    const std::unordered_map<int, AStarNode>& came_from,
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
        auto it = came_from.find(current_hash);
        
        if (it == came_from.end()) {
            break;  // Reached start
        }
        
        const AStarNode& parent = it->second;
        current_x = parent.x;
        current_y = parent.y;
        current_z = parent.z;
    }
    
    // Reverse path to get start to goal order
    std::reverse(path.begin(), path.end());
    return path;
}

int AStarPlanner::hashNode(int x, int y, int z) const
{
    // Simple hash function for 3D coordinates
    return x * 73856093 ^ y * 19349663 ^ z * 83492791;
} 