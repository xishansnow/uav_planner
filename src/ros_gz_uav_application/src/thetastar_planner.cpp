#include "thetastar_planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

// Neighbor directions (26-connected grid)
const int ThetaStarPlanner::neighbor_dirs_[26][3] = {
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

ThetaStarPlanner::ThetaStarPlanner(EnvironmentVoxel3D* env)
    : GlobalPlannerBase(env), heuristic_weight_(1.0), diagonal_cost_(1.414), 
      vertical_cost_(1.0), use_lazy_thetastar_(false)
{
}

ThetaStarPlanner::~ThetaStarPlanner()
{
}

bool ThetaStarPlanner::planPath(double start_x, double start_y, double start_z,
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
    std::priority_queue<ThetaStarNode, std::vector<ThetaStarNode>, std::greater<ThetaStarNode>> open_set;
    std::unordered_map<int, double> g_scores;
    std::unordered_map<int, ThetaStarNode> came_from;
    std::set<ThetaStarNode> closed_set;
    
    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                       goal_voxel_x, goal_voxel_y, goal_voxel_z);
    ThetaStarNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                            start_voxel_x, start_voxel_y, start_voxel_z, true);
    
    open_set.push(start_node);
    g_scores[hashNode(start_voxel_x, start_voxel_y, start_voxel_z)] = 0.0;
    came_from[hashNode(start_voxel_x, start_voxel_y, start_voxel_z)] = start_node;
    
    while (!open_set.empty()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            return false;  // Timeout
        }
        
        ThetaStarNode current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current.x == goal_voxel_x && current.y == goal_voxel_y && current.z == goal_voxel_z) {
            // Reconstruct path
            path = reconstructPath(came_from, goal_voxel_x, goal_voxel_y, goal_voxel_z);
            return true;
        }
        
        // Add to closed set
        closed_set.insert(current);
        
        // Get parent node
        int current_hash = hashNode(current.x, current.y, current.z);
        auto parent_it = came_from.find(current_hash);
        const ThetaStarNode* parent = nullptr;
        if (parent_it != came_from.end()) {
            parent = &parent_it->second;
        }
        
        // Get neighbors
        std::vector<ThetaStarNode> neighbors = getNeighbors(current);
        
        for (auto& neighbor : neighbors) {
            // Skip if already explored
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            // Theta* update: try to connect to parent's parent if possible
            if (parent && parent->x != current.x && parent->y != current.y && parent->z != current.z) {
                // Check if we can connect to parent's parent
                int grandparent_hash = hashNode(parent->parent_x, parent->parent_y, parent->parent_z);
                auto grandparent_it = came_from.find(grandparent_hash);
                
                if (grandparent_it != came_from.end()) {
                    const ThetaStarNode& grandparent = grandparent_it->second;
                    
                    // Check line of sight to grandparent
                    if (hasLineOfSight(grandparent.x, grandparent.y, grandparent.z,
                                      neighbor.x, neighbor.y, neighbor.z)) {
                        // Calculate cost through grandparent
                        double cost_through_grandparent = grandparent.g_cost + 
                            calculateDirectCost(grandparent.x, grandparent.y, grandparent.z,
                                              neighbor.x, neighbor.y, neighbor.z);
                        
                        int neighbor_hash = hashNode(neighbor.x, neighbor.y, neighbor.z);
                        auto g_it = g_scores.find(neighbor_hash);
                        
                        if (g_it == g_scores.end() || cost_through_grandparent < g_it->second) {
                            // Update through grandparent
                            neighbor.g_cost = cost_through_grandparent;
                            neighbor.parent_x = grandparent.x;
                            neighbor.parent_y = grandparent.y;
                            neighbor.parent_z = grandparent.z;
                            neighbor.has_line_of_sight = true;
                            neighbor.f_cost = cost_through_grandparent + neighbor.h_cost;
                            
                            g_scores[neighbor_hash] = cost_through_grandparent;
                            came_from[neighbor_hash] = neighbor;
                            
                            open_set.push(neighbor);
                            continue;
                        }
                    }
                }
            }
            
            // Regular A* update
            int neighbor_hash = hashNode(neighbor.x, neighbor.y, neighbor.z);
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
                neighbor.has_line_of_sight = false;
                neighbor.f_cost = tentative_g + neighbor.h_cost;
                
                came_from[neighbor_hash] = neighbor;
                g_scores[neighbor_hash] = tentative_g;
                
                open_set.push(neighbor);
            }
        }
    }
    
    return false;  // No path found
}

bool ThetaStarPlanner::replanPath(const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
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

void ThetaStarPlanner::setParameters(const std::string& param_name, double value)
{
    if (param_name == "heuristic_weight") {
        heuristic_weight_ = value;
    } else if (param_name == "diagonal_cost") {
        diagonal_cost_ = value;
    } else if (param_name == "vertical_cost") {
        vertical_cost_ = value;
    } else if (param_name == "use_lazy_thetastar") {
        use_lazy_thetastar_ = (value != 0.0);
    }
}

double ThetaStarPlanner::calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Euclidean distance heuristic
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return heuristic_weight_ * std::sqrt(dx*dx + dy*dy + dz*dz);
}

double ThetaStarPlanner::calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const
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

double ThetaStarPlanner::calculateDirectCost(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Direct Euclidean distance for line-of-sight connections
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

std::vector<ThetaStarNode> ThetaStarPlanner::getNeighbors(const ThetaStarNode& node) const
{
    std::vector<ThetaStarNode> neighbors;
    
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
        neighbors.emplace_back(nx, ny, nz, 0.0, 0.0, node.x, node.y, node.z, false);
    }
    
    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> ThetaStarPlanner::reconstructPath(
    const std::unordered_map<int, ThetaStarNode>& came_from,
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
        
        const ThetaStarNode& parent = it->second;
        current_x = parent.x;
        current_y = parent.y;
        current_z = parent.z;
    }
    
    // Reverse path to get start to goal order
    std::reverse(path.begin(), path.end());
    return path;
}

bool ThetaStarPlanner::hasLineOfSight(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Use Bresenham's line algorithm to check line of sight
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);
    
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int sz = (z1 < z2) ? 1 : -1;
    
    int x = x1, y = y1, z = z1;
    
    // Check if start or goal is occupied
    if (env_->isOccupied(x, y, z) || env_->isOccupied(x2, y2, z2)) {
        return false;
    }
    
    // Check intermediate points
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        
        for (int i = 0; i < dx; i++) {
            if (env_->isOccupied(x, y, z)) return false;
            
            if (err_1 > 0) {
                y += sy;
                err_1 -= 2 * dx;
            }
            if (err_2 > 0) {
                z += sz;
                err_2 -= 2 * dx;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dz;
            x += sx;
        }
    } else if (dy >= dx && dy >= dz) {
        int err_1 = 2 * dx - dy;
        int err_2 = 2 * dz - dy;
        
        for (int i = 0; i < dy; i++) {
            if (env_->isOccupied(x, y, z)) return false;
            
            if (err_1 > 0) {
                x += sx;
                err_1 -= 2 * dy;
            }
            if (err_2 > 0) {
                z += sz;
                err_2 -= 2 * dy;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dz;
            y += sy;
        }
    } else {
        int err_1 = 2 * dy - dz;
        int err_2 = 2 * dx - dz;
        
        for (int i = 0; i < dz; i++) {
            if (env_->isOccupied(x, y, z)) return false;
            
            if (err_1 > 0) {
                y += sy;
                err_1 -= 2 * dz;
            }
            if (err_2 > 0) {
                x += sx;
                err_2 -= 2 * dz;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dx;
            z += sz;
        }
    }
    
    return true;
}

int ThetaStarPlanner::hashNode(int x, int y, int z) const
{
    // Simple hash function for 3D coordinates
    return x * 73856093 ^ y * 19349663 ^ z * 83492791;
} 