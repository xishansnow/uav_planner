#include "single_global/jps_planner.hpp"
#include "env/environment_voxel3d.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

// Neighbor directions (26-connected grid)
const int JPSPlanner::neighbor_dirs_[26][3] = {
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

JPSPlanner::JPSPlanner(EnvironmentVoxel3D* env)
    : GlobalPlannerBase(env), heuristic_weight_(1.0), diagonal_cost_(1.414), 
      vertical_cost_(1.0), max_jump_distance_(100)
{
}

JPSPlanner::~JPSPlanner()
{
}

bool JPSPlanner::planPath(double start_x, double start_y, double start_z,
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
    
    // Initialize data structures
    std::priority_queue<JPSNode, std::vector<JPSNode>, std::greater<JPSNode>> open_set;
    std::unordered_map<Coord3D, double> g_scores;
    std::unordered_map<Coord3D, JPSNode> came_from;
    std::set<JPSNode> closed_set;
    
    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                       goal_voxel_x, goal_voxel_y, goal_voxel_z);
    JPSNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                       -1, -1, -1);  // Use invalid coordinates (-1, -1, -1) for start node parent
    
    open_set.push(start_node);
    g_scores[Coord3D(start_voxel_x, start_voxel_y, start_voxel_z)] = 0.0;
    came_from[Coord3D(start_voxel_x, start_voxel_y, start_voxel_z)] = start_node;
    
    while (!open_set.empty()) {
        // Check time limit
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        if (elapsed.count() > max_time * 1000) {
            return false;  // Timeout
        }
        
        JPSNode current = open_set.top();
        open_set.pop();
        
        // Check if we reached the goal
        if (current.x == goal_voxel_x && current.y == goal_voxel_y && current.z == goal_voxel_z) {
            // Reconstruct path
            path = reconstructPath(came_from, goal_voxel_x, goal_voxel_y, goal_voxel_z);
            
            // 保存原始路径
            original_path_ = path;
            
            return true;
        }
        
        // Add to closed set
        closed_set.insert(current);
        
        // Get jump points from current node
        std::vector<JPSNode> jump_points = getJumpPoints(current, goal_voxel_x, goal_voxel_y, goal_voxel_z);
        
        for (auto& jump_point : jump_points) {
            // Skip if already explored
            if (closed_set.find(jump_point) != closed_set.end()) {
                continue;
            }
            
            // Calculate heuristic for jump point if not already set
            if (jump_point.h_cost == 0.0) {
                jump_point.h_cost = calculateHeuristic(jump_point.x, jump_point.y, jump_point.z,
                                                     goal_voxel_x, goal_voxel_y, goal_voxel_z);
            }
            
            Coord3D jump_coord(jump_point.x, jump_point.y, jump_point.z);
            double tentative_g = current.g_cost + calculateCost(current.x, current.y, current.z,
                                                               jump_point.x, jump_point.y, jump_point.z);
            
            // Check if this path is better
            auto g_it = g_scores.find(jump_coord);
            if (g_it == g_scores.end() || tentative_g < g_it->second) {
                // This path is better
                jump_point.g_cost = tentative_g;
                jump_point.parent_x = current.x;
                jump_point.parent_y = current.y;
                jump_point.parent_z = current.z;
                jump_point.f_cost = tentative_g + jump_point.h_cost;
                
                came_from[jump_coord] = jump_point;
                g_scores[jump_coord] = tentative_g;
                
                open_set.push(jump_point);
            }
        }
    }
    
    return false;  // No path found
}

bool JPSPlanner::replanPath(const geometry_msgs::msg::PoseStamped& current_position,
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

void JPSPlanner::setParameters(const std::string& param_name, double value)
{
    if (param_name == "heuristic_weight") {
        heuristic_weight_ = value;
    } else if (param_name == "diagonal_cost") {
        diagonal_cost_ = value;
    } else if (param_name == "vertical_cost") {
        vertical_cost_ = value;
    } else if (param_name == "max_jump_distance") {
        max_jump_distance_ = static_cast<int>(value);
    }
}

double JPSPlanner::calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Euclidean distance heuristic
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;
    return heuristic_weight_ * std::sqrt(dx*dx + dy*dy + dz*dz);
}

double JPSPlanner::calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const
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

std::vector<JPSNode> JPSPlanner::getJumpPoints(const JPSNode& node, int goal_x, int goal_y, int goal_z) const
{
    std::vector<JPSNode> jump_points;
    
    // For each neighbor direction, try to jump
    for (int i = 0; i < num_neighbors_; ++i) {
        int dx = neighbor_dirs_[i][0];
        int dy = neighbor_dirs_[i][1];
        int dz = neighbor_dirs_[i][2];
        
        int jump_x, jump_y, jump_z;
        double jump_cost;
        
        if (jump(node.x, node.y, node.z, dx, dy, dz, goal_x, goal_y, goal_z,
                 jump_x, jump_y, jump_z, jump_cost)) {
            // Create jump point node
            JPSNode jump_node(jump_x, jump_y, jump_z, 0.0, 0.0, -1, -1, -1);
            jump_node.jump_direction[0] = dx;
            jump_node.jump_direction[1] = dy;
            jump_node.jump_direction[2] = dz;
            jump_points.push_back(jump_node);
        }
    }
    
    return jump_points;
}

bool JPSPlanner::jump(int x, int y, int z, int dx, int dy, int dz, 
                      int goal_x, int goal_y, int goal_z,
                      int& jump_x, int& jump_y, int& jump_z, double& jump_cost) const
{
    // Check if we can move in the given direction
    int next_x = x + dx;
    int next_y = y + dy;
    int next_z = z + dz;
    
    if (!isValidPosition(next_x, next_y, next_z)) {
        return false;
    }
    
    // Check if we reached the goal
    if (next_x == goal_x && next_y == goal_y && next_z == goal_z) {
        jump_x = next_x;
        jump_y = next_y;
        jump_z = next_z;
        jump_cost = calculateCost(x, y, z, next_x, next_y, next_z);
        return true;
    }
    
    // Check if this is a jump point
    if (isJumpPoint(next_x, next_y, next_z, dx, dy, dz)) {
        jump_x = next_x;
        jump_y = next_y;
        jump_z = next_z;
        jump_cost = calculateCost(x, y, z, next_x, next_y, next_z);
        return true;
    }
    
    // Recursively jump in the same direction
    double recursive_cost;
    if (jump(next_x, next_y, next_z, dx, dy, dz, goal_x, goal_y, goal_z,
             jump_x, jump_y, jump_z, recursive_cost)) {
        jump_cost = calculateCost(x, y, z, next_x, next_y, next_z) + recursive_cost;
        return true;
    }
    
    return false;
}

bool JPSPlanner::hasForcedNeighbor(int x, int y, int z, int dx, int dy, int dz) const
{
    // Check if there are forced neighbors in the given direction
    // This is a simplified implementation
    // In a full JPS implementation, this would check for specific patterns
    
    // For now, we'll use a simple heuristic: if we're moving diagonally
    // and there's an obstacle in one of the cardinal directions,
    // then the other cardinal direction becomes a forced neighbor
    
    if (dx != 0 && dy != 0) {  // Diagonal movement in XY plane
        int check_x = x + dx;
        int check_y = y + dy;
        
        // Check if there's an obstacle in one of the cardinal directions
        bool obstacle_x = !isValidPosition(check_x, y, z);
        bool obstacle_y = !isValidPosition(x, check_y, z);
        
        // If there's an obstacle in one direction but not the other,
        // the unobstructed direction becomes a forced neighbor
        if (obstacle_x != obstacle_y) {
            return true;
        }
    }
    
    return false;
}

bool JPSPlanner::isJumpPoint(int x, int y, int z, int dx, int dy, int dz) const
{
    // Check if this position is a jump point
    // A jump point occurs when:
    // 1. It's the goal
    // 2. It has forced neighbors
    // 3. It's a natural jump point (end of a straight line)
    
    // Check for forced neighbors
    if (hasForcedNeighbor(x, y, z, dx, dy, dz)) {
        return true;
    }
    
    // Check if we can't continue in the current direction
    int next_x = x + dx;
    int next_y = y + dy;
    int next_z = z + dz;
    
    if (!isValidPosition(next_x, next_y, next_z)) {
        return true;  // Natural jump point at obstacle
    }
    
    return false;
}

bool JPSPlanner::isValidPosition(int x, int y, int z) const
{
    // Check bounds
    int size_x, size_y, size_z;
    env_->GetGridSize(size_x, size_y, size_z);
    
    if (x < 0 || x >= size_x || y < 0 || y >= size_y || z < 0 || z >= size_z) {
        return false;
    }
    
    // Check if occupied
    if (env_->IsCellOccupied(x, y, z)) {
        return false;
    }
    
    return true;
}

std::vector<geometry_msgs::msg::PoseStamped> JPSPlanner::reconstructPath(
    const std::unordered_map<Coord3D, JPSNode>& came_from,
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
        auto it = came_from.find(current_coord);
        
        if (it == came_from.end()) {
            break;  // Reached start
        }
        
        const JPSNode& current_node = it->second;
        
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

void JPSPlanner::getStraightLinePath(int x1, int y1, int z1, int x2, int y2, int z2,
                                     std::vector<geometry_msgs::msg::PoseStamped>& path) const
{
    // Use Bresenham's line algorithm to generate straight line path
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);
    
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int sz = (z1 < z2) ? 1 : -1;
    
    int x = x1, y = y1, z = z1;
    
    // Add start point
    geometry_msgs::msg::PoseStamped pose;
    voxelToWorld(x, y, z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
    
    // Generate intermediate points
    if (dx >= dy && dx >= dz) {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;
        
        for (int i = 0; i < dx; i++) {
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
            
            // Add point to path
            voxelToWorld(x, y, z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.push_back(pose);
        }
    } else if (dy >= dx && dy >= dz) {
        int err_1 = 2 * dx - dy;
        int err_2 = 2 * dz - dy;
        
        for (int i = 0; i < dy; i++) {
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
            
            // Add point to path
            voxelToWorld(x, y, z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.push_back(pose);
        }
    } else {
        int err_1 = 2 * dy - dz;
        int err_2 = 2 * dx - dz;
        
        for (int i = 0; i < dz; i++) {
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
            
            // Add point to path
            voxelToWorld(x, y, z, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            path.push_back(pose);
        }
    }
} 