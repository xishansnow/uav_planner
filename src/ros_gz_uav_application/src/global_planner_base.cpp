#include "global_planner_base.hpp"
#include <cmath>
#include <algorithm>

GlobalPlannerBase::GlobalPlannerBase(EnvironmentVoxel3D* env)
    : env_(env)
{
}

GlobalPlannerBase::~GlobalPlannerBase()
{
}

bool GlobalPlannerBase::worldToVoxel(double world_x, double world_y, double world_z, 
                                     int& voxel_x, int& voxel_y, int& voxel_z) const
{
    if (!env_) return false;
    
    // Get environment bounds
    double min_x, min_y, min_z, max_x, max_y, max_z;
    env_->getBounds(min_x, min_y, min_z, max_x, max_y, max_z);
    
    // Get resolution
    double resolution = env_->getResolution();
    
    // Convert to voxel coordinates
    voxel_x = static_cast<int>((world_x - min_x) / resolution);
    voxel_y = static_cast<int>((world_y - min_y) / resolution);
    voxel_z = static_cast<int>((world_z - min_z) / resolution);
    
    // Check bounds
    int size_x, size_y, size_z;
    env_->getGridSize(size_x, size_y, size_z);
    
    return (voxel_x >= 0 && voxel_x < size_x &&
            voxel_y >= 0 && voxel_y < size_y &&
            voxel_z >= 0 && voxel_z < size_z);
}

void GlobalPlannerBase::voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                                     double& world_x, double& world_y, double& world_z) const
{
    if (!env_) return;
    
    // Get environment bounds
    double min_x, min_y, min_z, max_x, max_y, max_z;
    env_->getBounds(min_x, min_y, min_z, max_x, max_y, max_z);
    
    // Get resolution
    double resolution = env_->getResolution();
    
    // Convert to world coordinates
    world_x = min_x + (voxel_x + 0.5) * resolution;
    world_y = min_y + (voxel_y + 0.5) * resolution;
    world_z = min_z + (voxel_z + 0.5) * resolution;
}

std::vector<geometry_msgs::msg::PoseStamped> GlobalPlannerBase::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path)
{
    if (raw_path.size() < 3) return raw_path;
    
    std::vector<geometry_msgs::msg::PoseStamped> smoothed_path;
    smoothed_path.push_back(raw_path.front());
    
    // Simple path smoothing: remove unnecessary waypoints
    for (size_t i = 1; i < raw_path.size() - 1; ++i) {
        const auto& prev = raw_path[i-1];
        const auto& curr = raw_path[i];
        const auto& next = raw_path[i+1];
        
        // Check if we can skip this point
        if (hasLineOfSight(prev, next)) {
            // Skip this point
            continue;
        } else {
            // Keep this point
            smoothed_path.push_back(curr);
        }
    }
    
    smoothed_path.push_back(raw_path.back());
    return smoothed_path;
}

bool GlobalPlannerBase::hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                                       const geometry_msgs::msg::PoseStamped& goal) const
{
    if (!env_) return false;
    
    // Convert to voxel coordinates
    int start_x, start_y, start_z, goal_x, goal_y, goal_z;
    if (!worldToVoxel(start.pose.position.x, start.pose.position.y, start.pose.position.z,
                      start_x, start_y, start_z) ||
        !worldToVoxel(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                      goal_x, goal_y, goal_z)) {
        return false;
    }
    
    // Use Bresenham's line algorithm to check line of sight
    int dx = abs(goal_x - start_x);
    int dy = abs(goal_y - start_y);
    int dz = abs(goal_z - start_z);
    
    int sx = (start_x < goal_x) ? 1 : -1;
    int sy = (start_y < goal_y) ? 1 : -1;
    int sz = (start_z < goal_z) ? 1 : -1;
    
    int x = start_x, y = start_y, z = start_z;
    
    // Check if start or goal is occupied
    if (env_->isOccupied(x, y, z) || env_->isOccupied(goal_x, goal_y, goal_z)) {
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