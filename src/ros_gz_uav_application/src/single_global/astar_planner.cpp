#include "single_global/astar_planner.hpp"
#include "env/environment_voxel3d.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

// Neighbor directions (26-connected grid)
const int AStarPlanner::neighbor_dirs_[26][3] = {
    // Face neighbors (6)
    {-1, 0, 0},
    {1, 0, 0},
    {0, -1, 0},
    {0, 1, 0},
    {0, 0, -1},
    {0, 0, 1},
    // Edge neighbors (12)
    {-1, -1, 0},
    {-1, 1, 0},
    {1, -1, 0},
    {1, 1, 0},
    {-1, 0, -1},
    {-1, 0, 1},
    {1, 0, -1},
    {1, 0, 1},
    {0, -1, -1},
    {0, -1, 1},
    {0, 1, -1},
    {0, 1, 1},
    // Corner neighbors (8)
    {-1, -1, -1},
    {-1, -1, 1},
    {-1, 1, -1},
    {-1, 1, 1},
    {1, -1, -1},
    {1, -1, 1},
    {1, 1, -1},
    {1, 1, 1}};

AStarPlanner::AStarPlanner(EnvironmentVoxel3D *env)
    : GlobalPlannerBase(env), heuristic_weight_(1.0), diagonal_cost_(1.414), vertical_cost_(1.0)
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::planPath(double start_x, double start_y, double start_z,
                            double goal_x, double goal_y, double goal_z,
                            std::vector<geometry_msgs::msg::PoseStamped> &path,
                            SmootherType smooth_type,
                            double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert world coordinates to voxel coordinates
    int start_voxel_x, start_voxel_y, start_voxel_z;
    int goal_voxel_x, goal_voxel_y, goal_voxel_z;

    if (!worldToVoxel(start_x, start_y, start_z, start_voxel_x, start_voxel_y, start_voxel_z) ||
        !worldToVoxel(goal_x, goal_y, goal_z, goal_voxel_x, goal_voxel_y, goal_voxel_z))
    {
        return false;
    }

    // Check if start or goal is occupied
    if (env_->IsCellOccupied(start_voxel_x, start_voxel_y, start_voxel_z) ||
        env_->IsCellOccupied(goal_voxel_x, goal_voxel_y, goal_voxel_z))
    {
        return false;
    }

    // Initialize data structures
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
    std::unordered_map<Coord3D, double> g_scores;
    std::unordered_map<Coord3D, AStarNode> came_from;
    std::set<Coord3D> closed_set; // Use Coord3D instead of AStarNode for faster lookup

    // Initialize start node
    double h_start = calculateHeuristic(start_voxel_x, start_voxel_y, start_voxel_z,
                                        goal_voxel_x, goal_voxel_y, goal_voxel_z);

    AStarNode start_node(start_voxel_x, start_voxel_y, start_voxel_z, 0.0, h_start,
                         -1, -1, -1); // Use invalid coordinates (-1, -1, -1) for start node parent

    open_set.push(start_node);
    g_scores[Coord3D(start_voxel_x, start_voxel_y, start_voxel_z)] = 0.0;
    came_from[Coord3D(start_voxel_x, start_voxel_y, start_voxel_z)] = start_node;

    int nodes_expanded = 0;

    while (!open_set.empty())
    {
        // Check time limit

        // auto current_time = std::chrono::high_resolution_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        // if (elapsed.count() > max_time * 1000) {
        //     std::cout << "A*: Timeout after " << elapsed.count() << "ms" << std::endl;
        //     return false;
        // }

        AStarNode current = open_set.top();
        open_set.pop();

        nodes_expanded++;

        // Check if we reached the goal
        if (current.x == goal_voxel_x && current.y == goal_voxel_y && current.z == goal_voxel_z)
        {
            std::cout << "A*: Goal reached after expanding " << nodes_expanded << " nodes" << std::endl;
            path = reconstructPath(came_from, goal_voxel_x, goal_voxel_y, goal_voxel_z);

            // 保存原始路径
            original_path_ = path;

            return true;
        }

        // Add to closed set
        Coord3D current_coord(current.x, current.y, current.z);
        if (closed_set.find(current_coord) != closed_set.end())
        {
            continue; // Already explored
        }
        closed_set.insert(current_coord);

        // 调试信息
        // std::cout << "A*: Expanding node (" << current.x << ", " << current.y << ", " << current.z << ")" << std::endl;

        // Get neighbors
        std::vector<AStarNode> neighbors = getNeighbors(current);

        for (auto &neighbor : neighbors)
        {
            Coord3D neighbor_coord(neighbor.x, neighbor.y, neighbor.z);

            // Skip if already explored
            if (closed_set.find(neighbor_coord) != closed_set.end())
            {
                continue;
            }

            // Calculate heuristic for neighbor if not already set
            if (neighbor.h_cost == 0.0)
            {
                neighbor.h_cost = calculateHeuristic(neighbor.x, neighbor.y, neighbor.z,
                                                     goal_voxel_x, goal_voxel_y, goal_voxel_z);
            }

            double tentative_g = current.g_cost + calculateCost(current.x, current.y, current.z,
                                                                neighbor.x, neighbor.y, neighbor.z);

            // Check if this path is better
            auto g_it = g_scores.find(neighbor_coord);
            if (g_it == g_scores.end() || tentative_g < g_it->second)
            {
                // This path is better
                neighbor.g_cost = tentative_g;
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;
                neighbor.parent_z = current.z;
                neighbor.f_cost = tentative_g + neighbor.h_cost;

                came_from[neighbor_coord] = neighbor;
                g_scores[neighbor_coord] = tentative_g;

                open_set.push(neighbor);
            }
        }
    }

    // if (nodes_expanded >= max_nodes) {
    //     std::cout << "A*: Max nodes limit reached (" << max_nodes << ")" << std::endl;
    // } else {
    std::cout << "A*: No path found after expanding " << nodes_expanded << " nodes" << std::endl;
    // }

    return false; // No path found
}

bool AStarPlanner::replanPath(const geometry_msgs::msg::PoseStamped &current_position,
                              const std::vector<geometry_msgs::msg::PoseStamped> &current_path,
                              std::vector<geometry_msgs::msg::PoseStamped> &new_path,
                              SmootherType smooth_type,
                              double max_time)
{
    if (current_path.empty())
        return false;

    // Use provided current position as start
    // Find the goal (last waypoint in current path)
    const auto &goal_pos = current_path.front();

    return planPath(current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z,
                    goal_pos.pose.position.x, goal_pos.pose.position.y, goal_pos.pose.position.z,
                    new_path, smooth_type, max_time);
}

void AStarPlanner::setParameters(const std::string &param_name, double value)
{
    if (param_name == "heuristic_weight")
    {
        heuristic_weight_ = value;
    }
    else if (param_name == "diagonal_cost")
    {
        diagonal_cost_ = value;
    }
    else if (param_name == "vertical_cost")
    {
        vertical_cost_ = value;
    }
}

double AStarPlanner::calculateHeuristic(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Get voxel resolution for accurate distance calculation
    double resolution_xy = env_->GetResolutionXY();
    double resolution_z = env_->GetResolutionZ();

    // Calculate actual world distance considering voxel resolution
    double dx = (x2 - x1) * resolution_xy;
    double dy = (y2 - y1) * resolution_xy;
    double dz = (z2 - z1) * resolution_z;

    // Use Euclidean distance heuristic with resolution consideration
    return heuristic_weight_ * std::sqrt(dx * dx + dy * dy + dz * dz);
}

double AStarPlanner::calculateCost(int x1, int y1, int z1, int x2, int y2, int z2) const
{
    // Get voxel resolution for accurate cost calculation
    double resolution_xy = env_->GetResolutionXY();
    double resolution_z = env_->GetResolutionZ();

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);

    // Calculate movement type
    int max_delta = std::max({dx, dy, dz});

    if (max_delta == 0)
        return 0.0; // Same position

    double cost = 0.0;

    if (max_delta == 1)
    {
        // Face neighbor
        if (dx + dy + dz == 1)
        {
            if (dx == 1 || dy == 1)
            {
                cost = resolution_xy; // XY movement
            }
            else
            {
                cost = resolution_z; // Z movement
            }
        }
        // Edge neighbor
        else if (dx + dy + dz == 2)
        {
            if (dz == 0)
            {
                cost = diagonal_cost_ * resolution_xy; // XY diagonal
            }
            else
            {
                cost = std::sqrt(resolution_xy * resolution_xy + resolution_z * resolution_z); // Mixed diagonal
            }
        }
        // Corner neighbor
        else
        {
            cost = diagonal_cost_ * std::sqrt(resolution_xy * resolution_xy + resolution_xy * resolution_xy + resolution_z * resolution_z);
        }
    }
    else
    {
        // Direct distance for longer moves (should not happen in normal A*)
        double world_dx = dx * resolution_xy;
        double world_dy = dy * resolution_xy;
        double world_dz = dz * resolution_z;
        cost = std::sqrt(world_dx * world_dx + world_dy * world_dy + world_dz * world_dz);
    }

    return cost;
}

std::vector<AStarNode> AStarPlanner::getNeighbors(const AStarNode &node) const
{
    std::vector<AStarNode> neighbors;

    for (int i = 0; i < num_neighbors_; ++i)
    {
        int nx = node.x + neighbor_dirs_[i][0];
        int ny = node.y + neighbor_dirs_[i][1];
        int nz = node.z + neighbor_dirs_[i][2];

        if (nz > env_->GetGridZ() - 3 || nz < 3)
        {
            continue;
        }

        // Check if within map cell (using voxel coordinates)
        if (!env_->IsCellWithinMap(nx, ny, nz))
        {
            continue;
        }

        // Check if occupied (using voxel coordinates)
        if (env_->IsCellOccupied(nx, ny, nz))
        {
            continue;
        }

        // Create neighbor node with invalid parent coordinates initially
        // The actual parent will be set in planPath when the node is processed
        neighbors.emplace_back(nx, ny, nz, 0.0, 0.0, -1, -1, -1);
    }

    return neighbors;
}

std::vector<geometry_msgs::msg::PoseStamped> AStarPlanner::reconstructPath(
    const std::unordered_map<Coord3D, AStarNode> &came_from,
    int goal_x, int goal_y, int goal_z) const
{
    std::vector<geometry_msgs::msg::PoseStamped> path;

    int current_x = goal_x, current_y = goal_y, current_z = goal_z;

    // Safety check: prevent infinite loops
    const int max_iterations = 1000; // Reasonable limit for path reconstruction
    int iteration_count = 0;

    while (iteration_count < max_iterations)
    {
        // Add current position to path
        geometry_msgs::msg::PoseStamped pose;
        voxelToWorld(current_x, current_y, current_z,
                     pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        pose.pose.orientation.w = 1.0; // Default orientation
        path.push_back(pose);

        // Find parent
        Coord3D current_coord(current_x, current_y, current_z);
        auto it = came_from.find(current_coord);

        if (it == came_from.end())
        {
            break; // Reached start
        }

        const AStarNode &current_node = it->second;

        // Check if we've reached the start node (parent coordinates are invalid)
        if (current_node.parent_x == -1 && current_node.parent_y == -1 && current_node.parent_z == -1)
        {
            break;
        }

        // Safety check: prevent self-reference loops
        if (current_node.parent_x == current_node.x && current_node.parent_y == current_node.y && current_node.parent_z == current_node.z)
        {
            break;
        }

        // Move to the parent coordinates
        current_x = current_node.parent_x;
        current_y = current_node.parent_y;
        current_z = current_node.parent_z;

        iteration_count++;
    }

    // Check if we hit the iteration limit (indicates a problem)
    if (iteration_count >= max_iterations)
    {
        return std::vector<geometry_msgs::msg::PoseStamped>();
    }

    // Reverse path to get start to goal order
    std::reverse(path.begin(), path.end());
    return path;
}

// bool AStarPlanner::isValidPosition(int x, int y, int z) const
// {
//     // Check if within map cell (using voxel coordinates)
//     if (!env_->IsCellWithinMap(x, y, z)) {
//         return false;
//     }

//     // Check if occupied (using voxel coordinates)
//     if (env_->IsCellOccupied(x, y, z)) {
//         return false;
//     }

//     return true;
// }