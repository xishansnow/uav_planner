#include "single_global/multiscale_astar_planner.hpp"
#include "env/environment_voxel3d.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>

// 邻居方向 (26连通网格)
const int MultiScaleAStarPlanner::neighbor_dirs_[26][3] = {
    // 面邻居 (6)
    {-1, 0, 0},
    {1, 0, 0},
    {0, -1, 0},
    {0, 1, 0},
    {0, 0, -1},
    {0, 0, 1},
    // 边邻居 (12)
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
    // 角邻居 (8)
    {-1, -1, -1},
    {-1, -1, 1},
    {-1, 1, -1},
    {-1, 1, 1},
    {1, -1, -1},
    {1, -1, 1},
    {1, 1, -1},
    {1, 1, 1}};

// 默认参数说明:
// heuristic_weight_: 启发式权重，默认为1.0，用于平衡启发式估计和实际代价
// diagonal_cost_: 对角线移动的代价，默认为1.414 (√2)，表示对角线移动的距离
// vertical_cost_: 垂直移动的代价，默认为1.0，可以根据需要调整垂直移动的难度
// max_resolution_levels_: 最大分辨率层级数，默认为4，决定了多分辨率网格的层数
// min_resolution_: 最小分辨率，默认为0.1米，限制最细的网格分辨率
// adaptive_resolution_: 是否启用自适应分辨率，默认为true，允许在规划过程中动态调整分辨率

MultiScaleAStarPlanner::MultiScaleAStarPlanner(EnvironmentVoxel3D *env)
    : GlobalPlannerBase(env), heuristic_weight_(1.0), diagonal_cost_(1.414),
      vertical_cost_(1.0), max_resolution_levels_(4), min_resolution_(0.1),
      adaptive_resolution_(true)
{
    // 初始化分辨率层级, 根据环境大小和最高分辨率层级数, 计算所有分辨率层级的分辨率, 步长, 和代价因子
    initializeResolutionLevels(max_resolution_levels_);
}

MultiScaleAStarPlanner::~MultiScaleAStarPlanner()
{
}

void MultiScaleAStarPlanner::initializeResolutionLevels(int max_resolution_levels)
{
    resolution_levels_.clear();

    // 获取环境的基础分辨率
    double base_resolution = env_->GetResolutionXY();

    // 创建多分辨率层级 - 修复分辨率计算
    for (int i = 0; i < max_resolution_levels_; ++i)
    {
        double resolution = base_resolution * std::pow(2.0, i);
        int step_size = static_cast<int>(std::pow(2.0, i));
        double cost_factor = 1.0; // 保持代价一致，避免粗分辨率下代价过低

        resolution_levels_.emplace_back(i, resolution, step_size, cost_factor);

        std::cout << "Resolution level " << i << ": resolution=" << resolution
                  << "m, step_size=" << step_size << ", cost_factor=" << cost_factor << std::endl;
    }

    std::cout << "MultiScale A* initialized with " << resolution_levels_.size()
              << " resolution levels" << std::endl;
}

bool MultiScaleAStarPlanner::planPath(double start_x, double start_y, double start_z,
                                      double goal_x, double goal_y, double goal_z,
                                      std::vector<geometry_msgs::msg::PoseStamped> &path,
                                      SmootherType smooth_type,
                                      double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // 检查起点和终点是否有效
    int start_voxel_x, start_voxel_y, start_voxel_z;
    int goal_voxel_x, goal_voxel_y, goal_voxel_z;

    if (!worldToVoxel(start_x, start_y, start_z, start_voxel_x, start_voxel_y, start_voxel_z) ||
        !worldToVoxel(goal_x, goal_y, goal_z, goal_voxel_x, goal_voxel_y, goal_voxel_z))
    {
        std::cout << "MultiScale A*: Invalid start or goal position" << std::endl;
        return false;
    }

    if (env_->IsCellOccupied(start_voxel_x, start_voxel_y, start_voxel_z) ||
        env_->IsCellOccupied(goal_voxel_x, goal_voxel_y, goal_voxel_z))
    {
        std::cout << "MultiScale A*: Start or goal position is occupied" << std::endl;
        return false;
    }

    // 执行多尺度路径规划
    bool success = planPathMultiScale(start_x, start_y, start_z,
                                      goal_x, goal_y, goal_z, path, max_time);

    if (success)
    {
        std::cout << "MultiScale A*: Path found with " << path.size() << " waypoints" << std::endl;
        
        // 保存原始路径
        original_path_ = path;
        
        return true;
    }
    else
    {
        std::cout << "MultiScale A*: No path found" << std::endl;
    }

    return success;
}

bool MultiScaleAStarPlanner::planPathMultiScale(double start_x, double start_y, double start_z,
                                                double goal_x, double goal_y, double goal_z,
                                                std::vector<geometry_msgs::msg::PoseStamped> &path,
                                                double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // 确定最优起始分辨率层级
    int start_level = getOptimalStartLevel(start_x, start_y, start_z, goal_x, goal_y, goal_z);

    std::cout << "MultiScale A*: Starting search at resolution level " << start_level
              << " (resolution: " << resolution_levels_[start_level].resolution << "m)" << std::endl;

    // 从粗分辨率开始，逐步细化
    for (int level = start_level; level < max_resolution_levels_; ++level)
    {
        // 检查时间限制
        // auto current_time = std::chrono::high_resolution_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        // if (elapsed.count() > max_time * 1000) {
        //     std::cout << "MultiScale A*: Timeout at resolution level " << level << std::endl;
        //     return false;
        // }

        // 转换坐标到当前分辨率
        int start_res_x, start_res_y, start_res_z;
        int goal_res_x, goal_res_y, goal_res_z;

        worldToResolutionCoords(start_x, start_y, start_z, level, start_res_x, start_res_y, start_res_z);
        worldToResolutionCoords(goal_x, goal_y, goal_z, level, goal_res_x, goal_res_y, goal_res_z);

        // 创建起始和目标节点
        double h_start = calculateHeuristic(start_res_x, start_res_y, start_res_z, level,
                                            goal_res_x, goal_res_y, goal_res_z, level);
        MultiScaleAStarNode start_node(start_res_x, start_res_y, start_res_z, level, 0.0, h_start, -1, -1, -1, -1);
        MultiScaleAStarNode goal_node(goal_res_x, goal_res_y, goal_res_z, level, 0.0, 0.0, -1, -1, -1, -1);

        // 在当前分辨率下搜索路径
        std::vector<MultiScaleAStarNode> level_path;
        bool level_success = planPathAtResolution(level, start_node, goal_node, level_path, max_time);

        if (level_success)
        {
            std::cout << "MultiScale A*: Path found at resolution level " << level
                      << " with " << level_path.size() << " nodes" << std::endl;

            // 如果这是最高分辨率，直接返回结果
            if (level == max_resolution_levels_ - 1)
            {
                path = convertToWorldPath(level_path);
                return true;
            }

            // 否则，在更高分辨率下细化路径
            std::vector<MultiScaleAStarNode> refined_path = refinePathAtHigherResolution(level_path, level + 1);
            if (!refined_path.empty())
            {
                path = convertToWorldPath(refined_path);
                return true;
            }
        }
        else
        {
            std::cout << "MultiScale A*: No path found at resolution level " << level << std::endl;
        }
    }

    return false;
}

bool MultiScaleAStarPlanner::planPathAtResolution(int resolution_level,
                                                  const MultiScaleAStarNode &start_node,
                                                  const MultiScaleAStarNode &goal_node,
                                                  std::vector<MultiScaleAStarNode> &path,
                                                  double max_time)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "PlanPathAtResolution: Starting at level " << resolution_level
              << " from (" << start_node.x << ", " << start_node.y << ", " << start_node.z
              << ") to (" << goal_node.x << ", " << goal_node.y << ", " << goal_node.z << ")" << std::endl;

    // 检查起始点和目标点是否有效
    if (!isValidPositionAtResolution(start_node.x, start_node.y, start_node.z, resolution_level)) {
        std::cout << "PlanPathAtResolution: Start position is invalid or occupied" << std::endl;
        return false;
    }
    
    if (!isValidPositionAtResolution(goal_node.x, goal_node.y, goal_node.z, resolution_level)) {
        std::cout << "PlanPathAtResolution: Goal position is invalid or occupied" << std::endl;
        return false;
    }

    // 使用更高效的数据结构：Coord3D作为键而不是字符串
    std::priority_queue<MultiScaleAStarNode, std::vector<MultiScaleAStarNode>, std::greater<MultiScaleAStarNode>> open_set;
    std::unordered_map<Coord3D, double> g_scores;
    std::unordered_map<Coord3D, MultiScaleAStarNode> came_from;
    std::set<Coord3D> closed_set; // 使用Coord3D而不是MultiScaleAStarNode，提高查找效率

    // 初始化起始节点
    open_set.push(start_node);
    g_scores[Coord3D(start_node.x, start_node.y, start_node.z)] = 0.0;
    came_from[Coord3D(start_node.x, start_node.y, start_node.z)] = start_node;

    int nodes_expanded = 0;
    int max_nodes = 10000; // 防止搜索空间过大

    while (!open_set.empty() && nodes_expanded < max_nodes)
    {
        // 检查时间限制
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        // if (elapsed.count() > max_time * 1000) {
        //     std::cout << "PlanPathAtResolution: Timeout after " << elapsed.count() << "ms" << std::endl;
        //     return false;
        // }

        MultiScaleAStarNode current = open_set.top();
        open_set.pop();

        nodes_expanded++;

        // 检查是否到达目标
        if (current.x == goal_node.x && current.y == goal_node.y && current.z == goal_node.z)
        {
            std::cout << "PlanPathAtResolution: Goal reached after expanding " << nodes_expanded << " nodes" << std::endl;
            path = reconstructPathImproved(came_from, current);
            return true;
        }

        // 添加到关闭集（使用Coord3D）
        Coord3D current_coord(current.x, current.y, current.z);
        if (closed_set.find(current_coord) != closed_set.end())
        {
            continue; // 已经探索过
        }
        closed_set.insert(current_coord);
        // 用于跟踪探索过的节点，方便调试
        // std::cout << "AStarPlanner current node: " << current.x << ", " << current.y << ", " << current.z << std::endl;

        // 获取邻居
        std::vector<MultiScaleAStarNode> neighbors = getNeighbors(current);

        for (auto &neighbor : neighbors)
        {
            Coord3D neighbor_coord(neighbor.x, neighbor.y, neighbor.z);

            // 跳过已探索的节点
            if (closed_set.find(neighbor_coord) != closed_set.end())
            {
                continue;
            }

            // 计算启发式代价
            if (neighbor.h_cost == 0.0)
            {
                neighbor.h_cost = calculateHeuristic(neighbor.x, neighbor.y, neighbor.z, neighbor.resolution_level,
                                                     goal_node.x, goal_node.y, goal_node.z, goal_node.resolution_level);
            }

            double tentative_g = current.g_cost + calculateCost(current.x, current.y, current.z, current.resolution_level,
                                                                neighbor.x, neighbor.y, neighbor.z, neighbor.resolution_level);

            // 检查是否找到更好的路径
            auto g_it = g_scores.find(neighbor_coord);
            if (g_it == g_scores.end() || tentative_g < g_it->second)
            {
                // 这是更好的路径
                neighbor.g_cost = tentative_g;
                neighbor.parent_x = current.x;
                neighbor.parent_y = current.y;
                neighbor.parent_z = current.z;
                neighbor.parent_resolution_level = current.resolution_level;
                neighbor.f_cost = tentative_g + neighbor.h_cost;

                came_from[neighbor_coord] = neighbor;
                g_scores[neighbor_coord] = tentative_g;

                open_set.push(neighbor);
            }
        }
    }

    if (nodes_expanded >= max_nodes)
    {
        std::cout << "PlanPathAtResolution: Max nodes limit reached (" << max_nodes << ")" << std::endl;
    }
    else
    {
        std::cout << "PlanPathAtResolution: No path found after expanding " << nodes_expanded << " nodes" << std::endl;
    }
    return false;
}

int MultiScaleAStarPlanner::getOptimalStartLevel(double start_x, double start_y, double start_z,
                                                 double goal_x, double goal_y, double goal_z) const
{
    if (!adaptive_resolution_)
    {
        return 0; // 从最粗分辨率开始
    }

    // 计算起点和终点的距离
    double distance = std::sqrt(std::pow(goal_x - start_x, 2) +
                                std::pow(goal_y - start_y, 2) +
                                std::pow(goal_z - start_z, 2));

    // 获取环境信息
    int grid_size_x, grid_size_y, grid_size_z;
    env_->GetGridSize(grid_size_x, grid_size_y, grid_size_z);
    double base_resolution = env_->GetResolutionXY();

    // 计算搜索空间复杂度
    double search_complexity = calculateSearchComplexity(start_x, start_y, start_z,
                                                         goal_x, goal_y, goal_z, distance);

    // 计算障碍物密度
    double obstacle_density = calculateObstacleDensity(start_x, start_y, start_z,
                                                       goal_x, goal_y, goal_z, distance);

    // 计算最优起始分辨率层级
    int optimal_level = calculateOptimalResolutionLevel(distance, search_complexity,
                                                        obstacle_density, grid_size_x, grid_size_y, grid_size_z,
                                                        base_resolution);

    std::cout << "Optimal start level calculation:" << std::endl;
    std::cout << "  Distance: " << distance << "m" << std::endl;
    std::cout << "  Search complexity: " << search_complexity << std::endl;
    std::cout << "  Obstacle density: " << obstacle_density << std::endl;
    std::cout << "  Selected level: " << optimal_level << std::endl;

    return optimal_level;
}

void MultiScaleAStarPlanner::worldToResolutionCoords(double world_x, double world_y, double world_z,
                                                     int resolution_level,
                                                     int& res_x, int& res_y, int& res_z) const
{
    int voxel_x, voxel_y, voxel_z;
    env_->WorldToVoxel(world_x, world_y, world_z, voxel_x, voxel_y, voxel_z);
    int step_size = resolution_levels_[resolution_level].step_size;
    res_x = voxel_x / step_size;
    res_y = voxel_y / step_size;
    res_z = voxel_z / step_size;
    std::cout << "World (" << world_x << ", " << world_y << ", " << world_z
              << ") -> Voxel (" << voxel_x << ", " << voxel_y << ", " << voxel_z
              << ") -> Resolution " << resolution_level << " (" << res_x << ", " << res_y << ", " << res_z << ")" << std::endl;
}

void MultiScaleAStarPlanner::resolutionCoordsToWorld(int res_x, int res_y, int res_z,
                                                     int resolution_level,
                                                     double& world_x, double& world_y, double& world_z) const
{
    int step_size = resolution_levels_[resolution_level].step_size;
    int voxel_x = res_x * step_size;
    int voxel_y = res_y * step_size;
    int voxel_z = res_z * step_size;
    env_->VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
}

void MultiScaleAStarPlanner::resolutionCoordsToResolutionCoords(int from_x, int from_y, int from_z, int from_level,
                                                                int to_level, int &to_x, int &to_y, int &to_z) const
{
    // 将一种分辨率坐标转换为另一种分辨率坐标
    // 通过体素坐标作为中间步骤

    // 从源分辨率转换到体素坐标
    int from_step_size = resolution_levels_[from_level].step_size;
    int voxel_x = from_x * from_step_size;
    int voxel_y = from_y * from_step_size;
    int voxel_z = from_z * from_step_size;

    // 从体素坐标转换到目标分辨率坐标
    int to_step_size = resolution_levels_[to_level].step_size;
    to_x = voxel_x / to_step_size;
    to_y = voxel_y / to_step_size;
    to_z = voxel_z / to_step_size;

    std::cout << "Resolution conversion: (" << from_x << ", " << from_y << ", " << from_z
              << ") level " << from_level << " -> (" << to_x << ", " << to_y << ", " << to_z
              << ") level " << to_level << std::endl;
}

std::vector<geometry_msgs::msg::PoseStamped> MultiScaleAStarPlanner::convertToWorldPath(
    const std::vector<MultiScaleAStarNode> &resolution_path) const
{
    std::vector<geometry_msgs::msg::PoseStamped> world_path;

    for (const auto &node : resolution_path)
    {
        geometry_msgs::msg::PoseStamped pose;
        resolutionCoordsToWorld(node.x, node.y, node.z, node.resolution_level,
                                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        pose.pose.orientation.w = 1.0; // 默认方向
        world_path.push_back(pose);
    }

    return world_path;
}

std::vector<MultiScaleAStarNode> MultiScaleAStarPlanner::refinePathAtHigherResolution(
    const std::vector<MultiScaleAStarNode> &coarse_path,
    int target_resolution_level)
{
    if (coarse_path.empty() || target_resolution_level >= max_resolution_levels_)
    {
        return coarse_path;
    }

    std::vector<MultiScaleAStarNode> refined_path;

    for (size_t i = 0; i < coarse_path.size() - 1; ++i)
    {
        const auto &current = coarse_path[i];
        const auto &next = coarse_path[i + 1];

        // 将粗分辨率坐标转换为目标分辨率坐标
        // 注意：current.x, current.y, current.z 是粗分辨率坐标，不是世界坐标
        int current_res_x, current_res_y, current_res_z;
        int next_res_x, next_res_y, next_res_z;

        // 正确的坐标转换：从粗分辨率到细分辨率
        resolutionCoordsToResolutionCoords(current.x, current.y, current.z, current.resolution_level,
                                           target_resolution_level, current_res_x, current_res_y, current_res_z);
        resolutionCoordsToResolutionCoords(next.x, next.y, next.z, next.resolution_level,
                                           target_resolution_level, next_res_x, next_res_y, next_res_z);

        // std::cout << "Refining path segment: from (" << current.x << ", " << current.y << ", " << current.z
        //           << ") level " << current.resolution_level << " to (" << next.x << ", " << next.y << ", " << next.z
        //           << ") level " << next.resolution_level << std::endl;
        // std::cout << "Target resolution coordinates: (" << current_res_x << ", " << current_res_y << ", " << current_res_z
        //           << ") to (" << next_res_x << ", " << next_res_y << ", " << next_res_z << ") level " << target_resolution_level << std::endl;

        // 在高分辨率下搜索连接路径
        MultiScaleAStarNode start_node(current_res_x, current_res_y, current_res_z, target_resolution_level, 0.0, 0.0, -1, -1, -1, -1);
        MultiScaleAStarNode goal_node(next_res_x, next_res_y, next_res_z, target_resolution_level, 0.0, 0.0, -1, -1, -1, -1);

        std::vector<MultiScaleAStarNode> segment_path;
        if (planPathAtResolution(target_resolution_level, start_node, goal_node, segment_path, 1.0))
        {
            // 添加细化后的路径段（除了最后一个节点，避免重复）
            for (size_t j = 0; j < segment_path.size() - 1; ++j)
            {
                refined_path.push_back(segment_path[j]);
            }
        }
        else
        {
            // 如果细化失败，保持原路径（需要转换到目标分辨率）
            MultiScaleAStarNode current_at_target_res(current_res_x, current_res_y, current_res_z, target_resolution_level, 0.0, 0.0, -1, -1, -1, -1);
            refined_path.push_back(current_at_target_res);
        }
    }

    // 添加最后一个节点（转换到目标分辨率）
    if (!coarse_path.empty())
    {
        const auto &last = coarse_path.back();
        int last_res_x, last_res_y, last_res_z;
        resolutionCoordsToResolutionCoords(last.x, last.y, last.z, last.resolution_level,
                                           target_resolution_level, last_res_x, last_res_y, last_res_z);
        MultiScaleAStarNode last_at_target_res(last_res_x, last_res_y, last_res_z, target_resolution_level, 0.0, 0.0, -1, -1, -1, -1);
        refined_path.push_back(last_at_target_res);
    }

    std::cout << "Refined path has " << refined_path.size() << " nodes at resolution level " << target_resolution_level << std::endl;
    return refined_path;
}

double MultiScaleAStarPlanner::calculateHeuristic(int x1, int y1, int z1, int level1,
                                                  int x2, int y2, int z2, int level2) const
{
    // 欧几里得距离启发式
    double dx = (x2 - x1) * resolution_levels_[level1].resolution;
    double dy = (y2 - y1) * resolution_levels_[level1].resolution;
    double dz = (z2 - z1) * resolution_levels_[level1].resolution;

    return heuristic_weight_ * std::sqrt(dx * dx + dy * dy + dz * dz);
}

double MultiScaleAStarPlanner::calculateCost(int x1, int y1, int z1, int level1,
                                             int x2, int y2, int z2, int level2) const
{
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);

    // 计算移动类型
    int max_delta = std::max({dx, dy, dz});

    if (max_delta == 0)
        return 0.0; // 相同位置

    double base_cost = 0.0;
    if (max_delta == 1)
    {
        // 面邻居
        if (dx + dy + dz == 1)
            base_cost = 1.0;
        // 边邻居
        else if (dx + dy + dz == 2)
            base_cost = diagonal_cost_;
        // 角邻居
        else
            base_cost = diagonal_cost_ * 1.732; // sqrt(3)
    }
    else
    {
        // 直接距离
        base_cost = std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // 应用分辨率层级的代价因子
    double cost_factor = resolution_levels_[level1].cost_factor;
    return base_cost * cost_factor;
}

std::vector<MultiScaleAStarNode> MultiScaleAStarPlanner::getNeighbors(const MultiScaleAStarNode &node) const
{
    std::vector<MultiScaleAStarNode> neighbors;

    for (int i = 0; i < num_neighbors_; ++i)
    {
        int nx = node.x + neighbor_dirs_[i][0];
        int ny = node.y + neighbor_dirs_[i][1];
        int nz = node.z + neighbor_dirs_[i][2];

        // 检查邻居节点是否在环境范围内
        // if (nz > env_->GetGridZ() - 3 || nz < 3) {
        //     continue;
        // }
        
        // 检查邻居节点是否有效
        if (!isValidPositionAtResolution(nx, ny, nz, node.resolution_level))
        {
            continue;
        }

        // 创建邻居节点
        neighbors.emplace_back(nx, ny, nz, node.resolution_level, 0.0, 0.0, -1, -1, -1, -1);
    }

    return neighbors;
}

std::vector<MultiScaleAStarNode> MultiScaleAStarPlanner::reconstructPath(
    const std::unordered_map<std::string, MultiScaleAStarNode> &came_from,
    const MultiScaleAStarNode &goal_node) const
{
    std::vector<MultiScaleAStarNode> path;

    MultiScaleAStarNode current = goal_node;

    // 安全检查：防止无限循环
    const int max_iterations = 1000;
    int iteration_count = 0;

    while (iteration_count < max_iterations)
    {
        path.push_back(current);

        // 查找父节点
        std::string current_key = nodeToKey(current);
        auto it = came_from.find(current_key);

        if (it == came_from.end())
        {
            std::cout << "ReconstructPath: No parent found for node " << current_key << std::endl;
            break; // 到达起点
        }

        const MultiScaleAStarNode &parent = it->second;

        // 检查是否到达起始节点（父节点坐标为-1表示起始节点）
        if (parent.parent_x == -1 && parent.parent_y == -1 && parent.parent_z == -1)
        {
            path.push_back(parent);
            std::cout << "ReconstructPath: Reached start node" << std::endl;
            break;
        }

        // 安全检查：防止自引用循环
        if (parent.x == current.x && parent.y == current.y && parent.z == current.z)
        {
            std::cout << "ReconstructPath: Self-reference detected" << std::endl;
            break;
        }

        current = parent;
        iteration_count++;
    }

    // 检查是否达到迭代限制
    if (iteration_count >= max_iterations)
    {
        std::cout << "ReconstructPath: Max iterations reached" << std::endl;
        return std::vector<MultiScaleAStarNode>();
    }

    // 反转路径以获得从起点到终点的顺序
    std::reverse(path.begin(), path.end());

    std::cout << "ReconstructPath: Reconstructed path with " << path.size() << " nodes" << std::endl;
    return path;
}

std::vector<MultiScaleAStarNode> MultiScaleAStarPlanner::reconstructPathImproved(
    const std::unordered_map<Coord3D, MultiScaleAStarNode> &came_from,
    const MultiScaleAStarNode &goal_node) const
{
    std::vector<MultiScaleAStarNode> path;

    // 使用坐标直接进行路径重建，参考 AStarPlanner 的实现
    int current_x = goal_node.x, current_y = goal_node.y, current_z = goal_node.z;
    int current_resolution_level = goal_node.resolution_level;

    // 安全检查：防止无限循环
    const int max_iterations = 1000;
    int iteration_count = 0;

    while (iteration_count < max_iterations)
    {
        // 创建当前节点并添加到路径
        MultiScaleAStarNode current_node(current_x, current_y, current_z, current_resolution_level,
                                         0.0, 0.0, -1, -1, -1, -1);
        path.push_back(current_node);

        // 查找父节点
        Coord3D current_coord(current_x, current_y, current_z);
        auto it = came_from.find(current_coord);

        if (it == came_from.end())
        {
            std::cout << "ReconstructPathImproved: No parent found for node ("
                      << current_x << ", " << current_y << ", " << current_z << ")" << std::endl;
            break; // 到达起点
        }

        const MultiScaleAStarNode &current_node_data = it->second;

        // 检查是否到达起始节点（父节点坐标为-1表示起始节点）
        if (current_node_data.parent_x == -1 && current_node_data.parent_y == -1 && current_node_data.parent_z == -1)
        {
            std::cout << "ReconstructPathImproved: Reached start node" << std::endl;
            break;
        }

        // 安全检查：防止自引用循环
        if (current_node_data.parent_x == current_x && current_node_data.parent_y == current_y && current_node_data.parent_z == current_z)
        {
            std::cout << "ReconstructPathImproved: Self-reference detected" << std::endl;
            break;
        }

        // 移动到父节点坐标
        current_x = current_node_data.parent_x;
        current_y = current_node_data.parent_y;
        current_z = current_node_data.parent_z;
        current_resolution_level = current_node_data.parent_resolution_level;

        iteration_count++;
    }

    // 检查是否达到迭代限制
    if (iteration_count >= max_iterations)
    {
        std::cout << "ReconstructPathImproved: Max iterations reached" << std::endl;
        return std::vector<MultiScaleAStarNode>();
    }

    // 反转路径以获得从起点到终点的顺序
    std::reverse(path.begin(), path.end());

    std::cout << "ReconstructPathImproved: Reconstructed path with " << path.size() << " nodes" << std::endl;
    return path;
}

bool MultiScaleAStarPlanner::isValidPositionAtResolution(int x, int y, int z, int resolution_level) const
{
    // 检查分辨率层级是否有效
    if (resolution_level < 0 || resolution_level >= resolution_levels_.size()) {
        std::cout << "Invalid resolution level: " << resolution_level << std::endl;
        return false;
    }

    // 获取网格大小
    int size_x, size_y, size_z;
    env_->GetGridSize(size_x, size_y, size_z);

    // 根据分辨率层级调整边界检查
    int step_size = resolution_levels_[resolution_level].step_size;
    if (step_size <= 0) {
        std::cout << "Invalid step size: " << step_size << " for level " << resolution_level << std::endl;
        return false;
    }

    // 修复边界检查：使用向上取整确保覆盖所有体素
    int max_x = (size_x + step_size - 1) / step_size;
    int max_y = (size_y + step_size - 1) / step_size;
    int max_z = (size_z + step_size - 1) / step_size;

    if (x < 0 || x >= max_x || y < 0 || y >= max_y || z < 0 || z >= max_z)
    {
        return false;
    }

    // 检查是否被占用（在原始分辨率下检查）
    int voxel_x = x * step_size;
    int voxel_y = y * step_size;
    int voxel_z = z * step_size;

    // 更严格的障碍物检测：只要有一个体素被占用，就认为该位置无效
    for (int dx = 0; dx < step_size; ++dx)
    {
        for (int dy = 0; dy < step_size; ++dy)
        {
            for (int dz = 0; dz < step_size; ++dz)
            {
                int check_x = voxel_x + dx;
                int check_y = voxel_y + dy;
                int check_z = voxel_z + dz;

                if (check_x < size_x && check_y < size_y && check_z < size_z)
                {
                    if (env_->IsCellOccupied(check_x, check_y, check_z))
                    {
                        // 只要有一个体素被占用，就认为该位置无效
                        // std::cout << "Position (" << x << ", " << y << ", " << z << ") at level " << resolution_level 
                                //   << " is occupied at voxel (" << check_x << ", " << check_y << ", " << check_z << ")" << std::endl;
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

std::string MultiScaleAStarPlanner::nodeToKey(const MultiScaleAStarNode &node) const
{
    std::ostringstream oss;
    oss << node.resolution_level << "_" << node.x << "_" << node.y << "_" << node.z;
    return oss.str();
}

bool MultiScaleAStarPlanner::isPathValid(const std::vector<MultiScaleAStarNode> &path) const
{
    if (path.size() < 2)
        return true;

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        const auto &current = path[i];
        const auto &next = path[i + 1];

        // 检查相邻节点是否有效
        if (!isValidPositionAtResolution(current.x, current.y, current.z, current.resolution_level) ||
            !isValidPositionAtResolution(next.x, next.y, next.z, next.resolution_level))
        {
            return false;
        }
    }

    return true;
}

double MultiScaleAStarPlanner::estimatePathComplexity(double start_x, double start_y, double start_z,
                                                      double goal_x, double goal_y, double goal_z) const
{
    // 简单的复杂度估计：基于起点和终点之间的障碍物密度
    // 这里使用一个简化的估计方法

    double distance = std::sqrt(std::pow(goal_x - start_x, 2) +
                                std::pow(goal_y - start_y, 2) +
                                std::pow(goal_z - start_z, 2));

    // 距离越远，复杂度越高
    double distance_factor = std::min(distance / 50.0, 1.0);

    // 可以在这里添加更复杂的障碍物密度估计
    // 目前返回一个基于距离的简单估计
    return distance_factor;
}

double MultiScaleAStarPlanner::calculateSearchComplexity(double start_x, double start_y, double start_z,
                                                         double goal_x, double goal_y, double goal_z,
                                                         double distance) const
{
    // 计算搜索空间复杂度，考虑路径的几何特性
    double dx = goal_x - start_x;
    double dy = goal_y - start_y;
    double dz = goal_z - start_z;

    // 计算路径的方向复杂度（方向变化越多，复杂度越高）
    double direction_complexity = 0.0;

    // 如果路径主要在某个轴上，复杂度较低
    double max_component = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
    if (max_component > 0)
    {
        double x_ratio = std::abs(dx) / max_component;
        double y_ratio = std::abs(dy) / max_component;
        double z_ratio = std::abs(dz) / max_component;

        // 方向复杂度：如果路径不是直线，复杂度更高
        direction_complexity = 1.0 - std::max({x_ratio, y_ratio, z_ratio});
    }

    // 距离复杂度：距离越远，搜索空间越大
    double distance_complexity = std::min(distance / 100.0, 1.0);

    // 综合复杂度
    double search_complexity = 0.6 * distance_complexity + 0.4 * direction_complexity;

    return std::min(search_complexity, 1.0);
}

double MultiScaleAStarPlanner::calculateObstacleDensity(double start_x, double start_y, double start_z,
                                                        double goal_x, double goal_y, double goal_z,
                                                        double distance) const
{
    // 计算起点到终点直线路径上的障碍物密度
    int start_voxel_x, start_voxel_y, start_voxel_z;
    int goal_voxel_x, goal_voxel_y, goal_voxel_z;

    if (!worldToVoxel(start_x, start_y, start_z, start_voxel_x, start_voxel_y, start_voxel_z) ||
        !worldToVoxel(goal_x, goal_y, goal_z, goal_voxel_x, goal_voxel_y, goal_voxel_z))
    {
        return 0.5; // 默认中等密度
    }

    // 使用Bresenham算法采样直线路径上的体素
    int dx = abs(goal_voxel_x - start_voxel_x);
    int dy = abs(goal_voxel_y - start_voxel_y);
    int dz = abs(goal_voxel_z - start_voxel_z);

    int sx = (start_voxel_x < goal_voxel_x) ? 1 : -1;
    int sy = (start_voxel_y < goal_voxel_y) ? 1 : -1;
    int sz = (start_voxel_z < goal_voxel_z) ? 1 : -1;

    int x = start_voxel_x;
    int y = start_voxel_y;
    int z = start_voxel_z;

    int occupied_count = 0;
    int total_count = 0;

    // 3D Bresenham算法采样
    if (dx >= dy && dx >= dz)
    {
        int err_1 = 2 * dy - dx;
        int err_2 = 2 * dz - dx;

        for (int i = 0; i < dx; i++)
        {
            if (env_->IsCellOccupied(x, y, z))
            {
                occupied_count++;
            }
            total_count++;

            if (err_1 > 0)
            {
                y += sy;
                err_1 -= 2 * dx;
            }
            if (err_2 > 0)
            {
                z += sz;
                err_2 -= 2 * dx;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dz;
            x += sx;
        }
    }
    else if (dy >= dx && dy >= dz)
    {
        int err_1 = 2 * dx - dy;
        int err_2 = 2 * dz - dy;

        for (int i = 0; i < dy; i++)
        {
            if (env_->IsCellOccupied(x, y, z))
            {
                occupied_count++;
            }
            total_count++;

            if (err_1 > 0)
            {
                x += sx;
                err_1 -= 2 * dy;
            }
            if (err_2 > 0)
            {
                z += sz;
                err_2 -= 2 * dy;
            }
            err_1 += 2 * dx;
            err_2 += 2 * dz;
            y += sy;
        }
    }
    else
    {
        int err_1 = 2 * dy - dz;
        int err_2 = 2 * dx - dz;

        for (int i = 0; i < dz; i++)
        {
            if (env_->IsCellOccupied(x, y, z))
            {
                occupied_count++;
            }
            total_count++;

            if (err_1 > 0)
            {
                y += sy;
                err_1 -= 2 * dz;
            }
            if (err_2 > 0)
            {
                x += sx;
                err_2 -= 2 * dz;
            }
            err_1 += 2 * dy;
            err_2 += 2 * dx;
            z += sz;
        }
    }

    // 计算障碍物密度
    double obstacle_density = (total_count > 0) ? (double)occupied_count / total_count : 0.0;

    return obstacle_density;
}

int MultiScaleAStarPlanner::calculateOptimalResolutionLevel(double distance, double search_complexity,
                                                            double obstacle_density, int grid_size_x, int grid_size_y, int grid_size_z,
                                                            double base_resolution) const
{
    // 计算不同分辨率下的搜索空间大小
    std::vector<double> search_space_sizes;
    std::vector<double> efficiency_scores;

    for (int level = 0; level < max_resolution_levels_; ++level)
    {
        const auto &res_level = resolution_levels_[level];

        // 计算该分辨率下的搜索空间大小
        int step_size = res_level.step_size;
        int res_grid_x = grid_size_x / step_size;
        int res_grid_y = grid_size_y / step_size;
        int res_grid_z = grid_size_z / step_size;

        double search_space_size = (double)(res_grid_x * res_grid_y * res_grid_z);
        search_space_sizes.push_back(search_space_size);

        // 计算效率分数（综合考虑搜索空间大小、障碍物密度、路径复杂度）
        double efficiency_score = calculateEfficiencyScore(level, search_space_size,
                                                           obstacle_density, search_complexity, distance);
        efficiency_scores.push_back(efficiency_score);
    }

    // 选择效率分数最高的分辨率层级
    int optimal_level = 0;
    double best_efficiency = efficiency_scores[0];

    for (int level = 1; level < max_resolution_levels_; ++level)
    {
        if (efficiency_scores[level] > best_efficiency)
        {
            best_efficiency = efficiency_scores[level];
            optimal_level = level;
        }
    }

    // 输出详细信息
    std::cout << "Resolution level efficiency analysis:" << std::endl;
    for (int level = 0; level < max_resolution_levels_; ++level)
    {
        std::cout << "  Level " << level << ": search_space=" << search_space_sizes[level]
                  << ", efficiency=" << efficiency_scores[level] << std::endl;
    }

    return optimal_level;
}

double MultiScaleAStarPlanner::calculateEfficiencyScore(int level, double search_space_size,
                                                        double obstacle_density, double search_complexity,
                                                        double distance) const
{
    // 计算效率分数的综合指标

    // 1. 搜索空间效率：搜索空间越小，效率越高
    double search_space_efficiency = 1.0 / (1.0 + search_space_size / 1000000.0);

    // 2. 分辨率适应性：根据障碍物密度和复杂度调整
    double resolution_adaptation = 1.0;
    if (obstacle_density > 0.3)
    {
        // 高障碍物密度时，倾向于使用更细的分辨率
        resolution_adaptation = 1.0 - 0.3 * level / max_resolution_levels_;
    }
    else if (search_complexity > 0.7)
    {
        // 高复杂度时，倾向于使用更粗的分辨率
        resolution_adaptation = 1.0 - 0.2 * (max_resolution_levels_ - 1 - level) / max_resolution_levels_;
    }

    // 3. 距离适应性：距离越远，倾向于使用更粗的分辨率
    double distance_adaptation = 1.0;
    if (distance > 50.0)
    {
        distance_adaptation = 1.0 - 0.2 * level / max_resolution_levels_;
    }

    // 4. 计算复杂度惩罚：层级越高，计算复杂度越高
    double computation_penalty = 1.0 - 0.1 * level;

    // 综合效率分数
    double efficiency_score = search_space_efficiency * resolution_adaptation *
                              distance_adaptation * computation_penalty;

    return std::max(efficiency_score, 0.0);
}

bool MultiScaleAStarPlanner::replanPath(const geometry_msgs::msg::PoseStamped& current_position,
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

void MultiScaleAStarPlanner::setParameters(const std::string &param_name, double value)
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
    else if (param_name == "min_resolution")
    {
        min_resolution_ = value;
        initializeResolutionLevels();
    }
}

void MultiScaleAStarPlanner::setMaxResolutionLevels(int max_levels)
{
    max_resolution_levels_ = max_levels;
    initializeResolutionLevels();
}

void MultiScaleAStarPlanner::setMinResolution(double min_res)
{
    min_resolution_ = min_res;
    initializeResolutionLevels();
}

void MultiScaleAStarPlanner::setAdaptiveResolution(bool enable)
{
    adaptive_resolution_ = enable;
}
