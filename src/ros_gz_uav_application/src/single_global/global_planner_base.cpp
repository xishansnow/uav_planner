#include "single_global/global_planner_base.hpp"
#include "env/environment_voxel3d.hpp"
#include "smoother/smoother_factory.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.hpp>

GlobalPlannerBase::GlobalPlannerBase(EnvironmentVoxel3D* env) : env_(env), smoother_(nullptr)
{
}

GlobalPlannerBase::~GlobalPlannerBase()
{
}

bool GlobalPlannerBase::worldToVoxel(double world_x, double world_y, double world_z, int& voxel_x, int& voxel_y,
                                     int& voxel_z) const
{
  if (!env_)
    return false;

  // 使用与 EnvironmentVoxel3D 相同的坐标转换逻辑
  return env_->WorldToVoxel(world_x, world_y, world_z, voxel_x, voxel_y, voxel_z);
}

void GlobalPlannerBase::voxelToWorld(int voxel_x, int voxel_y, int voxel_z, double& world_x, double& world_y,
                                     double& world_z) const
{
  if (!env_)
    return;

  // 使用与 EnvironmentVoxel3D 相同的坐标转换逻辑
  env_->VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);
}

void GlobalPlannerBase::setSmoother(SmootherType smooth_type, int degree, int num_points)
{
    switch (smooth_type) {
        case SmootherType::NORMAL:
            smoother_ = SmootherFactory::createSmoother(SmootherType::NORMAL, degree, num_points);
            break;
        case SmootherType::BEZIER:
            smoother_ = SmootherFactory::createSmoother(SmootherType::BEZIER, degree, num_points);
            break;
        case SmootherType::BSPLINE:
            smoother_ = SmootherFactory::createSmoother(SmootherType::BSPLINE, degree, num_points);
            break;
        case SmootherType::MINVO:
            smoother_ = SmootherFactory::createSmoother(SmootherType::MINVO, degree, num_points);
            break;
        case SmootherType::NONE:
        default:
            smoother_ = nullptr;
            break;
    }
}

bool GlobalPlannerBase::hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                                       const geometry_msgs::msg::PoseStamped& goal) const
{
  if (!env_)
    return false;

  // Convert to voxel coordinates
  int start_x, start_y, start_z, goal_x, goal_y, goal_z;
  if (!worldToVoxel(start.pose.position.x, start.pose.position.y, start.pose.position.z, start_x, start_y, start_z) ||
      !worldToVoxel(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_x, goal_y, goal_z))
  {
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
  if (env_->IsCellOccupied(x, y, z) || env_->IsCellOccupied(goal_x, goal_y, goal_z))
  {
    return false;
  }

  // Check intermediate points
  if (dx >= dy && dx >= dz)
  {
    int err_1 = 2 * dy - dx;
    int err_2 = 2 * dz - dx;

    for (int i = 0; i < dx; i++)
    {
      if (env_->IsCellOccupied(x, y, z))
        return false;

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
        return false;

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
        return false;

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

  return true;
}