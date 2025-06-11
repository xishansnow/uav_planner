#include "sbpl_planner.hpp"
#include "environment_voxel3d.hpp"
#include <sbpl/headers.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

#include <cmath>
#include <vector>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

UAV_Sbpl_Planner::UAV_Sbpl_Planner(double resolution, double origin_x, double origin_y, double origin_z)
: resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y), origin_z_(origin_z)
{
  // 初始化3D体素网格尺寸（基于分辨率）
  voxel_width_ = 200;   // 200个体素单元
  voxel_height_ = 200;
  voxel_depth_ = 100;   // 100个体素单元（高度）
  
  // 初始化SBPL 3D体素环境（内部包含octomap）
  initializeSBPLEnvironment();
}

UAV_Sbpl_Planner::~UAV_Sbpl_Planner()
{
  if (env_) {
    delete env_;
  }
  if (planner_) {
    delete planner_;
  }
}

void UAV_Sbpl_Planner::initializeSBPLEnvironment()
{
  // 创建SBPL 3D体素环境
  env_ = new EnvironmentVoxel3D();
  
  // 初始化3D体素环境
  bool success = env_->InitializeEnv(
    voxel_width_, voxel_height_, voxel_depth_,
    resolution_, origin_x_, origin_y_, origin_z_
  );
  
  if (!success) {
    // 处理初始化失败
    return;
  }

  // 创建规划器
  planner_ = new ARAPlanner(env_, true);
  
  // 获取内部octomap引用
  octomap_ = env_->GetOctomap();
}

void UAV_Sbpl_Planner::updateMapFromScan(const sensor_msgs::msg::LaserScan &scan, 
                                        const geometry_msgs::msg::PoseStamped &sensor_pose)
{
  // 将激光扫描转换为点云
  octomap::Pointcloud pointcloud;
  laserScanToPointCloud(scan, sensor_pose, pointcloud);
  
  // 获取传感器位置
  octomap::point3d sensor_origin(
    sensor_pose.pose.position.x,
    sensor_pose.pose.position.y,
    sensor_pose.pose.position.z
  );
  
  // 直接插入到环境的octomap中
  if (env_) {
    env_->InsertPointCloud(pointcloud, sensor_origin, scan.range_max);
  }
}

void UAV_Sbpl_Planner::updateMapFromPointCloud(const sensor_msgs::msg::PointCloud2 &pointcloud_msg,
                                              const geometry_msgs::msg::PoseStamped &sensor_pose)
{
  // 将ROS点云消息转换为octomap点云
  octomap::Pointcloud pointcloud;
  
  // 这里需要实现点云转换逻辑
  // 为了简化，我们暂时跳过具体实现
  
  // 获取传感器位置
  octomap::point3d sensor_origin(
    sensor_pose.pose.position.x,
    sensor_pose.pose.position.y,
    sensor_pose.pose.position.z
  );
  
  // 直接插入到环境的octomap中
  if (env_) {
    env_->InsertPointCloud(pointcloud, sensor_origin);
  }
}

void UAV_Sbpl_Planner::laserScanToPointCloud(const sensor_msgs::msg::LaserScan &scan,
                                            const geometry_msgs::msg::PoseStamped &sensor_pose,
                                            octomap::Pointcloud &pointcloud)
{
  // 获取传感器姿态
  tf2::Transform sensor_transform;
  tf2::fromMsg(sensor_pose.pose, sensor_transform);
  
  // 清空点云
  pointcloud.clear();
  
  // 遍历激光扫描数据
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];
    
    // 跳过无效范围
    if (range < scan.range_min || range > scan.range_max || std::isnan(range)) {
      continue;
    }
    
    // 计算角度
    float angle = scan.angle_min + i * scan.angle_increment;
    
    // 计算局部坐标
    float local_x = range * cos(angle);
    float local_y = range * sin(angle);
    float local_z = 0.0;  // 假设激光扫描在XY平面
    
    // 转换到世界坐标
    tf2::Vector3 local_point(local_x, local_y, local_z);
    tf2::Vector3 world_point = sensor_transform * local_point;
    
    // 添加到点云
    pointcloud.push_back(world_point.x(), world_point.y(), world_point.z());
  }
}


bool UAV_Sbpl_Planner::worldToVoxel(double world_x, double world_y, double world_z, 
                                    int &voxel_x, int &voxel_y, int &voxel_z)
{
  voxel_x = static_cast<int>((world_x - origin_x_) / resolution_);
  voxel_y = static_cast<int>((world_y - origin_y_) / resolution_);
  voxel_z = static_cast<int>((world_z - origin_z_) / resolution_);
  
  return voxel_x >= 0 && voxel_x < voxel_width_ && 
         voxel_y >= 0 && voxel_y < voxel_height_ && 
         voxel_z >= 0 && voxel_z < voxel_depth_;
}

void UAV_Sbpl_Planner::voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                                    double &world_x, double &world_y, double &world_z)
{
  world_x = origin_x_ + voxel_x * resolution_;
  world_y = origin_y_ + voxel_y * resolution_;
  world_z = origin_z_ + voxel_z * resolution_;
}

bool UAV_Sbpl_Planner::isOccupied(double x, double y, double z) const
{
  if (!octomap_) {
    return false;
  }
  
  octomap::OcTreeNode* node = octomap_->search(x, y, z);
  if (node) {
    return octomap_->isNodeOccupied(node);
  }
  return false;
}

bool UAV_Sbpl_Planner::isFree(double x, double y, double z) const
{
  if (!octomap_) {
    return true;
  }
  
  octomap::OcTreeNode* node = octomap_->search(x, y, z);
  if (node) {
    return !octomap_->isNodeOccupied(node);
  }
  return true;  // 未知区域假设为自由
}

bool UAV_Sbpl_Planner::planPath(
    double start_x, double start_y, double start_z,
    double goal_x, double goal_y, double goal_z,
    std::vector<std::pair<double, double>> &xy_path,
    std::vector<double> &z_path,
    double max_time)
{
  // 检查起点和终点是否被占用
  if (isOccupied(start_x, start_y, start_z) || isOccupied(goal_x, goal_y, goal_z)) {
    return false;
  }
  
  // 转换为3D体素坐标进行规划
  int start_voxel_x, start_voxel_y, start_voxel_z;
  int goal_voxel_x, goal_voxel_y, goal_voxel_z;
  
  if (!worldToVoxel(start_x, start_y, start_z, start_voxel_x, start_voxel_y, start_voxel_z) ||
      !worldToVoxel(goal_x, goal_y, goal_z, goal_voxel_x, goal_voxel_y, goal_voxel_z)) {
    return false;
  }
  
  // 设置起点和终点
  int start_id = env_->SetStart(start_voxel_x, start_voxel_y, start_voxel_z);
  int goal_id = env_->SetGoal(goal_voxel_x, goal_voxel_y, goal_voxel_z);
  
  if (start_id < 0 || goal_id < 0) {
    return false;
  }
  
  // 规划路径
  std::vector<int> solution_state_ids;
  int result = planner_->replan(max_time, &solution_state_ids);
  
  if (result) {
    // 转换状态ID为3D坐标
    xy_path.clear();
    z_path.clear();
    
    for (int id : solution_state_ids) {
      int x, y, z;
      env_->GetCoordFromState(id, x, y, z);
      
      // 转换为世界坐标
      double world_x, world_y, world_z;
      voxelToWorld(x, y, z, world_x, world_y, world_z);
      
      xy_path.emplace_back(world_x, world_y);
      z_path.push_back(world_z);
    }
    
    return true;
  }
  
  return false;
}

bool UAV_Sbpl_Planner::planPath(
    int start_x, int start_y, int goal_x, int goal_y,
    std::vector<std::pair<int, int>> &grid_path,
    double max_time)
{
  // 对于2D路径规划，我们在中间高度进行规划
  int start_z = voxel_depth_ / 2;  // 使用中间高度
  int goal_z = voxel_depth_ / 2;
  
  // 设置起点和终点
  int start_id = env_->SetStart(start_x, start_y, start_z);
  int goal_id = env_->SetGoal(goal_x, goal_y, goal_z);
  
  if (start_id < 0 || goal_id < 0) {
    return false;
  }

  // 规划
  std::vector<int> solution_state_ids;
  int result = planner_->replan(max_time, &solution_state_ids);

  if (result) {
    // 转换状态ID为网格坐标（忽略Z坐标）
    grid_path.clear();
    for (int id : solution_state_ids) {
      int x, y, z;
      env_->GetCoordFromState(id, x, y, z);
      grid_path.emplace_back(x, y);
    }
    return true;
  }
  return false;
}