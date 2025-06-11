#pragma once

#include <vector>
#include <memory>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 前向声明
class EnvironmentVoxel3D;  // 使用3D体素环境
class ARAPlanner;

class UAV_Sbpl_Planner
{
public:
  UAV_Sbpl_Planner(double resolution, double origin_x, double origin_y, double origin_z);
  ~UAV_Sbpl_Planner();

  // 更新octomap从激光扫描数据
  void updateMapFromScan(const sensor_msgs::msg::LaserScan &scan, 
                        const geometry_msgs::msg::PoseStamped &sensor_pose);
  
  // 更新octomap从点云数据
  void updateMapFromPointCloud(const sensor_msgs::msg::PointCloud2 &pointcloud,
                              const geometry_msgs::msg::PoseStamped &sensor_pose);

  // 3D路径规划（真正的3D体素规划）
  bool planPath(
    double start_x, double start_y, double start_z,
    double goal_x, double goal_y, double goal_z,
    std::vector<std::pair<double, double>> &xy_path,
    std::vector<double> &z_path,
    double max_time);

  // 2D路径规划（保持兼容性）
  bool planPath(
    int start_x, int start_y, int goal_x, int goal_y,
    std::vector<std::pair<int, int>> &grid_path,
    double max_time);

  // 获取octomap
  std::shared_ptr<octomap::OcTree> getOctomap() const { return octomap_; }
  
  // 检查3D点是否被占用
  bool isOccupied(double x, double y, double z) const;
  
  // 检查3D点是否空闲
  bool isFree(double x, double y, double z) const;

private:
  void initializeSBPLEnvironment();
  
  // 将3D坐标转换为体素坐标
  bool worldToVoxel(double world_x, double world_y, double world_z, 
                    int &voxel_x, int &voxel_y, int &voxel_z);
  void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                    double &world_x, double &world_y, double &world_z);
  
  // 将激光扫描转换为点云
  void laserScanToPointCloud(const sensor_msgs::msg::LaserScan &scan,
                            const geometry_msgs::msg::PoseStamped &sensor_pose,
                            octomap::Pointcloud &pointcloud);

  // 环境参数
  double resolution_;
  double origin_x_, origin_y_, origin_z_;
  int voxel_width_, voxel_height_, voxel_depth_;  // 3D体素网格尺寸
  
  // Octomap（从环境内部获取）
  std::shared_ptr<octomap::OcTree> octomap_;
  
  // SBPL 3D体素环境
  EnvironmentVoxel3D *env_{nullptr};
  ARAPlanner *planner_{nullptr};
};