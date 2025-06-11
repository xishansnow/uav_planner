#pragma once

#include <vector>
// #include <sensor_msgs/msg/laser_scan.hpp>

class EnvironmentNAVXYTHETALAT;
class ARAPlanner;

class UAV_Sbpl_Planner
{
public:
  UAV_Sbpl_Planner(int width, int height, double resolution, double origin_x, double origin_y);
  ~UAV_Sbpl_Planner();

  // void updateMapFromScan(const sensor_msgs::msg::LaserScan &scan);

  bool planPath(
    int start_x, int start_y, int goal_x, int goal_y,
    std::vector<std::pair<int, int>> &grid_path,
    double max_time);

private:
  void initializeSBPLEnvironment();

  int width_, height_;
  double resolution_, origin_x_, origin_y_;
  std::vector<unsigned char> occupancy_grid_;

  EnvironmentNAVXYTHETALAT *env_{nullptr};
  ARAPlanner *planner_{nullptr};
};