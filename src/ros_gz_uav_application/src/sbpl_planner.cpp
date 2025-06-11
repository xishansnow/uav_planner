#include "sbpl_planner.hpp"
#include <sbpl/headers.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>

#include <cmath>
#include <vector>
#include <memory>

UAV_Sbpl_Planner::UAV_Sbpl_Planner(int width, int height, double resolution, double origin_x, double origin_y)
: width_(width), height_(height), resolution_(resolution), origin_x_(origin_x), origin_y_(origin_y)
{
  // Initialize occupancy grid
  occupancy_grid_.resize(width * height, 0);
  
  // Initialize SBPL environment
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
  // Create SBPL environment configuration
  env_ = new EnvironmentNAVXYTHETALAT();
  
  // Set environment parameters
  env_->SetEnvParameter("cost_inscribed_thresh", 200);
  env_->SetEnvParameter("cost_possibly_circumscribed_thresh", 200);
  env_->SetEnvParameter("cost_obstacle_thresh", 200);

  // Set up the environment with a blank map
  std::vector<unsigned char> mapdata(width_ * height_, 0);
  
  // Initialize environment with required parameters
  // env_->InitializeEnv(
  //   width_,  // size_x
  //   height_, // size_y
  //   nullptr, // mapdata
  //   0, 0, 0, // start (x,y,theta)
  //   0, 0, 0, // goal (x,y,theta)
  //   0.5, // nominalvel_mpersecs
  //   0.2, // timetoturn45degsinplace_secs
  //   0.1, // cellsize_m
  //   mapdata.data(), // grid data
  //   0.4, // nominalaccvel_mpersecs
  //   0.2, // timetoturn45degsinplace_secs
  //   std::vector<sbpl_2Dpt_t>(), // perimeterptsV
  //   0.0, // gridmaxx
  //   0.0  // gridmaxy
  // );

  // Create the planner
  planner_ = new ARAPlanner(env_, true);
}

// void UAV_Sbpl_Planner::updateMapFromScan(const sensor_msgs::msg::LaserScan &scan)
// {
//   // TODO: Implement occupancy grid update from laser scan
//   // For now, this is a stub.
// }

bool UAV_Sbpl_Planner::planPath(
    int start_x, int start_y, int goal_x, int goal_y,
    std::vector<std::pair<int, int>> &grid_path,
    double max_time)
{
  // Set start and goal
  int start_id = env_->SetStart(start_x, start_y, 0);
  int goal_id = env_->SetGoal(goal_x, goal_y, 0);

  // Plan
  std::vector<int> solution_state_ids;
  int result = planner_->replan(max_time, &solution_state_ids);

  if (result) {
    // Convert state IDs to grid coordinates
    grid_path.clear();
    for (int id : solution_state_ids) {
      int x, y, theta;
      env_->GetCoordFromState(id, x, y, theta);
      grid_path.emplace_back(x, y);
    }
    return true;
  }
  return false;
}