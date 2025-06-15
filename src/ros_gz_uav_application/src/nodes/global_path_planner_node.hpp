#ifndef GLOBAL_PATH_PLANNER_NODE_HPP
#define GLOBAL_PATH_PLANNER_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_util/robot_utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include "single_global/global_planner_base.hpp"
#include "single_global/planner_factory.hpp"
#include "env/environment_voxel3d.hpp"

class GlobalPathPlannerNode : public rclcpp::Node
{
public:
  GlobalPathPlannerNode();
  ~GlobalPathPlannerNode();

private:
  // 从 YAML 配置文件加载参数
  bool loadParametersFromYaml(const std::string& config_file_path);
  
  // 规划路径
  bool planPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path);
  //根据路径推算当前点，向仿真器发送跟随路径的命令
  void followPath(const nav_msgs::msg::Path & path);

  // Visualization functions
  void publishVisualization();
  void publishEnvironmentBounds(visualization_msgs::msg::MarkerArray& marker_array);
  void publishVoxelGrid(visualization_msgs::msg::MarkerArray& marker_array);
  void publishStartGoalPoints(visualization_msgs::msg::MarkerArray& marker_array);
  void publishPaths(visualization_msgs::msg::MarkerArray& marker_array);
  void publishOctomap();
  void publishMapTransform();

  // Member variables
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<EnvironmentVoxel3D> environment_;
  std::shared_ptr<GlobalPlannerBase> planner_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // Visualization publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_bounds_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr start_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr paths_pub_;
  
  // Octomap publishers
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_octomap_pub_;

  // Parameters
  std::string planner_type_;
  double planner_frequency_;
  double max_planning_time_;
  double goal_tolerance_;
  double robot_radius_;
  double map_resolution_;
  double origin_x_;
  double origin_y_;
  double origin_z_;
  
  // Environment parameters
  double env_width_;
  double env_height_;
  double env_depth_;
  int voxel_width_;
  int voxel_height_;
  int voxel_depth_;
  double voxel_size_xy_;
  double voxel_size_z_;

  // Current robot state
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path current_path_;
  bool has_goal_ = false;
  geometry_msgs::msg::PoseStamped goal_pose_;
};

#endif // GLOBAL_PATH_PLANNER_NODE_HPP 