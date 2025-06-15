#include <memory>
#include <string>
#include <vector>
#include <fstream>

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
#include <yaml-cpp/yaml.h>

#include "single_global/global_planner_base.hpp"
#include "single_global/planner_factory.hpp"
#include "env/environment_voxel3d.hpp"
#include "global_path_planner_node.hpp"

GlobalPathPlannerNode::GlobalPathPlannerNode()
: rclcpp::Node("global_path_planner_node")
{
  RCLCPP_INFO(this->get_logger(), "Global Path Planner Node created");

  // 声明参数（用于命令行覆盖）
  this->declare_parameter("config_file", "");
  this->declare_parameter("planner_type", "astar");
  this->declare_parameter("planner_frequency", 1.0);
  this->declare_parameter("max_planning_time", 5.0);
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("robot_radius", 0.3);
  this->declare_parameter("map_resolution", 0.1);
  this->declare_parameter("origin_x", -5.0);
  this->declare_parameter("origin_y", -5.0);
  this->declare_parameter("origin_z", 0.0);
  this->declare_parameter("env_width", 200.0);
  this->declare_parameter("env_height", 200.0);
  this->declare_parameter("env_depth", 100.0);
  this->declare_parameter("voxel_width", 200);
  this->declare_parameter("voxel_height", 200);
  this->declare_parameter("voxel_depth", 100);

  // 尝试从 YAML 文件加载参数
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    if (loadParametersFromYaml(config_file)) {
      RCLCPP_INFO(this->get_logger(), "Successfully loaded parameters from YAML file: %s", config_file.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to load parameters from YAML file: %s, using default values", config_file.c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "No config file specified, using default parameters");
  }

  // 获取参数（优先使用 YAML 文件中的值，如果没有则使用默认值）
  planner_type_ = this->get_parameter("planner_type").as_string();
  planner_frequency_ = this->get_parameter("planner_frequency").as_double();
  max_planning_time_ = this->get_parameter("max_planning_time").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  robot_radius_ = this->get_parameter("robot_radius").as_double();
  map_resolution_ = this->get_parameter("map_resolution").as_double();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  origin_z_ = this->get_parameter("origin_z").as_double();
  
  // Get environment parameters
  env_width_ = this->get_parameter("env_width").as_double();
  env_height_ = this->get_parameter("env_height").as_double();
  env_depth_ = this->get_parameter("env_depth").as_double();
  voxel_width_ = this->get_parameter("voxel_width").as_int();
  voxel_height_ = this->get_parameter("voxel_height").as_int();
  voxel_depth_ = this->get_parameter("voxel_depth").as_int();
  
  // Calculate resolutions
  voxel_size_xy_ = env_width_ / voxel_width_;
  voxel_size_z_ = env_depth_ / voxel_depth_;

  // Initialize TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Initialize environment
  environment_ = std::make_unique<EnvironmentVoxel3D>();
  bool env_success = environment_->InitializeEnv(
      env_width_, env_height_, env_depth_,
      voxel_size_xy_, voxel_size_z_,
      origin_x_, origin_y_, origin_z_);
  
  if (!env_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize environment!");
    return;
  }

  // Initialize planner using factory
  PlannerFactory::PlannerType type = PlannerFactory::ASTAR;
  if (planner_type_ == "thetastar") {
    type = PlannerFactory::THETASTAR;
  } else if (planner_type_ == "arastar") {
    type = PlannerFactory::ARASTAR;
  } else if (planner_type_ == "jps") {
    type = PlannerFactory::JPS;
  }
  
  planner_ = PlannerFactory::createPlanner(type, environment_.get());
  if (!planner_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create planner of type: %s", planner_type_.c_str());
    return;
  }

  // Create publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Initialize visualization publishers
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_markers", 10);
  environment_bounds_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("environment_bounds", 10);
  obstacles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacles", 10);
  voxel_grid_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("voxel_grid", 10);
  start_goal_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("start_goal_points", 10);
  paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("paths", 10);

  // Initialize octomap publishers
  octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
  binary_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("binary_octomap", 10);

  RCLCPP_INFO(this->get_logger(), "Global Path Planner Node configured successfully with planner type: %s", planner_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "Environment setup:");
  RCLCPP_INFO(this->get_logger(), "  World size: %.1f x %.1f x %.1f m", env_width_, env_height_, env_depth_);
  RCLCPP_INFO(this->get_logger(), "  Voxel grid: %d x %d x %d", voxel_width_, voxel_height_, voxel_depth_);
  RCLCPP_INFO(this->get_logger(), "  XY resolution: %.1f m", voxel_size_xy_);
  RCLCPP_INFO(this->get_logger(), "  Z resolution: %.1f m", voxel_size_z_);
}

GlobalPathPlannerNode::~GlobalPathPlannerNode()
{
  RCLCPP_INFO(this->get_logger(), "Global Path Planner Node destroyed");
}

bool GlobalPathPlannerNode::loadParametersFromYaml(const std::string& config_file_path)
{
  try {
    // 检查文件是否存在
    std::ifstream file(config_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open config file: %s", config_file_path.c_str());
      return false;
    }
    file.close();

    // 加载 YAML 文件
    YAML::Node config = YAML::LoadFile(config_file_path);
    
    // 检查是否存在节点配置
    if (!config["global_path_planner_node"]) {
      RCLCPP_ERROR(this->get_logger(), "Config file does not contain 'global_path_planner_node' section");
      return false;
    }

    YAML::Node params = config["global_path_planner_node"]["ros__parameters"];
    if (!params) {
      RCLCPP_ERROR(this->get_logger(), "Config file does not contain 'ros__parameters' section");
      return false;
    }

    // 读取参数并设置到 ROS 参数服务器
    if (params["planner_type"]) {
      this->set_parameter(rclcpp::Parameter("planner_type", params["planner_type"].as<std::string>()));
    }
    if (params["planner_frequency"]) {
      this->set_parameter(rclcpp::Parameter("planner_frequency", params["planner_frequency"].as<double>()));
    }
    if (params["max_planning_time"]) {
      this->set_parameter(rclcpp::Parameter("max_planning_time", params["max_planning_time"].as<double>()));
    }
    if (params["goal_tolerance"]) {
      this->set_parameter(rclcpp::Parameter("goal_tolerance", params["goal_tolerance"].as<double>()));
    }
    if (params["robot_radius"]) {
      this->set_parameter(rclcpp::Parameter("robot_radius", params["robot_radius"].as<double>()));
    }
    if (params["map_resolution"]) {
      this->set_parameter(rclcpp::Parameter("map_resolution", params["map_resolution"].as<double>()));
    }
    if (params["origin_x"]) {
      this->set_parameter(rclcpp::Parameter("origin_x", params["origin_x"].as<double>()));
    }
    if (params["origin_y"]) {
      this->set_parameter(rclcpp::Parameter("origin_y", params["origin_y"].as<double>()));
    }
    if (params["origin_z"]) {
      this->set_parameter(rclcpp::Parameter("origin_z", params["origin_z"].as<double>()));
    }
    if (params["env_width"]) {
      this->set_parameter(rclcpp::Parameter("env_width", params["env_width"].as<double>()));
    }
    if (params["env_height"]) {
      this->set_parameter(rclcpp::Parameter("env_height", params["env_height"].as<double>()));
    }
    if (params["env_depth"]) {
      this->set_parameter(rclcpp::Parameter("env_depth", params["env_depth"].as<double>()));
    }
    if (params["voxel_width"]) {
      this->set_parameter(rclcpp::Parameter("voxel_width", params["voxel_width"].as<int>()));
    }
    if (params["voxel_height"]) {
      this->set_parameter(rclcpp::Parameter("voxel_height", params["voxel_height"].as<int>()));
    }
    if (params["voxel_depth"]) {
      this->set_parameter(rclcpp::Parameter("voxel_depth", params["voxel_depth"].as<int>()));
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu parameters from YAML file", params.size());
    return true;

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading YAML config: %s", e.what());
    return false;
  }
}

bool GlobalPathPlannerNode::planPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  if (!planner_) {
    return false;
  }

  // 使用规划器进行路径规划
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  if (planner_->planPath(
      start.pose.position.x, start.pose.position.y, start.pose.position.z,
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
      poses, SmootherType::BEZIER, max_planning_time_)) {
    
    // 转换为ROS路径消息
    path.header.frame_id = "map";
    path.header.stamp = this->now();
    path.poses = poses;

    return true;
  }

  return false;
}

void GlobalPathPlannerNode::followPath(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    return;
  }

  // Simple path following: move towards the next waypoint
  geometry_msgs::msg::PoseStamped next_waypoint = path.poses[0];
  
  // Calculate distance and angle to next waypoint
  double dx = next_waypoint.pose.position.x - current_pose_.pose.position.x;
  double dy = next_waypoint.pose.position.y - current_pose_.pose.position.y;
  double dz = next_waypoint.pose.position.z - current_pose_.pose.position.z;
  double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  double angle = std::atan2(dy, dx);

  // Simple proportional control
  geometry_msgs::msg::Twist cmd_vel;
  if (distance > goal_tolerance_) {
    cmd_vel.linear.x = std::min(0.5, distance * 0.5);  // Proportional linear velocity
    cmd_vel.angular.z = angle * 1.0;  // Proportional angular velocity
    cmd_vel.linear.z = dz * 0.5;  // Vertical velocity for 3D movement
  } else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel.linear.z = 0.0;
  }

  cmd_vel_pub_->publish(cmd_vel);
}

void GlobalPathPlannerNode::publishVisualization()
{
  // Publish environment bounds
  auto bounds_array = visualization_msgs::msg::MarkerArray();
  publishEnvironmentBounds(bounds_array);
  environment_bounds_pub_->publish(bounds_array);
  
  // Publish voxel grid
  auto voxel_array = visualization_msgs::msg::MarkerArray();
  publishVoxelGrid(voxel_array);
  voxel_grid_pub_->publish(voxel_array);
  
  // Publish start-goal points
  auto start_goal_array = visualization_msgs::msg::MarkerArray();
  publishStartGoalPoints(start_goal_array);
  start_goal_pub_->publish(start_goal_array);
  
  // Publish paths
  auto paths_array = visualization_msgs::msg::MarkerArray();
  publishPaths(paths_array);
  paths_pub_->publish(paths_array);
  
  // Also publish everything to the original topic for backward compatibility
  auto combined_array = visualization_msgs::msg::MarkerArray();
  publishEnvironmentBounds(combined_array);
  publishVoxelGrid(combined_array);
  publishStartGoalPoints(combined_array);
  publishPaths(combined_array);
  marker_pub_->publish(combined_array);
  
  RCLCPP_INFO(this->get_logger(), "Published visualization markers:");
  RCLCPP_INFO(this->get_logger(), "  - Environment bounds: %zu markers", bounds_array.markers.size());
  RCLCPP_INFO(this->get_logger(), "  - Voxel grid: %zu markers", voxel_array.markers.size());
  RCLCPP_INFO(this->get_logger(), "  - Start/Goal points: %zu markers", start_goal_array.markers.size());
  RCLCPP_INFO(this->get_logger(), "  - Paths: %zu markers", paths_array.markers.size());
  RCLCPP_INFO(this->get_logger(), "  - Combined: %zu markers", combined_array.markers.size());
}

void GlobalPathPlannerNode::publishEnvironmentBounds(visualization_msgs::msg::MarkerArray& marker_array)
{
  // Create environment bounds marker
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "world";
  marker.header.stamp = this->now();
  marker.ns = "environment_bounds";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Set position and scale
  marker.pose.position.x = origin_x_ + env_width_ / 2.0;
  marker.pose.position.y = origin_y_ + env_height_ / 2.0;
  marker.pose.position.z = origin_z_ + env_depth_ / 2.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = env_width_;
  marker.scale.y = env_height_;
  marker.scale.z = env_depth_;
  
  // Set color (semi-transparent gray)
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 0.1;
  
  marker_array.markers.push_back(marker);
}

void GlobalPathPlannerNode::publishVoxelGrid(visualization_msgs::msg::MarkerArray& marker_array)
{
  if (!environment_) return;
  
  // Publish occupied voxels
  int occupied_count = 0;
  for (int x = 0; x < voxel_width_; ++x) {
    for (int y = 0; y < voxel_height_; ++y) {
      for (int z = 0; z < voxel_depth_; ++z) {
        if (environment_->GetCellCost(x, y, z) > 0) {
          auto marker = visualization_msgs::msg::Marker();
          marker.header.frame_id = "world";
          marker.header.stamp = this->now();
          marker.ns = "voxel_grid";
          marker.id = occupied_count++;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          
          // Convert voxel coordinates to world coordinates
          double world_x, world_y, world_z;
          environment_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
          
          marker.pose.position.x = world_x;
          marker.pose.position.y = world_y;
          marker.pose.position.z = world_z;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = voxel_size_xy_ * 0.9;
          marker.scale.y = voxel_size_xy_ * 0.9;
          marker.scale.z = voxel_size_z_ * 0.9;
          
          // Set color (red for occupied voxels)
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
          marker.color.a = 0.6;
          
          marker_array.markers.push_back(marker);
        }
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Published %d occupied voxels", occupied_count);
}

void GlobalPathPlannerNode::publishStartGoalPoints(visualization_msgs::msg::MarkerArray& marker_array)
{
  // Publish current pose (start point)
  auto start_marker = visualization_msgs::msg::Marker();
  start_marker.header.frame_id = "world";
  start_marker.header.stamp = this->now();
  start_marker.ns = "start_goal_points";
  start_marker.id = 0;
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  
  start_marker.pose = current_pose_.pose;
  start_marker.scale.x = 2.0;
  start_marker.scale.y = 2.0;
  start_marker.scale.z = 2.0;
  
  // Green for start point
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 0.8;
  
  marker_array.markers.push_back(start_marker);
  
  // Publish goal point if available
  if (has_goal_) {
    auto goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "start_goal_points";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    
    goal_marker.pose = goal_pose_.pose;
    goal_marker.scale.x = 2.0;
    goal_marker.scale.y = 2.0;
    goal_marker.scale.z = 2.0;
    
    // Red for goal point
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 0.8;
    
    marker_array.markers.push_back(goal_marker);
  }
}

void GlobalPathPlannerNode::publishPaths(visualization_msgs::msg::MarkerArray& marker_array)
{
  if (current_path_.poses.empty()) return;
  
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "world";
  marker.header.stamp = this->now();
  marker.ns = "paths";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Set line points
  for (const auto& pose : current_path_.poses) {
    geometry_msgs::msg::Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = pose.pose.position.z;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 1.0;  // Line width
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  
  // Blue for planned path
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.8;
  
  marker_array.markers.push_back(marker);
}

void GlobalPathPlannerNode::publishOctomap()
{
  if (!environment_) return;
  
  // Convert environment to octomap
  octomap::OcTree octree(voxel_size_xy_);
  
  for (int x = 0; x < voxel_width_; ++x) {
    for (int y = 0; y < voxel_height_; ++y) {
      for (int z = 0; z < voxel_depth_; ++z) {
        if (environment_->GetCellCost(x, y, z) > 0) {
          double world_x, world_y, world_z;
          environment_->VoxelToWorld(x, y, z, world_x, world_y, world_z);
          octree.updateNode(world_x, world_y, world_z, true);
        }
      }
    }
  }
  
  octree.updateInnerOccupancy();
  
  // Convert to ROS message
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msg.header.frame_id = "world";
  octomap_msg.header.stamp = this->now();
  octomap_msg.binary = false;
  octomap_msg.id = "OcTree";
  octomap_msg.resolution = voxel_size_xy_;
  
  octomap_msgs::binaryMapToMsg(octree, octomap_msg);
  octomap_pub_->publish(octomap_msg);
  
  // Also publish binary version
  octomap_msg.binary = true;
  octomap_msgs::binaryMapToMsg(octree, octomap_msg);
  binary_octomap_pub_->publish(octomap_msg);
  
  RCLCPP_INFO(this->get_logger(), "Published octomap");
}

void GlobalPathPlannerNode::publishMapTransform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "map";
  
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  
  tf_broadcaster_->sendTransform(transform);
  RCLCPP_INFO(this->get_logger(), "Published world->map transform");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPathPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 