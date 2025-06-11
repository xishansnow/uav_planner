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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/lifecycle_node.hpp>

#include "sbpl_planner.hpp"

class SBPLPathPlannerNode : public nav2_util::LifecycleNode
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
  SBPLPathPlannerNode()
  : nav2_util::LifecycleNode("sbpl_path_planner_node", "")
  {
    RCLCPP_INFO(this->get_logger(), "SBPL Path Planner Node created");
  }

  ~SBPLPathPlannerNode()
  {
    RCLCPP_INFO(this->get_logger(), "SBPL Path Planner Node destroyed");
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(this->get_logger(), "Configuring SBPL Path Planner Node");

    // Declare parameters
    this->declare_parameter("planner_frequency", 1.0);
    this->declare_parameter("max_planning_time", 5.0);
    this->declare_parameter("goal_tolerance", 0.5);
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("map_resolution", 0.1);
    this->declare_parameter("origin_x", -5.0);
    this->declare_parameter("origin_y", -5.0);
    this->declare_parameter("origin_z", 0.0);

    // Get parameters
    planner_frequency_ = this->get_parameter("planner_frequency").as_double();
    max_planning_time_ = this->get_parameter("max_planning_time").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    map_resolution_ = this->get_parameter("map_resolution").as_double();
    origin_x_ = this->get_parameter("origin_x").as_double();
    origin_y_ = this->get_parameter("origin_y").as_double();
    origin_z_ = this->get_parameter("origin_z").as_double();

    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize SBPL planner with octomap support
    sbpl_planner_ = std::make_unique<UAV_Sbpl_Planner>(
      map_resolution_, origin_x_, origin_y_, origin_z_);

    // Create action server
    this->action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this,
      "compute_path_to_pose",
      std::bind(&SBPLPathPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SBPLPathPlannerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&SBPLPathPlannerNode::handle_accepted, this, std::placeholders::_1));

    // Create publishers and subscribers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, std::bind(&SBPLPathPlannerNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&SBPLPathPlannerNode::scanCallback, this, std::placeholders::_1));

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", 10, std::bind(&SBPLPathPlannerNode::pointcloudCallback, this, std::placeholders::_1));

    // Create timer for periodic planning
    planning_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / planner_frequency_),
      std::bind(&SBPLPathPlannerNode::planningCallback, this));

    RCLCPP_INFO(this->get_logger(), "SBPL Path Planner Node configured successfully");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(this->get_logger(), "Activating SBPL Path Planner Node");
    path_pub_->on_activate();
    cmd_vel_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating SBPL Path Planner Node");
    path_pub_->on_deactivate();
    cmd_vel_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up SBPL Path Planner Node");
    tf_buffer_.reset();
    tf_listener_.reset();
    sbpl_planner_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_action::Server<ComputePathToPose>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<UAV_Sbpl_Planner> sbpl_planner_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // Parameters
  double planner_frequency_;
  double max_planning_time_;
  double goal_tolerance_;
  double robot_radius_;
  double map_resolution_;
  double origin_x_;
  double origin_y_;
  double origin_z_;

  // Current robot state
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::Path current_path_;
  bool has_goal_ = false;
  geometry_msgs::msg::PoseStamped goal_pose_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePathToPose::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    std::thread{std::bind(&SBPLPathPlannerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleComputePathToPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputePathToPose::Feedback>();
    auto result = std::make_shared<ComputePathToPose::Result>();

    // Set goal
    goal_pose_ = goal->goal;
    has_goal_ = true;

    // Plan path using 3D planning
    nav_msgs::msg::Path path;
    if (planPath3D(current_pose_, goal_pose_, path)) {
      result->path = path;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "3D Path planning completed successfully");
    } else {
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "3D Path planning failed");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 获取传感器姿态
    geometry_msgs::msg::PoseStamped sensor_pose;
    sensor_pose.header = msg->header;
    sensor_pose.pose = current_pose_.pose;  // 假设传感器在机器人中心
    
    // 更新octomap
    if (sbpl_planner_) {
      sbpl_planner_->updateMapFromScan(*msg, sensor_pose);
    }
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 获取传感器姿态
    geometry_msgs::msg::PoseStamped sensor_pose;
    sensor_pose.header = msg->header;
    sensor_pose.pose = current_pose_.pose;  // 假设传感器在机器人中心
    
    // 更新octomap
    if (sbpl_planner_) {
      sbpl_planner_->updateMapFromPointCloud(*msg, sensor_pose);
    }
  }

  void planningCallback()
  {
    if (has_goal_ && sbpl_planner_) {
      nav_msgs::msg::Path path;
      if (planPath3D(current_pose_, goal_pose_, path)) {
        current_path_ = path;
        path_pub_->publish(path);
        
        // Simple path following
        if (!path.poses.empty()) {
          followPath(path);
        }
      }
    }
  }

  bool planPath3D(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path)
  {
    if (!sbpl_planner_) {
      return false;
    }

    // 使用3D路径规划
    std::vector<std::pair<double, double>> xy_path;
    std::vector<double> z_path;
    
    if (sbpl_planner_->planPath(
        start.pose.position.x, start.pose.position.y, start.pose.position.z,
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        xy_path, z_path, max_planning_time_)) {
      
      // 转换为ROS路径消息
      path.header.frame_id = "map";
      path.header.stamp = this->now();
      path.poses.clear();

      for (size_t i = 0; i < xy_path.size(); ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = xy_path[i].first;
        pose.pose.position.y = xy_path[i].second;
        pose.pose.position.z = z_path[i];
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }

      return true;
    }

    return false;
  }

  bool planPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path)
  {
    if (!sbpl_planner_) {
      return false;
    }

    // Convert poses to grid coordinates
    int start_x, start_y, goal_x, goal_y;
    if (!worldToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
        !worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
      return false;
    }

    // Plan path using SBPL
    std::vector<std::pair<int, int>> grid_path;
    if (sbpl_planner_->planPath(start_x, start_y, goal_x, goal_y, grid_path, max_planning_time_)) {
      // Convert grid path back to world coordinates
      path.header.frame_id = "map";
      path.header.stamp = this->now();
      path.poses.clear();

      for (const auto & grid_point : grid_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        double world_x, world_y;
        gridToWorld(grid_point.first, grid_point.second, world_x, world_y);
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }

      return true;
    }

    return false;
  }

  void followPath(const nav_msgs::msg::Path & path)
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

  bool worldToGrid(double world_x, double world_y, int & grid_x, int & grid_y)
  {
    grid_x = static_cast<int>((world_x - origin_x_) / map_resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / map_resolution_);
    
    return grid_x >= 0 && grid_x < 1000 && grid_y >= 0 && grid_y < 1000;
  }

  void gridToWorld(int grid_x, int grid_y, double & world_x, double & world_y)
  {
    world_x = origin_x_ + grid_x * map_resolution_;
    world_y = origin_y_ + grid_y * map_resolution_;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SBPLPathPlannerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
} 