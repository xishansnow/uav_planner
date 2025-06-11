/*
 * Global Path Planner Node
 * ROS2 node for global path planning using different algorithms
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "planner_factory.hpp"
#include "environment_voxel3d.hpp"

class GlobalPathPlannerNode : public rclcpp::Node
{
public:
    GlobalPathPlannerNode() : Node("global_path_planner_node")
    {
        // Declare parameters
        this->declare_parameter("planner_type", "astar");
        this->declare_parameter("octomap_file", "");
        this->declare_parameter("environment_file", "");
        this->declare_parameter("max_planning_time", 5.0);
        this->declare_parameter("publish_path", true);
        this->declare_parameter("publish_markers", true);
        
        // Get parameters
        std::string planner_type = this->get_parameter("planner_type").as_string();
        std::string octomap_file = this->get_parameter("octomap_file").as_string();
        std::string environment_file = this->get_parameter("environment_file").as_string();
        max_planning_time_ = this->get_parameter("max_planning_time").as_double();
        publish_path_ = this->get_parameter("publish_path").as_bool();
        publish_markers_ = this->get_parameter("publish_markers").as_bool();
        
        // Initialize environment
        env_ = std::make_unique<EnvironmentVoxel3D>();
        
        if (!octomap_file.empty() && !environment_file.empty()) {
            if (!env_->loadFromFiles(octomap_file, environment_file)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load environment from files");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Loaded environment from %s and %s", 
                       octomap_file.c_str(), environment_file.c_str());
        } else {
            // Create a simple test environment
            createTestEnvironment();
            RCLCPP_INFO(this->get_logger(), "Created test environment");
        }
        
        // Create planner
        try {
            planner_ = PlannerFactory::createPlanner(planner_type, env_.get());
            RCLCPP_INFO(this->get_logger(), "Created %s planner", planner_type.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create planner: %s", e.what());
            return;
        }
        
        // Create publishers
        if (publish_path_) {
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
        }
        
        if (publish_markers_) {
            markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("path_markers", 10);
        }
        
        // Create subscribers
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&GlobalPathPlannerNode::goalCallback, this, std::placeholders::_1));
        
        // Create services
        plan_service_ = this->create_service<std_srvs::srv::Trigger>(
            "plan_path",
            std::bind(&GlobalPathPlannerNode::planPathService, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        // Create action server (if needed)
        // TODO: Add action server for long-running planning tasks
        
        // Initialize current pose
        current_pose_.pose.position.x = 0.0;
        current_pose_.pose.position.y = 0.0;
        current_pose_.pose.position.z = 0.0;
        current_pose_.pose.orientation.w = 1.0;
        
        RCLCPP_INFO(this->get_logger(), "Global Path Planner Node initialized");
    }
    
private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f, %.2f)", 
                   goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
        
        // Plan path immediately
        planPath();
    }
    
    bool planPathService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused
        
        bool success = planPath();
        response->success = success;
        
        if (success) {
            response->message = "Path planning successful";
        } else {
            response->message = "Path planning failed";
        }
        
        return true;
    }
    
    bool planPath()
    {
        if (!planner_) {
            RCLCPP_ERROR(this->get_logger(), "No planner available");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Planning path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                   current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z,
                   goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
        
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        bool success = planner_->planPath(
            current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z,
            goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z,
            path, max_planning_time_);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Path planning successful in %ld ms, path length: %zu", 
                       duration.count(), path.size());
            
            // Publish path
            if (publish_path_) {
                publishPath(path);
            }
            
            // Publish markers
            if (publish_markers_) {
                publishMarkers(path);
            }
            
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Path planning failed after %ld ms", duration.count());
            return false;
        }
    }
    
    void publishPath(const std::vector<geometry_msgs::msg::PoseStamped>& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        path_msg.poses = path;
        
        path_pub_->publish(path_msg);
    }
    
    void publishMarkers(const std::vector<geometry_msgs::msg::PoseStamped>& path)
    {
        visualization_msgs::msg::MarkerArray markers;
        
        // Path line marker
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->now();
        line_marker.ns = "path";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        line_marker.scale.x = 0.1;  // Line width
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 1.0;
        
        for (const auto& pose : path) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            line_marker.points.push_back(point);
        }
        
        markers.markers.push_back(line_marker);
        
        // Start and goal markers
        visualization_msgs::msg::Marker start_marker;
        start_marker.header.frame_id = "map";
        start_marker.header.stamp = this->now();
        start_marker.ns = "poses";
        start_marker.id = 1;
        start_marker.type = visualization_msgs::msg::Marker::SPHERE;
        start_marker.action = visualization_msgs::msg::Marker::ADD;
        start_marker.pose = current_pose_.pose;
        start_marker.scale.x = 0.3;
        start_marker.scale.y = 0.3;
        start_marker.scale.z = 0.3;
        start_marker.color.r = 0.0;
        start_marker.color.g = 0.0;
        start_marker.color.b = 1.0;
        start_marker.color.a = 1.0;
        
        visualization_msgs::msg::Marker goal_marker;
        goal_marker.header.frame_id = "map";
        goal_marker.header.stamp = this->now();
        goal_marker.ns = "poses";
        goal_marker.id = 2;
        goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
        goal_marker.action = visualization_msgs::msg::Marker::ADD;
        goal_marker.pose = goal_pose_.pose;
        goal_marker.scale.x = 0.3;
        goal_marker.scale.y = 0.3;
        goal_marker.scale.z = 0.3;
        goal_marker.color.r = 1.0;
        goal_marker.color.g = 0.0;
        goal_marker.color.b = 0.0;
        goal_marker.color.a = 1.0;
        
        markers.markers.push_back(start_marker);
        markers.markers.push_back(goal_marker);
        
        markers_pub_->publish(markers);
    }
    
    void createTestEnvironment()
    {
        // Create a simple 3D environment with some obstacles
        double resolution = 0.1;
        double min_x = -5.0, max_x = 5.0;
        double min_y = -5.0, max_y = 5.0;
        double min_z = 0.0, max_z = 3.0;
        
        env_->initialize(resolution, min_x, min_y, min_z, max_x, max_y, max_z);
        
        // Add some obstacles
        for (double x = -2.0; x <= 2.0; x += resolution) {
            for (double y = -2.0; y <= 2.0; y += resolution) {
                for (double z = 0.0; z <= 1.5; z += resolution) {
                    env_->setOccupied(x, y, z, true);
                }
            }
        }
        
        // Add a wall
        for (double x = 1.0; x <= 1.5; x += resolution) {
            for (double y = -3.0; y <= 3.0; y += resolution) {
                for (double z = 0.0; z <= 2.0; z += resolution) {
                    env_->setOccupied(x, y, z, true);
                }
            }
        }
    }
    
    // Member variables
    std::unique_ptr<EnvironmentVoxel3D> env_;
    std::shared_ptr<GlobalPlannerBase> planner_;
    
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    
    double max_planning_time_;
    bool publish_path_;
    bool publish_markers_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GlobalPathPlannerNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 