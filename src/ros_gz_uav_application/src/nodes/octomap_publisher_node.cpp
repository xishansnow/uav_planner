/*
 * Octomap Publisher Node
 * ROS2 node for publishing octomap from EnvironmentVoxel3D to rviz2
 */

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cstdlib>
#include <ctime>

#include "env/environment_voxel3d.hpp"

class OctomapPublisherNode : public rclcpp::Node
{
public:
    OctomapPublisherNode() : Node("octomap_publisher_node")
    {
        // Initialize random seed
        std::srand(std::time(nullptr));
        
        // Declare parameters
        this->declare_parameter("octomap_file", "");
        this->declare_parameter("environment_file", "");
        this->declare_parameter("publish_frequ1ency", 1.0);
        this->declare_parameter("frame_id", "map");
        this->declare_parameter("octomap_topic", "octomap_full");
        this->declare_parameter("binary_octomap_topic", "octomap_binary");
        
        // Get parameters
        std::string octomap_file = this->get_parameter("octomap_file").as_string();
        std::string environment_file = this->get_parameter("environment_file").as_string();
        publish_frequency_ = this->get_parameter("publish_frequency").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        octomap_topic_ = this->get_parameter("octomap_topic").as_string();
        binary_octomap_topic_ = this->get_parameter("binary_octomap_topic").as_string();
        
        // Initialize environment
        env_ = std::make_unique<EnvironmentVoxel3D>();
        
        if (!octomap_file.empty() && !environment_file.empty()) {
            if (!env_->LoadFromBTFileWithConfig(octomap_file)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load environment from file");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Loaded environment from %s", 
                       octomap_file.c_str());
        } else {
            // Create a simple test environment
            createTestEnvironment();
            RCLCPP_INFO(this->get_logger(), "Created test environment");
        }
        
        // Create publishers
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(octomap_topic_, 10);
        binary_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(binary_octomap_topic_, 10);
        
        // Create static transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish static transform for map frame
        publishMapTransform();
        
        // Create timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_frequency_),
            std::bind(&OctomapPublisherNode::publishOctomap, this));
        
        RCLCPP_INFO(this->get_logger(), "Octomap publisher node initialized");
    }

private:
    void createTestEnvironment()
    {
        // Create a simple 3D environment with some obstacles
        bool success = env_->InitializeEnv(1024, 1024, 200, 0.1, 0.1, -5.0, -5.0, 0.0);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize test environment");
            return;
        }
        
        // Add a sphere obstacle (random position, radius 15)
        int sphere_center_x = 200 + (rand() % 600);  // Random position between 200-800
        int sphere_center_y = 200 + (rand() % 600);
        int sphere_center_z = 10 + (rand() % 30);    // Random height between 10-40
        int sphere_radius = 10 + (rand() % 15);      // Random radius between 10-25
        
        for (int x = sphere_center_x - sphere_radius; x <= sphere_center_x + sphere_radius; ++x) {
            for (int y = sphere_center_y - sphere_radius; y <= sphere_center_y + sphere_radius; ++y) {
                for (int z = sphere_center_z - sphere_radius; z <= sphere_center_z + sphere_radius; ++z) {
                    // Check if point is within sphere
                    int dx = x - sphere_center_x;
                    int dy = y - sphere_center_y;
                    int dz = z - sphere_center_z;
                    if (dx*dx + dy*dy + dz*dz <= sphere_radius*sphere_radius) {
                        if (x >= 0 && x < 1024 && y >= 0 && y < 1024 && z >= 0 && z < 200) {
                            env_->UpdateCellCost(x, y, z, 255); // Occupied
                        }
                    }
                }
            }
        }
        
        // Add a cylinder obstacle (random position, radius 12, height 25)
        int cylinder_center_x = 200 + (rand() % 600);
        int cylinder_center_y = 200 + (rand() % 600);
        int cylinder_center_z = 5 + (rand() % 20);    // Random base height between 5-25
        int cylinder_radius = 8 + (rand() % 12);      // Random radius between 8-20
        int cylinder_height = 15 + (rand() % 25);     // Random height between 15-40
        
        for (int x = cylinder_center_x - cylinder_radius; x <= cylinder_center_x + cylinder_radius; ++x) {
            for (int y = cylinder_center_y - cylinder_radius; y <= cylinder_center_y + cylinder_radius; ++y) {
                for (int z = cylinder_center_z; z <= cylinder_center_z + cylinder_height; ++z) {
                    // Check if point is within cylinder (XY plane circle)
                    int dx = x - cylinder_center_x;
                    int dy = y - cylinder_center_y;
                    if (dx*dx + dy*dy <= cylinder_radius*cylinder_radius) {
                        if (x >= 0 && x < 1024 && y >= 0 && y < 1024 && z >= 0 && z < 200) {
                            env_->UpdateCellCost(x, y, z, 255); // Occupied
                        }
                    }
                }
            }
        }
        
        // Add a cube obstacle (random position, side length 20)
        int cube_center_x = 200 + (rand() % 600);
        int cube_center_y = 200 + (rand() % 600);
        int cube_center_z = 5 + (rand() % 30);        // Random base height between 5-35
        int cube_side = 10 + (rand() % 20);           // Random side length between 10-30
        
        for (int x = cube_center_x - cube_side/2; x <= cube_center_x + cube_side/2; ++x) {
            for (int y = cube_center_y - cube_side/2; y <= cube_center_y + cube_side/2; ++y) {
                for (int z = cube_center_z; z <= cube_center_z + cube_side; ++z) {
                    if (x >= 0 && x < 1024 && y >= 0 && y < 1024 && z >= 0 && z < 200) {
                        env_->UpdateCellCost(x, y, z, 255); // Occupied
                    }
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Created test environment with sphere, cylinder, and cube obstacles");
    }
    
    void publishMapTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";
        transform.child_frame_id = frame_id_;
        
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "Published map transform");
    }
    
    void publishOctomap()
    {
        auto octomap = env_->GetOctomap();
        if (!octomap) {
            RCLCPP_WARN(this->get_logger(), "No octomap available");
            return;
        }
        
        // Create octomap message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = frame_id_;
        octomap_msg.header.stamp = this->now();
        octomap_msg.binary = false;
        octomap_msg.id = "OcTree";
        
        // Convert octomap to message
        if (octomap_msgs::fullMapToMsg(*octomap, octomap_msg)) {
            octomap_pub_->publish(octomap_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to message");
        }
        
        // Create binary octomap message
        octomap_msgs::msg::Octomap binary_octomap_msg;
        binary_octomap_msg.header.frame_id = frame_id_;
        binary_octomap_msg.header.stamp = this->now();
        binary_octomap_msg.binary = true;
        binary_octomap_msg.id = "OcTree";
        
        // Convert octomap to binary message
        if (octomap_msgs::binaryMapToMsg(*octomap, binary_octomap_msg)) {
            binary_octomap_pub_->publish(binary_octomap_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap to binary message");
        }
    }
    
    std::unique_ptr<EnvironmentVoxel3D> env_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_octomap_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double publish_frequency_;
    std::string frame_id_;
    std::string octomap_topic_;
    std::string binary_octomap_topic_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctomapPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 