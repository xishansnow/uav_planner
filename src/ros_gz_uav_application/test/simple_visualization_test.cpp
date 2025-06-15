#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

class SimpleVisualizationTest : public rclcpp::Node
{
public:
    SimpleVisualizationTest() : Node("simple_visualization_test")
    {
        // Create publisher
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planning_visualization", 10);
        
        RCLCPP_INFO(this->get_logger(), "Simple visualization test node started");
        
        // Publish test markers immediately
        publishTestMarkers();
        
        // Set up timer to publish periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                this->publishTestMarkers();
            }
        );
    }

private:
    void publishTestMarkers()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Test marker 1: Red sphere at origin
        auto marker1 = visualization_msgs::msg::Marker();
        marker1.header.frame_id = "map";
        marker1.header.stamp = this->now();
        marker1.ns = "test";
        marker1.id = 0;
        marker1.type = visualization_msgs::msg::Marker::SPHERE;
        marker1.action = visualization_msgs::msg::Marker::ADD;
        
        marker1.pose.position.x = 5000.0;
        marker1.pose.position.y = 5000.0;
        marker1.pose.position.z = 1000.0;
        marker1.pose.orientation.w = 1.0;
        
        marker1.scale.x = 200.0;
        marker1.scale.y = 200.0;
        marker1.scale.z = 200.0;
        
        marker1.color.r = 1.0;
        marker1.color.g = 0.0;
        marker1.color.b = 0.0;
        marker1.color.a = 1.0;
        
        marker_array.markers.push_back(marker1);
        
        // Test marker 2: Green cube
        auto marker2 = visualization_msgs::msg::Marker();
        marker2.header.frame_id = "map";
        marker2.header.stamp = this->now();
        marker2.ns = "test";
        marker2.id = 1;
        marker2.type = visualization_msgs::msg::Marker::CUBE;
        marker2.action = visualization_msgs::msg::Marker::ADD;
        
        marker2.pose.position.x = 2000.0;
        marker2.pose.position.y = 2000.0;
        marker2.pose.position.z = 800.0;
        marker2.pose.orientation.w = 1.0;
        
        marker2.scale.x = 100.0;
        marker2.scale.y = 100.0;
        marker2.scale.z = 100.0;
        
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;
        marker2.color.b = 0.0;
        marker2.color.a = 0.8;
        
        marker_array.markers.push_back(marker2);
        
        // Test marker 3: Blue cylinder
        auto marker3 = visualization_msgs::msg::Marker();
        marker3.header.frame_id = "map";
        marker3.header.stamp = this->now();
        marker3.ns = "test";
        marker3.id = 2;
        marker3.type = visualization_msgs::msg::Marker::CYLINDER;
        marker3.action = visualization_msgs::msg::Marker::ADD;
        
        marker3.pose.position.x = 8000.0;
        marker3.pose.position.y = 2000.0;
        marker3.pose.position.z = 1500.0;
        marker3.pose.orientation.w = 1.0;
        
        marker3.scale.x = 300.0;
        marker3.scale.y = 300.0;
        marker3.scale.z = 400.0;
        
        marker3.color.r = 0.0;
        marker3.color.g = 0.0;
        marker3.color.b = 1.0;
        marker3.color.a = 0.7;
        
        marker_array.markers.push_back(marker3);
        
        // Publish the marker array
        marker_pub_->publish(marker_array);
        
        RCLCPP_INFO(this->get_logger(), "Published %zu test markers", marker_array.markers.size());
    }
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleVisualizationTest>();
    
    RCLCPP_INFO(node->get_logger(), "Simple visualization test started. Press Ctrl+C to exit.");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
} 