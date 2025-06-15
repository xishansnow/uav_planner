#pragma once
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>

class SmootherBase {
public:
    virtual ~SmootherBase() = default;
    virtual bool smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
        std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path) = 0;
}; 