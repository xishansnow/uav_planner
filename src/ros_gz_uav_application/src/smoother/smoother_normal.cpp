#include "smoother/smoother_normal.hpp"
#include <cmath>
#include <algorithm>

SmootherNormal::SmootherNormal(int num_points)
    : num_points_(num_points)
{
}

SmootherNormal::~SmootherNormal()
{
}

bool SmootherNormal::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
    std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path)
{
    if (raw_path.size() < 2) {
        smoothed_path = raw_path;
        return true;
    }

    // 首先进行基本的路径平滑（移除不必要的路径点）
    std::vector<geometry_msgs::msg::PoseStamped> basic_smoothed;
    
    if (raw_path.size() >= 3) {
        basic_smoothed.push_back(raw_path.front());

        // Simple path smoothing: remove unnecessary waypoints
        for (size_t i = 1; i < raw_path.size() - 1; ++i) {
            const auto& prev = raw_path[i - 1];
            const auto& curr = raw_path[i];
            const auto& next = raw_path[i + 1];

            // Check if we can skip this point
            if (hasLineOfSight(prev, next)) {
                // Skip this point
                continue;
            } else {
                // Keep this point
                basic_smoothed.push_back(curr);
            }
        }

        basic_smoothed.push_back(raw_path.back());
    } else {
        basic_smoothed = raw_path;
    }

    // 如果用户指定的点数小于等于基本平滑后的点数，直接返回
    if (num_points_ <= static_cast<int>(basic_smoothed.size())) {
        smoothed_path = basic_smoothed;
        return true;
    }

    // 根据指定的点数进行插值
    smoothed_path.clear();
    
    // 计算总路径长度
    double total_length = 0.0;
    for (size_t i = 1; i < basic_smoothed.size(); ++i) {
        double dx = basic_smoothed[i].pose.position.x - basic_smoothed[i-1].pose.position.x;
        double dy = basic_smoothed[i].pose.position.y - basic_smoothed[i-1].pose.position.y;
        double dz = basic_smoothed[i].pose.position.z - basic_smoothed[i-1].pose.position.z;
        total_length += std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // 生成指定数量的点
    for (int i = 0; i < num_points_; ++i) {
        double target_distance = (total_length * i) / (num_points_ - 1);
        
        // 找到目标距离对应的路径段
        double current_distance = 0.0;
        size_t segment_index = 0;
        
        for (size_t j = 1; j < basic_smoothed.size(); ++j) {
            double dx = basic_smoothed[j].pose.position.x - basic_smoothed[j-1].pose.position.x;
            double dy = basic_smoothed[j].pose.position.y - basic_smoothed[j-1].pose.position.y;
            double dz = basic_smoothed[j].pose.position.z - basic_smoothed[j-1].pose.position.z;
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (current_distance + segment_length >= target_distance) {
                segment_index = j - 1;
                break;
            }
            current_distance += segment_length;
        }
        
        // 如果到达最后一个点
        if (i == num_points_ - 1) {
            smoothed_path.push_back(basic_smoothed.back());
        } else {
            // 在当前段内插值
            double segment_start_distance = 0.0;
            for (size_t j = 1; j <= segment_index; ++j) {
                double dx = basic_smoothed[j].pose.position.x - basic_smoothed[j-1].pose.position.x;
                double dy = basic_smoothed[j].pose.position.y - basic_smoothed[j-1].pose.position.y;
                double dz = basic_smoothed[j].pose.position.z - basic_smoothed[j-1].pose.position.z;
                segment_start_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            double dx = basic_smoothed[segment_index+1].pose.position.x - basic_smoothed[segment_index].pose.position.x;
            double dy = basic_smoothed[segment_index+1].pose.position.y - basic_smoothed[segment_index].pose.position.y;
            double dz = basic_smoothed[segment_index+1].pose.position.z - basic_smoothed[segment_index].pose.position.z;
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            double t = (target_distance - segment_start_distance) / segment_length;
            t = std::max(0.0, std::min(1.0, t));  // 限制在[0,1]范围内
            
            smoothed_path.push_back(interpolate(basic_smoothed[segment_index], basic_smoothed[segment_index+1], t));
        }
    }

    return true;
}

geometry_msgs::msg::PoseStamped SmootherNormal::interpolate(
    const geometry_msgs::msg::PoseStamped& p1,
    const geometry_msgs::msg::PoseStamped& p2,
    double t) const
{
    geometry_msgs::msg::PoseStamped result;
    
    // 位置插值
    result.pose.position.x = (1 - t) * p1.pose.position.x + t * p2.pose.position.x;
    result.pose.position.y = (1 - t) * p1.pose.position.y + t * p2.pose.position.y;
    result.pose.position.z = (1 - t) * p1.pose.position.z + t * p2.pose.position.z;
    
    // 四元数插值（线性插值）
    result.pose.orientation.x = (1 - t) * p1.pose.orientation.x + t * p2.pose.orientation.x;
    result.pose.orientation.y = (1 - t) * p1.pose.orientation.y + t * p2.pose.orientation.y;
    result.pose.orientation.z = (1 - t) * p1.pose.orientation.z + t * p2.pose.orientation.z;
    result.pose.orientation.w = (1 - t) * p1.pose.orientation.w + t * p2.pose.orientation.w;
    
    // 归一化四元数
    double norm = std::sqrt(result.pose.orientation.x * result.pose.orientation.x +
                           result.pose.orientation.y * result.pose.orientation.y +
                           result.pose.orientation.z * result.pose.orientation.z +
                           result.pose.orientation.w * result.pose.orientation.w);
    if (norm > 0) {
        result.pose.orientation.x /= norm;
        result.pose.orientation.y /= norm;
        result.pose.orientation.z /= norm;
        result.pose.orientation.w /= norm;
    }
    
    return result;
}

bool SmootherNormal::hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                                   const geometry_msgs::msg::PoseStamped& goal) const
{
    // For now, we'll use a simple distance-based check
    // In a real implementation, you would check against the environment/obstacles
    
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double dz = goal.pose.position.z - start.pose.position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // If the distance is small, assume line of sight exists
    // This is a simplified implementation - in practice, you would need to check
    // against the actual environment representation
    return distance < 10.0;  // 10 meter threshold for simplicity
} 