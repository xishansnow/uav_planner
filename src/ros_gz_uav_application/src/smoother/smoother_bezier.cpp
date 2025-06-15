#include "smoother/smoother_bezier.hpp"
#include <algorithm>
#include <cmath>

BezierSmoother::BezierSmoother(int degree, int num_points)
    : degree_(degree), num_points_(num_points) {}

bool BezierSmoother::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
    std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path)
{
    if (raw_path.size() < 2) {
        smoothed_path = raw_path;
        return true;
    }
    
    // 根据度数生成控制点
    std::vector<geometry_msgs::msg::PoseStamped> controlPoints = generateControlPoints(raw_path);
    
    // 如果控制点数量不足，直接返回原始路径
    if (controlPoints.size() < 2) {
        smoothed_path = raw_path;
        return true;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> bezier_path;    
    for (int i = 0; i <= num_points_; ++i) {
        double t = static_cast<double>(i) / num_points_;
        bezier_path.push_back(deCasteljau(controlPoints, t));
    }
    smoothed_path = bezier_path;
    return true;
}

std::vector<geometry_msgs::msg::PoseStamped> BezierSmoother::generateControlPoints(
    const std::vector<geometry_msgs::msg::PoseStamped>& raw_path)
{
    std::vector<geometry_msgs::msg::PoseStamped> controlPoints;
    
    if (raw_path.size() < 2) {
        return raw_path;
    }
    
    // 根据度数决定控制点的生成策略
    switch (degree_) {
        case 1:  // 线性插值
            controlPoints = raw_path;
            break;
            
        case 2:  // 二次贝塞尔曲线
            if (raw_path.size() >= 3) {
                // 使用起点、中点和终点作为控制点
                controlPoints.push_back(raw_path.front());
                controlPoints.push_back(raw_path[raw_path.size() / 2]);
                controlPoints.push_back(raw_path.back());
            } else {
                controlPoints = raw_path;
            }
            break;
            
        case 3:  // 三次贝塞尔曲线（默认）
        default:
            if (raw_path.size() >= 4) {
                // 使用起点、1/3点、2/3点和终点作为控制点
                controlPoints.push_back(raw_path.front());
                controlPoints.push_back(raw_path[raw_path.size() / 3]);
                controlPoints.push_back(raw_path[2 * raw_path.size() / 3]);
                controlPoints.push_back(raw_path.back());
            } else if (raw_path.size() == 3) {
                // 对于3个点，使用起点、中点和终点
                controlPoints.push_back(raw_path.front());
                controlPoints.push_back(raw_path[1]);
                controlPoints.push_back(raw_path.back());
            } else {
                controlPoints = raw_path;
            }
            break;
            
        case 4:  // 四次贝塞尔曲线
            if (raw_path.size() >= 5) {
                controlPoints.push_back(raw_path.front());
                controlPoints.push_back(raw_path[raw_path.size() / 4]);
                controlPoints.push_back(raw_path[raw_path.size() / 2]);
                controlPoints.push_back(raw_path[3 * raw_path.size() / 4]);
                controlPoints.push_back(raw_path.back());
            } else {
                // 降级到三次贝塞尔曲线
                return generateControlPoints(raw_path);
            }
            break;
            
        case 5:  // 五次贝塞尔曲线
            if (raw_path.size() >= 6) {
                controlPoints.push_back(raw_path.front());
                controlPoints.push_back(raw_path[raw_path.size() / 5]);
                controlPoints.push_back(raw_path[2 * raw_path.size() / 5]);
                controlPoints.push_back(raw_path[3 * raw_path.size() / 5]);
                controlPoints.push_back(raw_path[4 * raw_path.size() / 5]);
                controlPoints.push_back(raw_path.back());
            } else {
                // 降级到四次贝塞尔曲线
                return generateControlPoints(raw_path);
            }
            break;
    }
    
    return controlPoints;
}

geometry_msgs::msg::PoseStamped BezierSmoother::interpolate(
    const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2, double t) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = (1 - t) * p1.pose.position.x + t * p2.pose.position.x;
    p.pose.position.y = (1 - t) * p1.pose.position.y + t * p2.pose.position.y;
    p.pose.position.z = (1 - t) * p1.pose.position.z + t * p2.pose.position.z;
    
    // 简单的四元数插值（线性插值）
    p.pose.orientation.x = (1 - t) * p1.pose.orientation.x + t * p2.pose.orientation.x;
    p.pose.orientation.y = (1 - t) * p1.pose.orientation.y + t * p2.pose.orientation.y;
    p.pose.orientation.z = (1 - t) * p1.pose.orientation.z + t * p2.pose.orientation.z;
    p.pose.orientation.w = (1 - t) * p1.pose.orientation.w + t * p2.pose.orientation.w;
    
    // 归一化四元数
    double norm = std::sqrt(p.pose.orientation.x * p.pose.orientation.x +
                           p.pose.orientation.y * p.pose.orientation.y +
                           p.pose.orientation.z * p.pose.orientation.z +
                           p.pose.orientation.w * p.pose.orientation.w);
    if (norm > 0) {
        p.pose.orientation.x /= norm;
        p.pose.orientation.y /= norm;
        p.pose.orientation.z /= norm;
        p.pose.orientation.w /= norm;
    }
    
    return p;
}

geometry_msgs::msg::PoseStamped BezierSmoother::deCasteljau(
    const std::vector<geometry_msgs::msg::PoseStamped>& controlPoints, double t) {
    std::vector<geometry_msgs::msg::PoseStamped> points = controlPoints;
    while (points.size() > 1) {
        std::vector<geometry_msgs::msg::PoseStamped> next;
        for (size_t i = 0; i < points.size() - 1; ++i) {
            next.push_back(interpolate(points[i], points[i + 1], t));
        }
        points = next;
    }
    return points[0];
} 