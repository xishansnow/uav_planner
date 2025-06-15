#pragma once
#include "smoother/smoother_base.hpp"

class BezierSmoother : public SmootherBase {
public:
    BezierSmoother(int degree = 2, int num_points = 32);
    bool smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
        std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path) override;
    
    // 设置贝塞尔曲线的度数
    void setDegree(int degree) { degree_ = degree; }
    
    // 获取贝塞尔曲线的度数
    int getDegree() const { return degree_; }
    
private:
    int num_points_;
    int degree_;  // 贝塞尔曲线的度数
    
    geometry_msgs::msg::PoseStamped interpolate(const geometry_msgs::msg::PoseStamped& p1, const geometry_msgs::msg::PoseStamped& p2, double t);
    geometry_msgs::msg::PoseStamped deCasteljau(const std::vector<geometry_msgs::msg::PoseStamped>& controlPoints, double t);
    
    // 新增：根据度数生成控制点
    std::vector<geometry_msgs::msg::PoseStamped> generateControlPoints(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path);
}; 