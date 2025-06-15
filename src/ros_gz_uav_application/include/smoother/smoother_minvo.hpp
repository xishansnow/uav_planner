#pragma once
#include "smoother/smoother_base.hpp"

class MINVOSmoother : public SmootherBase {
public:
    MINVOSmoother(int degree = 2, int num_points = 32);
    bool smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
        std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path) override;
private:
    int degree_;
    int num_points_;
    double minvoBasis(int i, int degree, double t);
    geometry_msgs::msg::PoseStamped evaluateMINVO(const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
                                                 int degree, double t);
    unsigned long long factorial(unsigned int n);
}; 