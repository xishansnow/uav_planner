#pragma once
#include "smoother/smoother_base.hpp"

class BSplineSmoother : public SmootherBase {
public:
    BSplineSmoother(int degree = 2, int num_points = 32);
    bool smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
        std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path) override;
private:
    int degree_;
    int num_points_;
    std::vector<double> generateKnotVector(int num_control_points, int degree);
    double bSplineBasis(int i, int k, double u, const std::vector<double>& knots);
    geometry_msgs::msg::PoseStamped evaluateBSpline(const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
                                                   const std::vector<double>& knots, int degree, double u);
}; 