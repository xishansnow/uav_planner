#include "smoother/smoother_minvo.hpp"
#include <cmath>

MINVOSmoother::MINVOSmoother(int degree, int num_points)
    : degree_(degree), num_points_(num_points) {}

bool MINVOSmoother::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
    std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path)
{
    if (control_points.size() < degree_ + 1) {
        return false;
    }
    std::vector<geometry_msgs::msg::PoseStamped> minvo_path;
    for (int i = 0; i < num_points_; ++i) {
        double t = static_cast<double>(i) / (num_points_ - 1);
        geometry_msgs::msg::PoseStamped point = evaluateMINVO(control_points, degree_, t);
        minvo_path.push_back(point);
    }
    minvo_path.back() = control_points.back();
    smoothed_path = minvo_path;
    return true;
}

double MINVOSmoother::minvoBasis(int i, int degree, double t)
{
    if (degree == 1) {
        if (i == 0) return 1.0 - t;
        if (i == 1) return t;
        return 0.0;
    }
    else if (degree == 2) {
        if (i == 0) return (1.0 - t) * (1.0 - t);
        if (i == 1) return 2.0 * t * (1.0 - t);
        if (i == 2) return t * t;
        return 0.0;
    }
    else if (degree == 3) {
        if (i == 0) return (1.0 - t) * (1.0 - t) * (1.0 - t);
        if (i == 1) return 3.0 * t * (1.0 - t) * (1.0 - t);
        if (i == 2) return 3.0 * t * t * (1.0 - t);
        if (i == 3) return t * t * t;
        return 0.0;
    }
    else {
        double result = 0.0;
        for (int j = 0; j <= degree; ++j) {
            double bernstein = factorial(degree) / (factorial(j) * factorial(degree - j)) * 
                              std::pow(t, j) * std::pow(1.0 - t, degree - j);
            double minvo_coeff = (i == j) ? 1.0 : 0.0;
            result += minvo_coeff * bernstein;
        }
        return result;
    }
}

geometry_msgs::msg::PoseStamped MINVOSmoother::evaluateMINVO(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
    int degree, double t)
{
    geometry_msgs::msg::PoseStamped result;
    result.pose.position.x = 0.0;
    result.pose.position.y = 0.0;
    result.pose.position.z = 0.0;
    result.pose.orientation.w = 1.0;
    result.pose.orientation.x = 0.0;
    result.pose.orientation.y = 0.0;
    result.pose.orientation.z = 0.0;
    for (size_t i = 0; i < control_points.size(); ++i) {
        double basis = minvoBasis(i, degree, t);
        result.pose.position.x += basis * control_points[i].pose.position.x;
        result.pose.position.y += basis * control_points[i].pose.position.y;
        result.pose.position.z += basis * control_points[i].pose.position.z;
    }
    return result;
}

unsigned long long MINVOSmoother::factorial(unsigned int n) {
    if (n == 0 || n == 1) return 1;
    return n * factorial(n - 1);
} 