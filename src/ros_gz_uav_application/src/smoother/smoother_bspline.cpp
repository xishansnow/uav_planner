#include "smoother/smoother_bspline.hpp"
#include <cmath>

BSplineSmoother::BSplineSmoother(int degree, int num_points)
    : degree_(degree), num_points_(num_points) {}

bool BSplineSmoother::smoothPath(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
    std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path)
{
    if (control_points.size() < degree_ + 1) {
        return false;
    }
    std::vector<geometry_msgs::msg::PoseStamped> bspline_path;
    std::vector<double> knots = generateKnotVector(control_points.size(), degree_);
    double u_min = knots[degree_];
    double u_max = knots[knots.size() - degree_ - 1];
    for (int i = 0; i < num_points_; ++i) {
        double u = u_min + (u_max - u_min) * static_cast<double>(i) / num_points_;
        geometry_msgs::msg::PoseStamped point = evaluateBSpline(control_points, knots, degree_, u);
        bspline_path.push_back(point);
    }
    bspline_path.push_back(control_points.back());
    smoothed_path = bspline_path;
    return true;
}

std::vector<double> BSplineSmoother::generateKnotVector(int num_control_points, int degree)
{
    std::vector<double> knots;
    int num_knots = num_control_points + degree + 1;
    for (int i = 0; i < num_knots; ++i) {
        if (i < degree + 1) {
            knots.push_back(0.0);
        } else if (i >= num_control_points) {
            knots.push_back(1.0);
        } else {
            knots.push_back(static_cast<double>(i - degree) / (num_control_points - degree));
        }
    }
    return knots;
}

double BSplineSmoother::bSplineBasis(int i, int k, double u, const std::vector<double>& knots)
{
    if (k == 1) {
        if (u >= knots[i] && u < knots[i + 1]) {
            return 1.0;
        } else {
            return 0.0;
        }
    }
    double d1 = knots[i + k - 1] - knots[i];
    double d2 = knots[i + k] - knots[i + 1];
    double c1 = (d1 > 1e-10) ? (u - knots[i]) / d1 : 0.0;
    double c2 = (d2 > 1e-10) ? (knots[i + k] - u) / d2 : 0.0;
    return c1 * bSplineBasis(i, k - 1, u, knots) + c2 * bSplineBasis(i + 1, k - 1, u, knots);
}

geometry_msgs::msg::PoseStamped BSplineSmoother::evaluateBSpline(
    const std::vector<geometry_msgs::msg::PoseStamped>& control_points,
    const std::vector<double>& knots, int degree, double u)
{
    geometry_msgs::msg::PoseStamped result;
    result.pose.position.x = 0.0;
    result.pose.position.y = 0.0;
    result.pose.position.z = 0.0;
    result.pose.orientation.w = 1.0;
    result.pose.orientation.x = 0.0;
    result.pose.orientation.y = 0.0;
    result.pose.orientation.z = 0.0;
    double total_weight = 0.0;
    for (size_t i = 0; i < control_points.size(); ++i) {
        double basis = bSplineBasis(i, degree + 1, u, knots);
        total_weight += basis;
        result.pose.position.x += basis * control_points[i].pose.position.x;
        result.pose.position.y += basis * control_points[i].pose.position.y;
        result.pose.position.z += basis * control_points[i].pose.position.z;
    }
    if (total_weight > 1e-10) {
        result.pose.position.x /= total_weight;
        result.pose.position.y /= total_weight;
        result.pose.position.z /= total_weight;
    }
    return result;
} 