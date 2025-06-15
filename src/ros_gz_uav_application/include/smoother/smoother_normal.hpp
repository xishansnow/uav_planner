/*
 * Normal Path Smoother
 * Implements line-of-sight based path smoothing
 */

#ifndef SMOOTHER_NORMAL_HPP
#define SMOOTHER_NORMAL_HPP

#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "smoother/smoother_base.hpp"

/**
 * \brief Normal smoother that removes unnecessary waypoints using line-of-sight checks
 */
class SmootherNormal : public SmootherBase
{
public:
    /**
     * \brief Constructor
     */
    SmootherNormal(int num_points = 32);

    /**
     * \brief Destructor
     */
    virtual ~SmootherNormal();

    /**
     * \brief Smooth a path by removing unnecessary waypoints
     * \param raw_path Input path to smooth
     * \return Smoothed path
     */
    virtual bool smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path,
        std::vector<geometry_msgs::msg::PoseStamped>& smoothed_path) override;

    /**
     * \brief Set the number of points for smoothing
     */
    void setNumPoints(int num_points) { num_points_ = num_points; }

    /**
     * \brief Get the number of points for smoothing
     */
    int getNumPoints() const { return num_points_; }

private:
    int num_points_;  // 平滑后的节点数量

    /**
     * \brief Check if there is a clear line of sight between two points
     * \param start Start pose
     * \param goal Goal pose
     * \return true if line of sight exists, false otherwise
     */
    bool hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal) const;

    /**
     * \brief Interpolate between two points
     * \param p1 First point
     * \param p2 Second point
     * \param t Interpolation parameter (0.0 to 1.0)
     * \return Interpolated point
     */
    geometry_msgs::msg::PoseStamped interpolate(
        const geometry_msgs::msg::PoseStamped& p1,
        const geometry_msgs::msg::PoseStamped& p2,
        double t) const;
};

#endif // SMOOTHER_NORMAL_HPP 