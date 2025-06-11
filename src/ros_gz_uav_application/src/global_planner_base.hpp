/*
 * Global Path Planner Base Class
 * Provides common interface for different path planning algorithms
 */

#ifndef GLOBAL_PLANNER_BASE_HPP
#define GLOBAL_PLANNER_BASE_HPP

#include <vector>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "environment_voxel3d.hpp"

// Forward declarations
class EnvironmentVoxel3D;

/**
 * \brief Base class for global path planning algorithms
 */
class GlobalPlannerBase
{
public:
    /**
     * \brief Constructor
     */
    GlobalPlannerBase(EnvironmentVoxel3D* env);

    /**
     * \brief Virtual destructor
     */
    virtual ~GlobalPlannerBase();

    /**
     * \brief Plan path from start to goal
     * \param start_x, start_y, start_z Start position
     * \param goal_x, goal_y, goal_z Goal position
     * \param path Output path
     * \param max_time Maximum planning time in seconds
     * \return true if path found, false otherwise
     */
    virtual bool planPath(double start_x, double start_y, double start_z,
                         double goal_x, double goal_y, double goal_z,
                         std::vector<geometry_msgs::msg::PoseStamped>& path,
                         double max_time = 5.0) = 0;

    /**
     * \brief Replan path from current position to goal
     * \param current_path Current path
     * \param new_path Output new path
     * \param max_time Maximum planning time in seconds
     * \return true if path found, false otherwise
     */
    virtual bool replanPath(const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
                           std::vector<geometry_msgs::msg::PoseStamped>& new_path,
                           double max_time = 5.0) = 0;

    /**
     * \brief Set algorithm parameters
     */
    virtual void setParameters(const std::string& param_name, double value) = 0;

    /**
     * \brief Get algorithm name
     */
    virtual std::string getAlgorithmName() const = 0;

protected:
    // Environment
    EnvironmentVoxel3D* env_;

    // Helper functions
    bool worldToVoxel(double world_x, double world_y, double world_z, 
                      int& voxel_x, int& voxel_y, int& voxel_z) const;
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                      double& world_x, double& world_y, double& world_z) const;
    
    // Path smoothing
    std::vector<geometry_msgs::msg::PoseStamped> smoothPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& raw_path);
    
    // Line of sight check for Theta*
    bool hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal) const;
};

#endif // GLOBAL_PLANNER_BASE_HPP 