/*
 * Global Path Planner Base Class
 * Provides common interface for different path planning algorithms
 */

#ifndef GLOBAL_PLANNER_BASE_HPP
#define GLOBAL_PLANNER_BASE_HPP

#include <vector>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "env/environment_voxel3d.hpp"
#include "smoother/smoother_factory.hpp"

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
                         SmootherType smooth_type= SmootherType::BSPLINE,
                         double max_time = 5.0) = 0;

    /**
     * \brief Replan path from current position to goal
     * \param current_position Current position
     * \param current_path Current path
     * \param new_path Output new path
     * \param smooth_type Smoothing type
     * \param max_time Maximum planning time in seconds
     * \return true if path found, false otherwise
     */
    virtual bool replanPath(const geometry_msgs::msg::PoseStamped& current_position,
                           const std::vector<geometry_msgs::msg::PoseStamped>& current_path,
                           std::vector<geometry_msgs::msg::PoseStamped>& new_path,
                           SmootherType smooth_type = SmootherType::BSPLINE,       
                           double max_time = 5.0) = 0;

    /**
     * \brief Set algorithm parameters
     * \param param_name Parameter name
     * \param value Parameter value
     */
    virtual void setParameters(const std::string& param_name, double value) = 0;

    /**
     * \brief Get algorithm name
     */
    virtual std::string getAlgorithmName() const = 0;

    /**
     * \brief Get original path before smoothing (if available)
     * \return Original path before smoothing, empty if not available
     */
    virtual std::vector<geometry_msgs::msg::PoseStamped> getOriginalPath() const { return {}; }

protected:
    // Environment
    EnvironmentVoxel3D* env_;
    
    // Smoother
    std::shared_ptr<SmootherBase> smoother_;

    // Helper functions of EnvironmentVoxel3D::WorldToVoxel and EnvironmentVoxel3D::VoxelToWorld
    bool worldToVoxel(double world_x, double world_y, double world_z, 
                      int& voxel_x, int& voxel_y, int& voxel_z) const;
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                      double& world_x, double& world_y, double& world_z) const;
 

    
    // Set smoother based on smooth type
    void setSmoother(SmootherType smooth_type, int degree, int num_points);

    // Line of sight check for Theta*
    bool hasLineOfSight(const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal) const;
};

#endif // GLOBAL_PLANNER_BASE_HPP 