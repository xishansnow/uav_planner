/*
 * Global Path Planner Factory
 * Factory class for creating different path planning algorithms
 */

#ifndef PLANNER_FACTORY_HPP
#define PLANNER_FACTORY_HPP

#include <memory>
#include <string>
#include "global_planner_base.hpp"
#include "astar_planner.hpp"
#include "thetastar_planner.hpp"
#include "arastar_planner.hpp"

/**
 * \brief Factory class for creating global path planners
 */
class PlannerFactory
{
public:
    /**
     * \brief Available planner types
     */
    enum PlannerType
    {
        ASTAR,
        THETASTAR,
        ARASTAR
    };
    
    /**
     * \brief Create a planner instance
     * \param type The type of planner to create
     * \param env The environment to use
     * \return Shared pointer to the created planner
     */
    static std::shared_ptr<GlobalPlannerBase> createPlanner(PlannerType type, EnvironmentVoxel3D* env);
    
    /**
     * \brief Create a planner instance from string
     * \param planner_name The name of the planner ("astar", "thetastar", "arastar")
     * \param env The environment to use
     * \return Shared pointer to the created planner
     */
    static std::shared_ptr<GlobalPlannerBase> createPlanner(const std::string& planner_name, EnvironmentVoxel3D* env);
    
    /**
     * \brief Get list of available planner names
     * \return Vector of planner names
     */
    static std::vector<std::string> getAvailablePlanners();
    
    /**
     * \brief Convert planner type to string
     * \param type The planner type
     * \return String representation
     */
    static std::string plannerTypeToString(PlannerType type);
    
    /**
     * \brief Convert string to planner type
     * \param name The planner name
     * \return Planner type
     */
    static PlannerType stringToPlannerType(const std::string& name);
};

#endif // PLANNER_FACTORY_HPP 