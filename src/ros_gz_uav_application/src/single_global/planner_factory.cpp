#include "single_global/planner_factory.hpp"
#include <algorithm>
#include <stdexcept>
#include <memory>

std::shared_ptr<GlobalPlannerBase> PlannerFactory::createPlanner(PlannerType type, EnvironmentVoxel3D* env)
{
    switch (type) {
        case ASTAR:
            return std::make_shared<AStarPlanner>(env);
        case THETASTAR:
            return std::make_shared<ThetaStarPlanner>(env);
        case ARASTAR:
            return std::make_shared<ARAStarPlanner>(env);
        case JPS:
            return std::make_shared<JPSPlanner>(env);
        case MULTISCALE_ASTAR:
            return std::make_shared<MultiScaleAStarPlanner>(env);
        default:
            throw std::invalid_argument("Unknown planner type");
    }
}

std::shared_ptr<GlobalPlannerBase> PlannerFactory::createPlanner(const std::string& planner_name, EnvironmentVoxel3D* env)
{
    PlannerType type = stringToPlannerType(planner_name);
    return createPlanner(type, env);
}

std::vector<std::string> PlannerFactory::getAvailablePlanners()
{
    return {"astar", "thetastar", "arastar", "jps", "multiscale_astar"};
}

std::string PlannerFactory::plannerTypeToString(PlannerType type)
{
    switch (type) {
        case ASTAR:
            return "astar";
        case THETASTAR:
            return "thetastar";
        case ARASTAR:
            return "arastar";
        case JPS:
            return "jps";
        case MULTISCALE_ASTAR:
            return "multiscale_astar";
        default:
            return "unknown";
    }
}

PlannerFactory::PlannerType PlannerFactory::stringToPlannerType(const std::string& name)
{
    std::string lower_name = name;
    std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
    
    if (lower_name == "astar" || lower_name == "a*") {
        return ASTAR;
    } else if (lower_name == "thetastar" || lower_name == "theta*") {
        return THETASTAR;
    } else if (lower_name == "arastar" || lower_name == "ara*") {
        return ARASTAR;
    } else if (lower_name == "jps" || lower_name == "jump point search") {
        return JPS;
    } else if (lower_name == "multiscale_astar" || lower_name == "multiscale a*" || lower_name == "multiscale") {
        return MULTISCALE_ASTAR;
    } else {
        throw std::invalid_argument("Unknown planner name: " + name);
    }
} 