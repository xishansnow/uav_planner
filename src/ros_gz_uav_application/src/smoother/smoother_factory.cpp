#include "smoother/smoother_factory.hpp"
#include "smoother/smoother_normal.hpp"
#include "smoother/smoother_bezier.hpp"
#include "smoother/smoother_bspline.hpp"
#include "smoother/smoother_minvo.hpp"

std::shared_ptr<SmootherBase> SmootherFactory::createSmoother(SmootherType type, int degree, int num_points) {
    switch (type) {
        case SmootherType::NORMAL:
            return std::make_shared<SmootherNormal>(num_points);
        case SmootherType::BEZIER:
            return std::make_shared<BezierSmoother>(degree, num_points);
        case SmootherType::BSPLINE:
            return std::make_shared<BSplineSmoother>(degree, num_points);
        case SmootherType::MINVO:
            return std::make_shared<MINVOSmoother>(degree, num_points);
        case SmootherType::NONE:
        default:
            return nullptr;
    }
} 