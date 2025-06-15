#pragma once
#include <memory>
#include "smoother/smoother_base.hpp"

enum class SmootherType {
    NONE = 0,
    NORMAL = 1,
    BEZIER = 2,
    BSPLINE = 3,
    MINVO = 4
};

class SmootherFactory {
public:
    static std::shared_ptr<SmootherBase> createSmoother(SmootherType type, int degree = 2, int num_points = 20);
}; 