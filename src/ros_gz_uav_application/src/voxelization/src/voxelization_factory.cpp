#include "voxelization_base.hpp"
#include "voxelization_algorithms.hpp"
#include <memory>

namespace voxelization {

std::shared_ptr<VoxelizationBase> VoxelizationFactory::createAlgorithm(AlgorithmType type) {
    switch (type) {
        case AlgorithmType::CPU_SEQUENTIAL:
            return std::make_shared<CPUSequentialVoxelization>();
            
        case AlgorithmType::CPU_PARALLEL:
            return std::make_shared<CPUParallelVoxelization>();
            
        case AlgorithmType::GPU_CUDA:
            return std::make_shared<GPUCudaVoxelization>();
            
        case AlgorithmType::HYBRID:
            return std::make_shared<HybridVoxelization>();
            
        default:
            return nullptr;
    }
}

std::vector<std::string> VoxelizationFactory::getAvailableAlgorithms() {
    return {
        "CPU_SEQUENTIAL",
        "CPU_PARALLEL", 
        "GPU_CUDA",
        "HYBRID"
    };
}

} // namespace voxelization 