#ifndef VOXELIZATION_ALGORITHMS_HPP
#define VOXELIZATION_ALGORITHMS_HPP

#include "voxelization_base.hpp"
#include <vector>
#include <memory>
#include <fstream>

namespace voxelization {

/**
 * @brief CPU sequential voxelization algorithm
 */
class CPUSequentialVoxelization : public VoxelizationBase {
public:
    CPUSequentialVoxelization();
    ~CPUSequentialVoxelization() override;
    
    void initialize(int grid_x, int grid_y, int grid_z,
                   double resolution_xy, double resolution_z,
                   double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) override;
    
    int voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                      double buffer_size = 0.0,
                      unsigned char cost_value = 255) override;
    
    int voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                        double buffer_size = 0.0,
                        unsigned char cost_value = 255) override;
    
    unsigned char* getVoxelGrid() const override { return voxel_grid_.data(); }
    std::vector<int> getGridDimensions() const override { return {grid_x_, grid_y_, grid_z_}; }
    std::vector<double> getResolutions() const override { return {resolution_xy_, resolution_z_}; }
    
    bool worldToVoxel(double world_x, double world_y, double world_z,
                     int& voxel_x, int& voxel_y, int& voxel_z) const override;
    
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                     double& world_x, double& world_y, double& world_z) const override;
    
    bool isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const override;
    void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) override;
    unsigned char getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const override;
    void clear() override;
    
    bool saveToFile(const std::string& filename) const override;
    bool loadFromFile(const std::string& filename) override;
    
private:
    std::vector<unsigned char> voxel_grid_;
    int grid_x_, grid_y_, grid_z_;
    double resolution_xy_, resolution_z_;
    double origin_x_, origin_y_, origin_z_;
    
    int markBoxEntity(const std::shared_ptr<SpatialEntity>& entity,
                     double buffer_size, unsigned char cost_value);
    int markCylinderEntity(const std::shared_ptr<SpatialEntity>& entity,
                          double buffer_size, unsigned char cost_value);
    int markSphereEntity(const std::shared_ptr<SpatialEntity>& entity,
                        double buffer_size, unsigned char cost_value);
    int markMeshEntity(const std::shared_ptr<SpatialEntity>& entity,
                      double buffer_size, unsigned char cost_value);
};

/**
 * @brief CPU parallel voxelization algorithm using OpenMP
 */
class CPUParallelVoxelization : public VoxelizationBase {
public:
    CPUParallelVoxelization();
    ~CPUParallelVoxelization() override;
    
    void initialize(int grid_x, int grid_y, int grid_z,
                   double resolution_xy, double resolution_z,
                   double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) override;
    
    int voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                      double buffer_size = 0.0,
                      unsigned char cost_value = 255) override;
    
    int voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                        double buffer_size = 0.0,
                        unsigned char cost_value = 255) override;
    
    unsigned char* getVoxelGrid() const override { return voxel_grid_.data(); }
    std::vector<int> getGridDimensions() const override { return {grid_x_, grid_y_, grid_z_}; }
    std::vector<double> getResolutions() const override { return {resolution_xy_, resolution_z_}; }
    
    bool worldToVoxel(double world_x, double world_y, double world_z,
                     int& voxel_x, int& voxel_y, int& voxel_z) const override;
    
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                     double& world_x, double& world_y, double& world_z) const override;
    
    bool isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const override;
    void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) override;
    unsigned char getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const override;
    void clear() override;
    
    bool saveToFile(const std::string& filename) const override;
    bool loadFromFile(const std::string& filename) override;
    
private:
    std::vector<unsigned char> voxel_grid_;
    int grid_x_, grid_y_, grid_z_;
    double resolution_xy_, resolution_z_;
    double origin_x_, origin_y_, origin_z_;
    
    int markBoxEntityParallel(const std::shared_ptr<SpatialEntity>& entity,
                             double buffer_size, unsigned char cost_value);
    int markCylinderEntityParallel(const std::shared_ptr<SpatialEntity>& entity,
                                  double buffer_size, unsigned char cost_value);
    int markSphereEntityParallel(const std::shared_ptr<SpatialEntity>& entity,
                                double buffer_size, unsigned char cost_value);
    int markMeshEntityParallel(const std::shared_ptr<SpatialEntity>& entity,
                              double buffer_size, unsigned char cost_value);
};

/**
 * @brief GPU voxelization algorithm using CUDA
 */
class GPUCudaVoxelization : public VoxelizationBase {
public:
    GPUCudaVoxelization();
    ~GPUCudaVoxelization() override;
    
    void initialize(int grid_x, int grid_y, int grid_z,
                   double resolution_xy, double resolution_z,
                   double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) override;
    
    int voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                      double buffer_size = 0.0,
                      unsigned char cost_value = 255) override;
    
    int voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                        double buffer_size = 0.0,
                        unsigned char cost_value = 255) override;
    
    unsigned char* getVoxelGrid() const override { return voxel_grid_.data(); }
    std::vector<int> getGridDimensions() const override { return {grid_x_, grid_y_, grid_z_}; }
    std::vector<double> getResolutions() const override { return {resolution_xy_, resolution_z_}; }
    
    bool worldToVoxel(double world_x, double world_y, double world_z,
                     int& voxel_x, int& voxel_y, int& voxel_z) const override;
    
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                     double& world_x, double& world_y, double& world_z) const override;
    
    bool isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const override;
    void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) override;
    unsigned char getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const override;
    void clear() override;
    
    bool saveToFile(const std::string& filename) const override;
    bool loadFromFile(const std::string& filename) override;
    
private:
    std::vector<unsigned char> voxel_grid_;
    int grid_x_, grid_y_, grid_z_;
    double resolution_xy_, resolution_z_;
    double origin_x_, origin_y_, origin_z_;
    
    // GPU memory pointers
    unsigned char* d_voxel_grid_;
    size_t grid_size_;
    
    bool gpu_available_;
    
    void allocateGPUMemory();
    void freeGPUMemory();
    void copyToGPU();
    void copyFromGPU();
    
    int markBoxEntityGPU(const std::shared_ptr<SpatialEntity>& entity,
                        double buffer_size, unsigned char cost_value);
    int markCylinderEntityGPU(const std::shared_ptr<SpatialEntity>& entity,
                             double buffer_size, unsigned char cost_value);
    int markSphereEntityGPU(const std::shared_ptr<SpatialEntity>& entity,
                           double buffer_size, unsigned char cost_value);
};

/**
 * @brief Hybrid voxelization algorithm (CPU + GPU)
 */
class HybridVoxelization : public VoxelizationBase {
public:
    HybridVoxelization();
    ~HybridVoxelization() override;
    
    void initialize(int grid_x, int grid_y, int grid_z,
                   double resolution_xy, double resolution_z,
                   double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) override;
    
    int voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                      double buffer_size = 0.0,
                      unsigned char cost_value = 255) override;
    
    int voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                        double buffer_size = 0.0,
                        unsigned char cost_value = 255) override;
    
    unsigned char* getVoxelGrid() const override { return voxel_grid_.data(); }
    std::vector<int> getGridDimensions() const override { return {grid_x_, grid_y_, grid_z_}; }
    std::vector<double> getResolutions() const override { return {resolution_xy_, resolution_z_}; }
    
    bool worldToVoxel(double world_x, double world_y, double world_z,
                     int& voxel_x, int& voxel_y, int& voxel_z) const override;
    
    void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                     double& world_x, double& world_y, double& world_z) const override;
    
    bool isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const override;
    void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) override;
    unsigned char getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const override;
    void clear() override;
    
    bool saveToFile(const std::string& filename) const override;
    bool loadFromFile(const std::string& filename) override;
    
private:
    std::vector<unsigned char> voxel_grid_;
    int grid_x_, grid_y_, grid_z_;
    double resolution_xy_, resolution_z_;
    double origin_x_, origin_y_, origin_z_;
    
    std::unique_ptr<CPUSequentialVoxelization> cpu_voxelizer_;
    std::unique_ptr<GPUCudaVoxelization> gpu_voxelizer_;
    
    bool gpu_available_;
    
    // Strategy selection
    enum class Strategy {
        CPU_ONLY,
        GPU_ONLY,
        HYBRID
    };
    
    Strategy selectStrategy(const std::shared_ptr<SpatialEntity>& entity) const;
    Strategy selectStrategy(const std::vector<std::shared_ptr<SpatialEntity>>& entities) const;
};

} // namespace voxelization

#endif // VOXELIZATION_ALGORITHMS_HPP 