#ifndef VOXELIZATION_BASE_HPP
#define VOXELIZATION_BASE_HPP

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>

namespace voxelization {

/**
 * @brief Base class for spatial entities that can be voxelized
 */
class SpatialEntity {
public:
    virtual ~SpatialEntity() = default;
    
    /**
     * @brief Get the bounding box of the entity
     * @return Bounding box as (min_x, min_y, min_z, max_x, max_y, max_z)
     */
    virtual std::vector<double> getBoundingBox() const = 0;
    
    /**
     * @brief Check if a point is inside the entity
     * @param x, y, z World coordinates
     * @return True if point is inside
     */
    virtual bool isPointInside(double x, double y, double z) const = 0;
    
    /**
     * @brief Get entity type as string
     * @return Entity type name
     */
    virtual std::string getType() const = 0;
    
    /**
     * @brief Get entity properties as a map
     * @return Properties map
     */
    virtual std::map<std::string, double> getProperties() const = 0;
};

/**
 * @brief Base class for voxelization algorithms
 */
class VoxelizationBase {
public:
    virtual ~VoxelizationBase() = default;
    
    /**
     * @brief Initialize voxelization with grid parameters
     * @param grid_x, grid_y, grid_z Grid dimensions
     * @param resolution_xy, resolution_z Voxel resolutions
     * @param origin_x, origin_y, origin_z Grid origin
     */
    virtual void initialize(int grid_x, int grid_y, int grid_z,
                          double resolution_xy, double resolution_z,
                          double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0) = 0;
    
    /**
     * @brief Voxelize a single spatial entity
     * @param entity Spatial entity to voxelize
     * @param buffer_size Buffer size around entity
     * @param cost_value Cost value for occupied voxels
     * @return Number of voxels marked
     */
    virtual int voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                              double buffer_size = 0.0,
                              unsigned char cost_value = 255) = 0;
    
    /**
     * @brief Voxelize multiple spatial entities
     * @param entities Vector of spatial entities
     * @param buffer_size Buffer size around entities
     * @param cost_value Cost value for occupied voxels
     * @return Number of voxels marked
     */
    virtual int voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                                double buffer_size = 0.0,
                                unsigned char cost_value = 255) = 0;
    
    /**
     * @brief Get the voxel grid
     * @return Pointer to voxel grid data
     */
    virtual unsigned char* getVoxelGrid() const = 0;
    
    /**
     * @brief Get grid dimensions
     * @return Grid dimensions as (x, y, z)
     */
    virtual std::vector<int> getGridDimensions() const = 0;
    
    /**
     * @brief Get voxel resolutions
     * @return Resolutions as (xy, z)
     */
    virtual std::vector<double> getResolutions() const = 0;
    
    /**
     * @brief Convert world coordinates to voxel coordinates
     * @param world_x, world_y, world_z World coordinates
     * @param voxel_x, voxel_y, voxel_z Output voxel coordinates
     * @return True if conversion successful
     */
    virtual bool worldToVoxel(double world_x, double world_y, double world_z,
                             int& voxel_x, int& voxel_y, int& voxel_z) const = 0;
    
    /**
     * @brief Convert voxel coordinates to world coordinates
     * @param voxel_x, voxel_y, voxel_z Voxel coordinates
     * @param world_x, world_y, world_z Output world coordinates
     */
    virtual void voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                             double& world_x, double& world_y, double& world_z) const = 0;
    
    /**
     * @brief Check if voxel coordinates are valid
     * @param voxel_x, voxel_y, voxel_z Voxel coordinates
     * @return True if coordinates are within grid bounds
     */
    virtual bool isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const = 0;
    
    /**
     * @brief Update voxel cost value
     * @param voxel_x, voxel_y, voxel_z Voxel coordinates
     * @param cost_value Cost value to set
     */
    virtual void updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) = 0;
    
    /**
     * @brief Get voxel cost value
     * @param voxel_x, voxel_y, voxel_z Voxel coordinates
     * @return Cost value
     */
    virtual unsigned char getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const = 0;
    
    /**
     * @brief Clear all voxels
     */
    virtual void clear() = 0;
    
    /**
     * @brief Save voxel grid to file
     * @param filename Output filename
     * @return True if successful
     */
    virtual bool saveToFile(const std::string& filename) const = 0;
    
    /**
     * @brief Load voxel grid from file
     * @param filename Input filename
     * @return True if successful
     */
    virtual bool loadFromFile(const std::string& filename) = 0;
};

/**
 * @brief Factory class for creating voxelization algorithms
 */
class VoxelizationFactory {
public:
    enum class AlgorithmType {
        CPU_SEQUENTIAL,
        CPU_PARALLEL,
        GPU_CUDA,
        HYBRID
    };
    
    /**
     * @brief Create a voxelization algorithm
     * @param type Algorithm type
     * @return Shared pointer to voxelization algorithm
     */
    static std::shared_ptr<VoxelizationBase> createAlgorithm(AlgorithmType type);
    
    /**
     * @brief Get available algorithm types
     * @return Vector of available algorithm types
     */
    static std::vector<std::string> getAvailableAlgorithms();
};

} // namespace voxelization

#endif // VOXELIZATION_BASE_HPP 