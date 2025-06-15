/*
 * 3D Voxel Environment for Path Planning
 * Simplified version without SBPL dependency
 * Adapted for UAV path planning with octomap support
 * Loosely coupled with path planners
 */

#ifndef ENV_ENVIRONMENT_VOXEL3D_HPP
#define ENV_ENVIRONMENT_VOXEL3D_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>

// Configuration structure for 3D voxel environment
typedef struct ENV_VOXEL3D_CONFIG
{
    ENV_VOXEL3D_CONFIG() 
    {
        // Default values
        Env_x = 100;
        Env_y = 100;
        Env_z = 50;
        ResolutionXY = 0.1;
        ResolutionZ = 0.1;
        OriginX = -5.0;
        OriginY = -5.0;
        OriginZ = 0.0;
        numofdirs = 26;
    }

    int Env_x;   // X dimension in voxels
    int Env_y;  // Y dimension in voxels
    int Env_z;   // Z dimension in voxels
    double ResolutionXY; // Size of each voxel in meters (X and Y)
    double ResolutionZ; // Size of each voxel in Z direction (can be different from X/Y)
    double OriginX;   // Origin X coordinate in world frame
    double OriginY;   // Origin Y coordinate in world frame
    double OriginZ;   // Origin Z coordinate in world frame
    
    // Movement directions (26-connectivity for 3D)
    int dx_[26];
    int dy_[26];
    int dz_[26];
    int dxy_distance_mm_[26]; // Distances for each movement direction
    
    int numofdirs; // Number of movement directions (26 for 3D)
} EnvVoxel3DConfig_t;

/**
 * \brief 3D Voxel Environment for path planning
 * Simplified version without SBPL dependency
 * Supports octomap integration and 3D path planning for UAVs
 * Loosely coupled with path planners - no internal state management
 */
class EnvironmentVoxel3D
{
public:
    /**
     * \brief Constructor
     */
    EnvironmentVoxel3D();

    /**
     * \brief Destructor
     */
    ~EnvironmentVoxel3D();

    /**
     * \brief Initialize environment with 3D voxel grid which has the same resolution in X and Y directions
     */
    virtual bool InitializeEnv(int env_x, int env_y, int env_z, 
                              double resolution, double origin_x, double origin_y, double origin_z);

    /**
     * \brief Initialize environment with 3D voxel grid and different Z resolution
     */
    virtual bool InitializeEnv(int env_x, int env_y, int env_z, 
                              double resolution_xy, double resolution_z,
                              double origin_x, double origin_y, double origin_z);

    /**
     * \brief Check if coordinates are within map bounds
     */
    virtual bool IsCellWithinMap(int voxel_x, int voxel_y, int voxel_z) const;


    /**
     * \brief Check if voxel is valid
     */
    virtual bool IsValidCell(int voxel_x, int voxel_y, int voxel_z) const;

    /**
     * \brief Check if voxel is occupied
     */
    virtual bool IsCellOccupied(int voxel_x, int voxel_y, int voxel_z);

    /**
     * \brief Check if voxel is free (known to be empty)
     */
    virtual bool IsCellFree(int voxel_x, int voxel_y, int voxel_z);


    /**
     * \brief Check if voxel is unknown (not explored)
     */
    virtual bool IsCellUnknown(int voxel_x, int voxel_y, int voxel_z);

    /**
     * \brief Get map cost for a voxel
     * Octomap 的 cost 值范围是 0-100，0 表示 free，100 表示 occupied，50 表示 unknown
     * Octomap 的 logodds 值范围是 -inf 到 inf，-inf 表示 free，inf 表示 occupied，0 表示 unknown
     * 此处为了习惯，使用了 cost 值，如果想使用 logodds 值，可以直接调用 octomap 的 getLogOdds 函数
     * 0 - 表示 free
     * 255 - 表示障碍物占用或超出边界
     * 128 - 表示 path 占用
     */
    virtual unsigned char GetCellCost(int voxel_x, int voxel_y, int voxel_z);


    /**
     * \brief Update cost of a voxel
     * 由于底层使用 octomap，所以需要将 cost 值转换为 logodds 值，转换公式为：logodds = -100 * log(1 - cost / 100)
     * 0 - 表示 free，octomap 中无对应节点
     * 255 - 表示障碍物占用或超出边界
     * 128 - 表示 path 占用
     */
    virtual bool UpdateCellCost(int voxel_x, int voxel_y, int voxel_z, unsigned char newcost);

    /**
     * \brief Update environment from octomap
     */
    virtual void UpdateFromOctomap(const std::shared_ptr<octomap::OcTree>& octomap);

    /**
     * \brief Insert pointcloud into environment
     */
    virtual void InsertPointCloud(const octomap::Pointcloud& pointcloud, 
                                 const octomap::point3d& sensor_origin);

    /**
     * \brief Get octomap
     */
    virtual std::shared_ptr<octomap::OcTree> GetOctomap() const { return octomap_; }

    /**
     * \brief Convert world coordinates to voxel coordinates
     */
    bool WorldToVoxel(double world_x, double world_y, double world_z, 
                      int& voxel_x, int& voxel_y, int& voxel_z) const;

    /**
     * \brief Convert voxel coordinates to world coordinates
     */
    void VoxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                      double& world_x, double& world_y, double& world_z) const;

    /**
     * \brief Load environment from binary tree file
     */
    bool LoadFromBTFile(const std::string& filename);

    /**
     * \brief Save environment to binary tree file
     */
    bool SaveToBTFile(const std::string& filename) const;

    /**int
     * \brief Load environment from binary tree file with config
     */
    bool LoadFromBTFileWithConfig(const std::string& filename);

    /**
     * \brief Save environment to binary tree file with config
     */
    bool SaveToBTFileWithConfig(const std::string& filename) const;

    /**
     * \brief Save configuration to YAML file
     */
    bool SaveConfigToYAML(const std::string& filename) const;

    /**
     * \brief Load configuration from YAML file
     */
    bool LoadConfigFromYAML(const std::string& filename);
    /**
     * \brief Get grid length in X dimension
     */
    int GetGridX() const { return EnvVoxel3DCfg.Env_x; }

    /**
     * \brief Get grid length in Y dimension
     */
    int GetGridY() const { return EnvVoxel3DCfg.Env_y; }

    /**
     * \brief Get grid length in Z dimension 
     */
    int GetGridZ() const { return EnvVoxel3DCfg.Env_z; }


    /**
     * \brief Get grid size
     */
    void GetGridSize(int& size_x, int& size_y, int& size_z) const;

    /**
     * \brief Get environment bounds
     */
    void GetEnvBounds(double& min_x, double& min_y, double& min_z, 
                      double& max_x, double& max_y, double& max_z) const;

    /**
     * \brief Get XY resolution
     */
    double GetResolutionXY() const;

    /**
     * \brief Get Z resolution
     */
    double GetResolutionZ() const;

    /**
     * \brief Get octomap occupancy probability for a voxel
     */
    virtual double GetCellOccupancyProbability(int voxel_x, int voxel_y, int voxel_z);

protected:
    EnvVoxel3DConfig_t EnvVoxel3DCfg;

    // Octomap for 3D environment representation
    std::shared_ptr<octomap::OcTree> octomap_;

    virtual void Computedxyz();
    
    virtual void InitializeEnvConfig();
};

#endif // ENV_ENVIRONMENT_VOXEL3D_HPP