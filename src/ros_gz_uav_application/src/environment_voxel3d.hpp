/*
 * 3D Voxel Environment for SBPL
 * Based on MoveIt's Environment_Chain3D design
 * Adapted for UAV path planning with octomap support
 */

#ifndef ENVIRONMENT_VOXEL3D_HPP
#define ENVIRONMENT_VOXEL3D_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <sbpl/discrete_space_information/environment.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>

// Forward declarations
class CMDPSTATE;
class MDPConfig;

// Configuration structure for 3D voxel environment
typedef struct ENV_VOXEL3D_CONFIG
{
    ENV_VOXEL3D_CONFIG() 
    {
        // Remove VoxelGrid3D as we'll use OcTree directly
    }

    int EnvWidth_c;   // X dimension in voxels
    int EnvHeight_c;  // Y dimension in voxels
    int EnvDepth_c;   // Z dimension in voxels
    double VoxelSize; // Size of each voxel in meters
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

// Hash entry structure for 3D coordinates
typedef struct ENVHASHENTRY_3D
{
    int stateID;
    int X;
    int Y;
    int Z;
} EnvVoxel3DHashEntry_t;

// Environment variables
typedef struct ENVVOXEL3D
{
    ENVVOXEL3D() 
    {
        Coord2StateIDHashTable = nullptr;
    }

    int startstateid;
    int goalstateid;
    bool bInitialized;

    // Hash table for mapping coordinates to state IDs
    int HashTableSize;
    std::vector<EnvVoxel3DHashEntry_t*>* Coord2StateIDHashTable;

    // Vector mapping state IDs to coordinates
    std::vector<EnvVoxel3DHashEntry_t*> StateID2CoordTable;

    // Additional variables
} EnvironmentVoxel3D_t;

/**
 * \brief 3D Voxel Environment for SBPL path planning
 * Supports octomap integration and 3D path planning for UAVs
 */
class EnvironmentVoxel3D : public DiscreteSpaceInformation
{
public:
    /**
     * \brief Constructor
     */
    EnvironmentVoxel3D();

    /**
     * \brief Initialize environment from file
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief Initialize environment with 3D voxel grid
     */
    virtual bool InitializeEnv(int width, int height, int depth, 
                              double voxel_size, double origin_x, double origin_y, double origin_z);

    /**
     * \brief Initialize MDP config with start/goal IDs
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief Get heuristic from one state to another
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief Get heuristic to goal
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief Get heuristic from start
     */
    virtual int GetStartHeuristic(int stateID);

    /**
     * \brief Set all actions and outcomes for a state
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief Set all predecessors for a state
     */
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
     * \brief Get successors of a state
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    /**
     * \brief Get predecessors of a state
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief Get size of created environment
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief Print state information
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief Print environment configuration
     */
    virtual void PrintEnv_Config(FILE* fOut);

    /**
     * \brief Set start state
     */
    virtual int SetStart(int x, int y, int z);

    /**
     * \brief Set goal state
     */
    virtual int SetGoal(int x, int y, int z);

    /**
     * \brief Update cost of a voxel
     */
    virtual bool UpdateCost(int x, int y, int z, unsigned char newcost);

    /**
     * \brief Get coordinate from state ID
     */
    virtual void GetCoordFromState(int stateID, int& x, int& y, int& z) const;

    /**
     * \brief Get state ID from coordinate
     */
    virtual int GetStateFromCoord(int x, int y, int z) const;

    /**
     * \brief Check if voxel is obstacle
     */
    virtual bool IsObstacle(int x, int y, int z);

    /**
     * \brief Get map cost at voxel
     */
    virtual unsigned char GetMapCost(int x, int y, int z);

    /**
     * \brief Check if coordinate is within map bounds
     */
    virtual bool IsWithinMapCell(int X, int Y, int Z) const;

    /**
     * \brief Update environment from external octomap
     */
    virtual void UpdateFromOctomap(const std::shared_ptr<octomap::OcTree>& octomap);

    /**
     * \brief Insert point cloud into internal octomap
     */
    virtual void InsertPointCloud(const octomap::Pointcloud& pointcloud, 
                                 const octomap::point3d& sensor_origin,
                                 double max_range = -1.0);

    /**
     * \brief Get internal octomap
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
     * \brief Destructor
     */
    virtual ~EnvironmentVoxel3D();

    /**
     * \brief 从octomap .bt文件加载体素地图
     */
    bool LoadFromBTFile(const std::string& filename);

    /**
     * \brief 保存octomap体素地图到.bt文件
     */
    bool SaveToBTFile(const std::string& filename) const;

    /**
     * \brief 从octomap .bt文件和同名.yaml文件加载体素地图和环境参数
     */
    bool LoadFromBTFileWithConfig(const std::string& filename);

    /**
     * \brief 保存octomap体素地图到.bt文件和同名.yaml文件
     */
    bool SaveToBTFileWithConfig(const std::string& filename) const;

    /**
     * \brief 保存环境参数到YAML文件
     */
    bool SaveConfigToYAML(const std::string& filename) const;

    /**
     * \brief 从YAML文件加载环境参数
     */
    bool LoadConfigFromYAML(const std::string& filename);

protected:
    // Member data
    EnvVoxel3DConfig_t EnvVoxel3DCfg;
    EnvironmentVoxel3D_t EnvVoxel3D;
    
    // Internal octomap for voxel management
    std::shared_ptr<octomap::OcTree> octomap_;

    // Helper functions
    virtual void InitializeEnvironment();
    virtual void ComputeHeuristicValues();
    virtual void Computedxyz();
    virtual bool IsValidCell(int X, int Y, int Z) const;
    virtual unsigned int GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z) const;
    virtual void PrintHashTableHist();
    virtual EnvVoxel3DHashEntry_t* GetHashEntry(int X, int Y, int Z) const;
    virtual EnvVoxel3DHashEntry_t* CreateNewHashEntry(int X, int Y, int Z);
    virtual void ReadConfiguration(FILE* fCfg);
    virtual void InitializeEnvConfig();
    virtual void PrintTimeStat(FILE* fOut);
    virtual bool AreEquivalent(int StateID1, int StateID2);
    virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV);
    virtual void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV);
    virtual void GetRandomNeighs(int stateID, std::vector<int>* NeighIDV, std::vector<int>* CLowV, int nNumofNeighs, int nDist_c, bool bSuccs);
    virtual void SetConfiguration(int width, int height, int depth, const unsigned char*** voxel_data, int startx, int starty, int startz, int goalx, int goaly, int goalz);
    virtual bool InitGeneral();
    virtual void PrintHeader(FILE* fOut);
    virtual int cost(int state1coord[], int state2coord[]);
    virtual void PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal = false, FILE* fOut = NULL);
};

#endif // ENVIRONMENT_VOXEL3D_HPP