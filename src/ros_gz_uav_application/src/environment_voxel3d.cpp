/*
 * 3D Voxel Environment for SBPL - Implementation
 * Based on MoveIt's Environment_Chain3D design
 * Adapted for UAV path planning with octomap support
 */

#include "environment_voxel3d.hpp"
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <cstring>

#define ENVVOXEL3D_COSTMULT 1000
#define ENVVOXEL3D_DEFAULTOBSTHRESH 1
#define ENVVOXEL3D_MAXDIRS 26

EnvironmentVoxel3D::EnvironmentVoxel3D()
{
    EnvVoxel3D.Coord2StateIDHashTable = nullptr;
    EnvVoxel3D.bInitialized = false;
    octomap_ = nullptr;
}

EnvironmentVoxel3D::~EnvironmentVoxel3D()
{
    // Clean up hash table
    if (EnvVoxel3D.Coord2StateIDHashTable != nullptr) {
        for (int i = 0; i < EnvVoxel3D.HashTableSize; i++) {
            delete EnvVoxel3D.Coord2StateIDHashTable->at(i);
        }
        delete EnvVoxel3D.Coord2StateIDHashTable;
    }
}

bool EnvironmentVoxel3D::InitializeEnv(const char* sEnvFile)
{
    // TODO: Implement file-based initialization
    // For now, return false as we prefer direct initialization
    std::cout << "EnvironmentVoxel3D::InitializeEnv(const char* sEnvFile) not implemented" << std::endl;
    // Try to load from BT file with config first
    std::string filename(sEnvFile);
    if (LoadFromBTFileWithConfig(filename)) {
        return true;
    }
    
    // If that fails, try loading just the BT file
    if (LoadFromBTFile(filename)) {
        return true;
    }
    
    // If both fail, print error and return false
    std::cout << "Failed to load environment from file: " << sEnvFile << std::endl;
    std::cout << "Tried both LoadFromBTFileWithConfig and LoadFromBTFile" << std::endl;
    
    return false;
}

bool EnvironmentVoxel3D::InitializeEnv(int width, int height, int depth, 
                                      double voxel_size, double origin_x, double origin_y, double origin_z)
{
    EnvVoxel3DCfg.EnvWidth_c = width;
    EnvVoxel3DCfg.EnvHeight_c = height;
    EnvVoxel3DCfg.EnvDepth_c = depth;
    EnvVoxel3DCfg.VoxelSize = voxel_size;
    EnvVoxel3DCfg.OriginX = origin_x;
    EnvVoxel3DCfg.OriginY = origin_y;
    EnvVoxel3DCfg.OriginZ = origin_z;
    EnvVoxel3DCfg.numofdirs = ENVVOXEL3D_MAXDIRS;

    // Initialize internal octomap
    octomap_ = std::make_shared<octomap::OcTree>(voxel_size);
    
    // Set octomap parameters
    octomap_->setProbHit(0.7);  // Hit probability
    octomap_->setProbMiss(0.4); // Miss probability
    octomap_->setClampingThresMin(0.1192); // Min threshold
    octomap_->setClampingThresMax(0.971);  // Max threshold

    // Initialize movement directions (26-connectivity)
    Computedxyz();

    // Initialize environment
    InitializeEnvironment();

    return true;
}

bool EnvironmentVoxel3D::InitializeMDPCfg(MDPConfig *MDPCfg)
{
    if (!EnvVoxel3D.bInitialized) {
        return false;
    }

    // Initialize MDP config
    MDPCfg->goalstateid = EnvVoxel3D.goalstateid;
    MDPCfg->startstateid = EnvVoxel3D.startstateid;

    return true;
}

int EnvironmentVoxel3D::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    // Manhattan distance heuristic for 3D
    int from_x, from_y, from_z, to_x, to_y, to_z;
    GetCoordFromState(FromStateID, from_x, from_y, from_z);
    GetCoordFromState(ToStateID, to_x, to_y, to_z);
    
    return (abs(from_x - to_x) + abs(from_y - to_y) + abs(from_z - to_z)) * ENVVOXEL3D_COSTMULT;
}

int EnvironmentVoxel3D::GetGoalHeuristic(int stateID)
{
    return GetFromToHeuristic(stateID, EnvVoxel3D.goalstateid);
}

int EnvironmentVoxel3D::GetStartHeuristic(int stateID)
{
    return GetFromToHeuristic(EnvVoxel3D.startstateid, stateID);
}

void EnvironmentVoxel3D::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    // This function is typically not used in SBPL
    // Successors are computed on-demand in GetSuccs
}

void EnvironmentVoxel3D::SetAllPreds(CMDPSTATE* state)
{
    // This function is typically not used in SBPL
    // Predecessors are computed on-demand in GetPreds
}

void EnvironmentVoxel3D::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV)
{
    SuccIDV->clear();
    CostV->clear();

    int source_x, source_y, source_z;
    GetCoordFromState(SourceStateID, source_x, source_y, source_z);

    // Check all 26 neighbors
    for (int dir = 0; dir < EnvVoxel3DCfg.numofdirs; dir++) {
        int new_x = source_x + EnvVoxel3DCfg.dx_[dir];
        int new_y = source_y + EnvVoxel3DCfg.dy_[dir];
        int new_z = source_z + EnvVoxel3DCfg.dz_[dir];

        // Check if neighbor is valid and not an obstacle
        if (IsValidCell(new_x, new_y, new_z) && !IsObstacle(new_x, new_y, new_z)) {
            int new_state_id = GetStateFromCoord(new_x, new_y, new_z);
            if (new_state_id >= 0) {
                SuccIDV->push_back(new_state_id);
                CostV->push_back(EnvVoxel3DCfg.dxy_distance_mm_[dir]);
            }
        }
    }
}

void EnvironmentVoxel3D::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
    PredIDV->clear();
    CostV->clear();

    int target_x, target_y, target_z;
    GetCoordFromState(TargetStateID, target_x, target_y, target_z);

    // Check all 26 neighbors (predecessors are the same as successors in undirected graph)
    for (int dir = 0; dir < EnvVoxel3DCfg.numofdirs; dir++) {
        int new_x = target_x + EnvVoxel3DCfg.dx_[dir];
        int new_y = target_y + EnvVoxel3DCfg.dy_[dir];
        int new_z = target_z + EnvVoxel3DCfg.dz_[dir];

        // Check if neighbor is valid and not an obstacle
        if (IsValidCell(new_x, new_y, new_z) && !IsObstacle(new_x, new_y, new_z)) {
            int new_state_id = GetStateFromCoord(new_x, new_y, new_z);
            if (new_state_id >= 0) {
                PredIDV->push_back(new_state_id);
                CostV->push_back(EnvVoxel3DCfg.dxy_distance_mm_[dir]);
            }
        }
    }
}

int EnvironmentVoxel3D::SizeofCreatedEnv()
{
    return EnvVoxel3D.StateID2CoordTable.size();
}

void EnvironmentVoxel3D::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    if (fOut == NULL) {
        fOut = stdout;
    }

    int x, y, z;
    GetCoordFromState(stateID, x, y, z);
    fprintf(fOut, "State %d: (%d, %d, %d)", stateID, x, y, z);
    
    if (bVerbose) {
        fprintf(fOut, " - Cost: %d", GetMapCost(x, y, z));
    }
    fprintf(fOut, "\n");
}

void EnvironmentVoxel3D::PrintEnv_Config(FILE* fOut)
{
    if (fOut == NULL) {
        fOut = stdout;
    }

    fprintf(fOut, "Environment Configuration:\n");
    fprintf(fOut, "  Dimensions: %d x %d x %d\n", 
            EnvVoxel3DCfg.EnvWidth_c, EnvVoxel3DCfg.EnvHeight_c, EnvVoxel3DCfg.EnvDepth_c);
    fprintf(fOut, "  Voxel Size: %.3f m\n", EnvVoxel3DCfg.VoxelSize);
    fprintf(fOut, "  Origin: (%.3f, %.3f, %.3f)\n", 
            EnvVoxel3DCfg.OriginX, EnvVoxel3DCfg.OriginY, EnvVoxel3DCfg.OriginZ);
    fprintf(fOut, "  Movement Directions: %d\n", EnvVoxel3DCfg.numofdirs);
    fprintf(fOut, "  Using OcTree for voxel management\n");
}

int EnvironmentVoxel3D::SetStart(int x, int y, int z)
{
    if (!IsValidCell(x, y, z)) {
        return -1;
    }

    EnvVoxel3D.startstateid = GetStateFromCoord(x, y, z);
    return EnvVoxel3D.startstateid;
}

int EnvironmentVoxel3D::SetGoal(int x, int y, int z)
{
    if (!IsValidCell(x, y, z)) {
        return -1;
    }

    EnvVoxel3D.goalstateid = GetStateFromCoord(x, y, z);
    return EnvVoxel3D.goalstateid;
}

bool EnvironmentVoxel3D::UpdateCost(int x, int y, int z, unsigned char newcost)
{
    if (!IsValidCell(x, y, z)) {
        return false;
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(x, y, z, world_x, world_y, world_z);

    // Update octomap
    if (newcost > ENVVOXEL3D_DEFAULTOBSTHRESH) {
        octomap_->updateNode(world_x, world_y, world_z, true);
    } else {
        octomap_->updateNode(world_x, world_y, world_z, false);
    }

    return true;
}

void EnvironmentVoxel3D::GetCoordFromState(int stateID, int& x, int& y, int& z) const
{
    if (stateID < 0 || stateID >= static_cast<int>(EnvVoxel3D.StateID2CoordTable.size())) {
        x = y = z = -1;
        return;
    }

    EnvVoxel3DHashEntry_t* entry = EnvVoxel3D.StateID2CoordTable[stateID];
    x = entry->X;
    y = entry->Y;
    z = entry->Z;
}

int EnvironmentVoxel3D::GetStateFromCoord(int x, int y, int z) const
{
    if (!IsValidCell(x, y, z)) {
        return -1;
    }

    EnvVoxel3DHashEntry_t* entry = GetHashEntry(x, y, z);
    if (entry == nullptr) {
        // Cannot create new entry in const function, return -1
        return -1;
    }

    return entry->stateID;
}

bool EnvironmentVoxel3D::IsObstacle(int x, int y, int z)
{
    if (!IsValidCell(x, y, z)) {
        return true; // Out of bounds is considered obstacle
    }

    return (GetMapCost(x, y, z) >= ENVVOXEL3D_DEFAULTOBSTHRESH);
}

unsigned char EnvironmentVoxel3D::GetMapCost(int x, int y, int z)
{
    if (!IsValidCell(x, y, z)) {
        return 255; // Maximum cost for invalid cells
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(x, y, z, world_x, world_y, world_z);

    // Query octomap
    octomap::OcTreeNode* node = octomap_->search(world_x, world_y, world_z);
    if (node) {
        if (octomap_->isNodeOccupied(node)) {
            return 255; // Occupied
        } else {
            return 0;   // Free
        }
    }
    
    return 128; // Unknown
}

bool EnvironmentVoxel3D::IsWithinMapCell(int X, int Y, int Z) const
{
    return (X >= 0 && X < EnvVoxel3DCfg.EnvWidth_c &&
            Y >= 0 && Y < EnvVoxel3DCfg.EnvHeight_c &&
            Z >= 0 && Z < EnvVoxel3DCfg.EnvDepth_c);
}

void EnvironmentVoxel3D::UpdateFromOctomap(const std::shared_ptr<octomap::OcTree>& external_octomap)
{
    if (!external_octomap || !EnvVoxel3D.bInitialized || !octomap_) {
        return;
    }

    // Clear current octomap
    octomap_->clear();

    // Copy occupied nodes from external octomap
    for (octomap::OcTree::leaf_iterator it = external_octomap->begin_leafs(); it != external_octomap->end_leafs(); ++it) {
        if (external_octomap->isNodeOccupied(*it)) {
            double world_x = it.getX();
            double world_y = it.getY();
            double world_z = it.getZ();

            // Check if within our environment bounds
            int voxel_x, voxel_y, voxel_z;
            if (WorldToVoxel(world_x, world_y, world_z, voxel_x, voxel_y, voxel_z)) {
                if (IsValidCell(voxel_x, voxel_y, voxel_z)) {
                    octomap_->updateNode(world_x, world_y, world_z, true);
                }
            }
        }
    }

    // Update internal nodes
    octomap_->updateInnerOccupancy();
}

void EnvironmentVoxel3D::InsertPointCloud(const octomap::Pointcloud& pointcloud, 
                                         const octomap::point3d& sensor_origin,
                                         double max_range)
{
    if (!octomap_ || !EnvVoxel3D.bInitialized) {
        return;
    }

    // Insert point cloud into octomap
    if (max_range > 0) {
        octomap_->insertPointCloud(pointcloud, sensor_origin, max_range);
    } else {
        octomap_->insertPointCloud(pointcloud, sensor_origin);
    }

    // Update internal nodes
    octomap_->updateInnerOccupancy();
}

bool EnvironmentVoxel3D::WorldToVoxel(double world_x, double world_y, double world_z, 
                                      int& voxel_x, int& voxel_y, int& voxel_z) const
{
    voxel_x = static_cast<int>((world_x - EnvVoxel3DCfg.OriginX) / EnvVoxel3DCfg.VoxelSize);
    voxel_y = static_cast<int>((world_y - EnvVoxel3DCfg.OriginY) / EnvVoxel3DCfg.VoxelSize);
    voxel_z = static_cast<int>((world_z - EnvVoxel3DCfg.OriginZ) / EnvVoxel3DCfg.VoxelSize);

    return IsValidCell(voxel_x, voxel_y, voxel_z);
}

void EnvironmentVoxel3D::VoxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                                      double& world_x, double& world_y, double& world_z) const
{
    world_x = EnvVoxel3DCfg.OriginX + voxel_x * EnvVoxel3DCfg.VoxelSize;
    world_y = EnvVoxel3DCfg.OriginY + voxel_y * EnvVoxel3DCfg.VoxelSize;
    world_z = EnvVoxel3DCfg.OriginZ + voxel_z * EnvVoxel3DCfg.VoxelSize;
}

void EnvironmentVoxel3D::InitializeEnvironment()
{
    // Initialize hash table
    EnvVoxel3D.HashTableSize = EnvVoxel3DCfg.EnvWidth_c * EnvVoxel3DCfg.EnvHeight_c * EnvVoxel3DCfg.EnvDepth_c;
    EnvVoxel3D.Coord2StateIDHashTable = new std::vector<EnvVoxel3DHashEntry_t*>[EnvVoxel3D.HashTableSize];
    EnvVoxel3D.StateID2CoordTable.clear();

    EnvVoxel3D.bInitialized = true;
}

void EnvironmentVoxel3D::ComputeHeuristicValues()
{
    // Heuristic computation is done on-demand in GetGoalHeuristic
}

void EnvironmentVoxel3D::Computedxyz()
{
    // Initialize 26-connectivity movement directions
    int idx = 0;
    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dz = -1; dz <= 1; dz++) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue; // Skip the center point
                }

                EnvVoxel3DCfg.dx_[idx] = dx;
                EnvVoxel3DCfg.dy_[idx] = dy;
                EnvVoxel3DCfg.dz_[idx] = dz;

                // Calculate distance (diagonal = sqrt(3), face = sqrt(2), edge = 1)
                double distance = sqrt(dx*dx + dy*dy + dz*dz);
                EnvVoxel3DCfg.dxy_distance_mm_[idx] = static_cast<int>(distance * ENVVOXEL3D_COSTMULT);

                idx++;
            }
        }
    }
}

bool EnvironmentVoxel3D::IsValidCell(int X, int Y, int Z) const
{
    return IsWithinMapCell(X, Y, Z);
}

unsigned int EnvironmentVoxel3D::GETHASHBIN(unsigned int X, unsigned int Y, unsigned int Z) const
{
    // Simple hash function for 3D coordinates
    return (X + Y * EnvVoxel3DCfg.EnvWidth_c + Z * EnvVoxel3DCfg.EnvWidth_c * EnvVoxel3DCfg.EnvHeight_c) % EnvVoxel3D.HashTableSize;
}

void EnvironmentVoxel3D::PrintHashTableHist()
{
    // Implementation for hash table statistics
}

EnvVoxel3DHashEntry_t* EnvironmentVoxel3D::GetHashEntry(int X, int Y, int Z) const
{
    unsigned int bin = GETHASHBIN(X, Y, Z);
    
    for (auto entry : EnvVoxel3D.Coord2StateIDHashTable[bin]) {
        if (entry->X == X && entry->Y == Y && entry->Z == Z) {
            return entry;
        }
    }
    
    return nullptr;
}

EnvVoxel3DHashEntry_t* EnvironmentVoxel3D::CreateNewHashEntry(int X, int Y, int Z)
{
    EnvVoxel3DHashEntry_t* entry = new EnvVoxel3DHashEntry_t;
    entry->X = X;
    entry->Y = Y;
    entry->Z = Z;
    entry->stateID = EnvVoxel3D.StateID2CoordTable.size();

    // Add to hash table
    unsigned int bin = GETHASHBIN(X, Y, Z);
    EnvVoxel3D.Coord2StateIDHashTable[bin].push_back(entry);

    // Add to state ID table
    EnvVoxel3D.StateID2CoordTable.push_back(entry);

    return entry;
}

// Stub implementations for remaining virtual functions
void EnvironmentVoxel3D::ReadConfiguration(FILE* fCfg) {}
void EnvironmentVoxel3D::InitializeEnvConfig() {}
void EnvironmentVoxel3D::PrintTimeStat(FILE* fOut) {}
bool EnvironmentVoxel3D::AreEquivalent(int StateID1, int StateID2) { return StateID1 == StateID2; }
void EnvironmentVoxel3D::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV) {}
void EnvironmentVoxel3D::GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV) {}
void EnvironmentVoxel3D::GetRandomNeighs(int stateID, std::vector<int>* NeighIDV, std::vector<int>* CLowV, int nNumofNeighs, int nDist_c, bool bSuccs) {}
void EnvironmentVoxel3D::SetConfiguration(int width, int height, int depth, const unsigned char*** voxel_data, int startx, int starty, int startz, int goalx, int goaly, int goalz) {}
bool EnvironmentVoxel3D::InitGeneral() { return true; }
void EnvironmentVoxel3D::PrintHeader(FILE* fOut) {}
int EnvironmentVoxel3D::cost(int state1coord[], int state2coord[]) { return ENVVOXEL3D_COSTMULT; }
void EnvironmentVoxel3D::PrintSuccGoal(int SourceStateID, int costtogoal, bool bVerbose, bool bLocal, FILE* fOut) {}

bool EnvironmentVoxel3D::LoadFromBTFile(const std::string& filename) {
    if (!octomap_) return false;
    return octomap_->readBinary(filename);
}

bool EnvironmentVoxel3D::SaveToBTFile(const std::string& filename) const {
    if (!octomap_) return false;
    return octomap_->writeBinary(filename);
}

bool EnvironmentVoxel3D::LoadFromBTFileWithConfig(const std::string& filename) {
    // 首先加载.bt文件
    if (!LoadFromBTFile(filename)) {
        return false;
    }
    
    // 构造同名的.yaml文件路径
    std::string yaml_filename = filename;
    size_t dot_pos = yaml_filename.find_last_of('.');
    if (dot_pos != std::string::npos) {
        yaml_filename = yaml_filename.substr(0, dot_pos) + ".yaml";
    } else {
        yaml_filename += ".yaml";
    }
    
    // 尝试加载YAML配置文件
    if (LoadConfigFromYAML(yaml_filename)) {
        // 如果YAML文件存在且加载成功，使用YAML中的参数
        // 否则保持octomap的默认参数
    }
    
    return true;
}

bool EnvironmentVoxel3D::SaveToBTFileWithConfig(const std::string& filename) const {
    // 首先保存.bt文件
    if (!SaveToBTFile(filename)) {
        return false;
    }
    
    // 构造同名的.yaml文件路径
    std::string yaml_filename = filename;
    size_t dot_pos = yaml_filename.find_last_of('.');
    if (dot_pos != std::string::npos) {
        yaml_filename = yaml_filename.substr(0, dot_pos) + ".yaml";
    } else {
        yaml_filename += ".yaml";
    }
    
    // 保存YAML配置文件
    return SaveConfigToYAML(yaml_filename);
}

bool EnvironmentVoxel3D::SaveConfigToYAML(const std::string& filename) const {
    try {
        YAML::Node config;
        
        // 保存环境配置
        config["environment"] = YAML::Node();
        config["environment"]["width"] = EnvVoxel3DCfg.EnvWidth_c;
        config["environment"]["height"] = EnvVoxel3DCfg.EnvHeight_c;
        config["environment"]["depth"] = EnvVoxel3DCfg.EnvDepth_c;
        config["environment"]["voxel_size"] = EnvVoxel3DCfg.VoxelSize;
        config["environment"]["origin_x"] = EnvVoxel3DCfg.OriginX;
        config["environment"]["origin_y"] = EnvVoxel3DCfg.OriginY;
        config["environment"]["origin_z"] = EnvVoxel3DCfg.OriginZ;
        config["environment"]["num_directions"] = EnvVoxel3DCfg.numofdirs;
        
        // 保存octomap参数（如果octomap存在）
        if (octomap_) {
            config["octomap"] = YAML::Node();
            config["octomap"]["resolution"] = octomap_->getResolution();
            config["octomap"]["prob_hit"] = octomap_->getProbHit();
            config["octomap"]["prob_miss"] = octomap_->getProbMiss();
            config["octomap"]["clamping_thres_min"] = octomap_->getClampingThresMin();
            config["octomap"]["clamping_thres_max"] = octomap_->getClampingThresMax();
        }
        
        // 保存到文件
        std::ofstream fout(filename);
        if (!fout.is_open()) {
            return false;
        }
        
        fout << config;
        fout.close();
        
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool EnvironmentVoxel3D::LoadConfigFromYAML(const std::string& filename) {
    try {
        YAML::Node config = YAML::LoadFile(filename);
        
        // 加载环境配置
        if (config["environment"]) {
            YAML::Node env_config = config["environment"];
            
            if (env_config["width"]) EnvVoxel3DCfg.EnvWidth_c = env_config["width"].as<int>();
            if (env_config["height"]) EnvVoxel3DCfg.EnvHeight_c = env_config["height"].as<int>();
            if (env_config["depth"]) EnvVoxel3DCfg.EnvDepth_c = env_config["depth"].as<int>();
            if (env_config["voxel_size"]) EnvVoxel3DCfg.VoxelSize = env_config["voxel_size"].as<double>();
            if (env_config["origin_x"]) EnvVoxel3DCfg.OriginX = env_config["origin_x"].as<double>();
            if (env_config["origin_y"]) EnvVoxel3DCfg.OriginY = env_config["origin_y"].as<double>();
            if (env_config["origin_z"]) EnvVoxel3DCfg.OriginZ = env_config["origin_z"].as<double>();
            if (env_config["num_directions"]) EnvVoxel3DCfg.numofdirs = env_config["num_directions"].as<int>();
        }
        
        // 加载octomap参数（如果octomap存在且配置中有octomap节点）
        if (octomap_ && config["octomap"]) {
            YAML::Node octomap_config = config["octomap"];
            
            if (octomap_config["prob_hit"]) octomap_->setProbHit(octomap_config["prob_hit"].as<double>());
            if (octomap_config["prob_miss"]) octomap_->setProbMiss(octomap_config["prob_miss"].as<double>());
            if (octomap_config["clamping_thres_min"]) octomap_->setClampingThresMin(octomap_config["clamping_thres_min"].as<double>());
            if (octomap_config["clamping_thres_max"]) octomap_->setClampingThresMax(octomap_config["clamping_thres_max"].as<double>());
        }
        
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}