/*
 * 3D Voxel Environment for Path Planning - Implementation
 * Simplified version without SBPL dependency
 * Adapted for UAV path planning with octomap support
 * Loosely coupled with path planners
 */

#include "env/environment_voxel3d.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <yaml-cpp/yaml.h>

EnvironmentVoxel3D::EnvironmentVoxel3D()
    : octomap_(std::make_shared<octomap::OcTree>(0.1))
{
    InitializeEnvConfig();
    Computedxyz();
}

EnvironmentVoxel3D::~EnvironmentVoxel3D()
{
    // No cleanup needed - octomap is managed by shared_ptr
}

bool EnvironmentVoxel3D::InitializeEnv(int env_width, int env_height, int env_depth,
                                       double voxel_size, double origin_x, double origin_y, double origin_z)
{
    return InitializeEnv(env_width, env_height, env_depth, voxel_size, voxel_size, origin_x, origin_y, origin_z);
}

bool EnvironmentVoxel3D::InitializeEnv(int env_width, int env_height, int env_depth,
                                       double resolution_xy, double resolution_z,
                                       double origin_x, double origin_y, double origin_z)
{
    EnvVoxel3DCfg.Env_x = env_width;
    EnvVoxel3DCfg.Env_y = env_height;
    EnvVoxel3DCfg.Env_z = env_depth;
    EnvVoxel3DCfg.ResolutionXY = resolution_xy;
    EnvVoxel3DCfg.ResolutionZ = resolution_z;
    EnvVoxel3DCfg.OriginX = origin_x;
    EnvVoxel3DCfg.OriginY = origin_y;
    EnvVoxel3DCfg.OriginZ = origin_z;

    // Initialize octomap with the XY resolution
    octomap_ = std::make_shared<octomap::OcTree>(resolution_xy);

    return true;
}

bool EnvironmentVoxel3D::IsCellOccupied(int voxel_x, int voxel_y, int voxel_z)
{
    // First check if the cell is within map bounds
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return true; // Out of bounds cells are considered occupied for safety
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Query octomap
    octomap::point3d point(world_x, world_y, world_z);

    octomap::OcTreeNode *node = octomap_->search(point);

    if (!node)
    {
        // 没有节点，在 octomap 中的本义指 Unknown 区域，但在这里被用于表示 free 区域
        return false;
    }

    // Known region - check if occupied
    // return octomap_->isNodeOccupied(node);
    // 只要有节点，就视为 occupied （在 octomap 中，节点表示 occupied 和 free 区域）
    return true;
}

unsigned char EnvironmentVoxel3D::GetCellCost(int voxel_x, int voxel_y, int voxel_z)
{
    // First check if the cell is within map bounds
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return 255; // Out of bounds cells have maximum cost
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Query octomap
    octomap::point3d point(world_x, world_y, world_z);
    octomap::OcTreeNode *node = octomap_->search(point);

    if (node)
    {
        // Convert occupancy probability to cost (0-255)
        double prob = node->getOccupancy();
        return static_cast<unsigned char>(prob * 255);
    }
    else
    {
        return 0; // Free cells have minimum cost
    }
}

bool EnvironmentVoxel3D::IsCellWithinMap(int voxel_x, int voxel_y, int voxel_z) const
{
    return voxel_x >= 0 && voxel_x < EnvVoxel3DCfg.Env_x &&
           voxel_y >= 0 && voxel_y < EnvVoxel3DCfg.Env_y &&
           voxel_z >= 0 && voxel_z < EnvVoxel3DCfg.Env_z;
}

bool EnvironmentVoxel3D::UpdateCellCost(int voxel_x, int voxel_y, int voxel_z, unsigned char newcost)
{
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return false;
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Update octomap
    octomap::point3d point(world_x, world_y, world_z);

    // newcost 大于 0，表示 occupied，否则设置为free
    if (newcost > 0)
    {
        octomap_->setNodeValue(point, octomap::logodds(newcost));
    }
    else
    {
        octomap_->deleteNode(point);
    }

    return true;
}

void EnvironmentVoxel3D::UpdateFromOctomap(const std::shared_ptr<octomap::OcTree> &octomap)
{
    if (octomap)
    {
        octomap_ = octomap;
    }
}

void EnvironmentVoxel3D::InsertPointCloud(const octomap::Pointcloud &pointcloud,
                                          const octomap::point3d &sensor_origin)
{
    octomap_->insertPointCloud(pointcloud, sensor_origin);
    octomap_->updateInnerOccupancy();
}

bool EnvironmentVoxel3D::WorldToVoxel(double world_x, double world_y, double world_z,
                                      int &voxel_x, int &voxel_y, int &voxel_z) const
{
    voxel_x = static_cast<int>((world_x - EnvVoxel3DCfg.OriginX) / EnvVoxel3DCfg.ResolutionXY);
    voxel_y = static_cast<int>((world_y - EnvVoxel3DCfg.OriginY) / EnvVoxel3DCfg.ResolutionXY);
    voxel_z = static_cast<int>((world_z - EnvVoxel3DCfg.OriginZ) / EnvVoxel3DCfg.ResolutionZ);

    return IsCellWithinMap(voxel_x, voxel_y, voxel_z);
}

void EnvironmentVoxel3D::VoxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                                      double &world_x, double &world_y, double &world_z) const
{
    world_x = EnvVoxel3DCfg.OriginX + (voxel_x + 0.5) * EnvVoxel3DCfg.ResolutionXY;
    world_y = EnvVoxel3DCfg.OriginY + (voxel_y + 0.5) * EnvVoxel3DCfg.ResolutionXY;
    world_z = EnvVoxel3DCfg.OriginZ + (voxel_z + 0.5) * EnvVoxel3DCfg.ResolutionZ;
}

bool EnvironmentVoxel3D::LoadFromBTFile(const std::string &filename)
{
    try
    {
        octomap_ = std::make_shared<octomap::OcTree>(filename);
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading octomap from file: " << e.what() << std::endl;
        return false;
    }
}

bool EnvironmentVoxel3D::SaveToBTFile(const std::string &filename) const
{
    try
    {
        return octomap_->writeBinary(filename);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error saving octomap to file: " << e.what() << std::endl;
        return false;
    }
}

bool EnvironmentVoxel3D::LoadFromBTFileWithConfig(const std::string &filename)
{
    if (!LoadFromBTFile(filename))
    {
        return false;
    }

    // Try to load config from YAML file with same base name
    std::string config_file = filename.substr(0, filename.find_last_of('.')) + ".yaml";
    return LoadConfigFromYAML(config_file);
}

bool EnvironmentVoxel3D::SaveToBTFileWithConfig(const std::string &filename) const
{
    if (!SaveToBTFile(filename))
    {
        return false;
    }

    // Save config to YAML file with same base name
    std::string config_file = filename.substr(0, filename.find_last_of('.')) + ".yaml";
    return SaveConfigToYAML(config_file);
}

bool EnvironmentVoxel3D::SaveConfigToYAML(const std::string &filename) const
{
    try
    {
        YAML::Node config;
        config["env_width"] = EnvVoxel3DCfg.Env_x;
        config["env_height"] = EnvVoxel3DCfg.Env_y;
        config["env_depth"] = EnvVoxel3DCfg.Env_z;
        config["voxel_size"] = EnvVoxel3DCfg.ResolutionXY;
        config["voxel_size_z"] = EnvVoxel3DCfg.ResolutionZ;
        config["origin_x"] = EnvVoxel3DCfg.OriginX;
        config["origin_y"] = EnvVoxel3DCfg.OriginY;
        config["origin_z"] = EnvVoxel3DCfg.OriginZ;

        std::ofstream file(filename);
        file << config;
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error saving config to YAML: " << e.what() << std::endl;
        return false;
    }
}

bool EnvironmentVoxel3D::LoadConfigFromYAML(const std::string &filename)
{
    try
    {
        YAML::Node config = YAML::LoadFile(filename);

        EnvVoxel3DCfg.Env_x = config["env_width"].as<int>();
        EnvVoxel3DCfg.Env_y = config["env_height"].as<int>();
        EnvVoxel3DCfg.Env_z = config["env_depth"].as<int>();
        EnvVoxel3DCfg.ResolutionXY = config["voxel_size"].as<double>();
        EnvVoxel3DCfg.ResolutionZ = config["voxel_size_z"].as<double>();
        EnvVoxel3DCfg.OriginX = config["origin_x"].as<double>();
        EnvVoxel3DCfg.OriginY = config["origin_y"].as<double>();
        EnvVoxel3DCfg.OriginZ = config["origin_z"].as<double>();

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading config from YAML: " << e.what() << std::endl;
        return false;
    }
}

void EnvironmentVoxel3D::GetGridSize(int &size_x, int &size_y, int &size_z) const
{
    size_x = EnvVoxel3DCfg.Env_x;
    size_y = EnvVoxel3DCfg.Env_y;
    size_z = EnvVoxel3DCfg.Env_z;
}

void EnvironmentVoxel3D::GetEnvBounds(double &min_x, double &min_y, double &min_z,
                                      double &max_x, double &max_y, double &max_z) const
{
    min_x = EnvVoxel3DCfg.OriginX;
    min_y = EnvVoxel3DCfg.OriginY;
    min_z = EnvVoxel3DCfg.OriginZ;

    max_x = EnvVoxel3DCfg.OriginX + EnvVoxel3DCfg.Env_x * EnvVoxel3DCfg.ResolutionXY;
    max_y = EnvVoxel3DCfg.OriginY + EnvVoxel3DCfg.Env_y * EnvVoxel3DCfg.ResolutionXY;
    max_z = EnvVoxel3DCfg.OriginZ + EnvVoxel3DCfg.Env_z * EnvVoxel3DCfg.ResolutionZ;
}

double EnvironmentVoxel3D::GetResolutionXY() const
{
    return EnvVoxel3DCfg.ResolutionXY;
}

double EnvironmentVoxel3D::GetResolutionZ() const
{
    return EnvVoxel3DCfg.ResolutionZ;
}

void EnvironmentVoxel3D::Computedxyz()
{
    // Define 26-connectivity movement directions
    int idx = 0;
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;

                EnvVoxel3DCfg.dx_[idx] = dx;
                EnvVoxel3DCfg.dy_[idx] = dy;
                EnvVoxel3DCfg.dz_[idx] = dz;

                // Calculate distance (in mm for compatibility)
                double distance = std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;
                EnvVoxel3DCfg.dxy_distance_mm_[idx] = static_cast<int>(distance);

                idx++;
            }
        }
    }
    EnvVoxel3DCfg.numofdirs = 26;
}

bool EnvironmentVoxel3D::IsValidCell(int voxel_x, int voxel_y, int voxel_z) const
{
    return IsCellWithinMap(voxel_x, voxel_y, voxel_z);
}

void EnvironmentVoxel3D::InitializeEnvConfig()
{
    // Default configuration is already set in the constructor
}

bool EnvironmentVoxel3D::IsCellFree(int voxel_x, int voxel_y, int voxel_z)
{
    // First check if the cell is within map bounds
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return false; // Out of bounds cells are not considered free
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Query octomap
    octomap::point3d point(world_x, world_y, world_z);
    octomap::OcTreeNode *node = octomap_->search(point);

    if (!node)
    {
        // Unknown region - not free
        return false;
    }

    // Known region - check if free (not occupied)
    return !octomap_->isNodeOccupied(node);
}

bool EnvironmentVoxel3D::IsCellUnknown(int voxel_x, int voxel_y, int voxel_z)
{
    // First check if the cell is within map bounds
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return false; // Out of bounds cells are not unknown, they're invalid
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Query octomap
    octomap::point3d point(world_x, world_y, world_z);
    octomap::OcTreeNode *node = octomap_->search(point);

    // Unknown if no node exists in octomap
    return !node;
}

double EnvironmentVoxel3D::GetCellOccupancyProbability(int voxel_x, int voxel_y, int voxel_z)
{
    // First check if the cell is within map bounds
    if (!IsCellWithinMap(voxel_x, voxel_y, voxel_z))
    {
        return 1.0; // Out of bounds cells have maximum occupancy probability
    }

    // Convert voxel coordinates to world coordinates
    double world_x, world_y, world_z;
    VoxelToWorld(voxel_x, voxel_y, voxel_z, world_x, world_y, world_z);

    // Query octomap
    octomap::point3d point(world_x, world_y, world_z);
    octomap::OcTreeNode *node = octomap_->search(point);

    if (!node)
    {
        // Unknown region - return 0.5 (uncertain)
        return 0.5;
    }

    // Known region - return occupancy probability
    return node->getOccupancy();
}