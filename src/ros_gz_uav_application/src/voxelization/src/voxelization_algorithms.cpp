#include "voxelization_algorithms.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <omp.h>

namespace voxelization {

// CPUSequentialVoxelization implementation
CPUSequentialVoxelization::CPUSequentialVoxelization() 
    : grid_x_(0), grid_y_(0), grid_z_(0), resolution_xy_(0), resolution_z_(0),
      origin_x_(0), origin_y_(0), origin_z_(0) {}

CPUSequentialVoxelization::~CPUSequentialVoxelization() = default;

void CPUSequentialVoxelization::initialize(int grid_x, int grid_y, int grid_z,
                                         double resolution_xy, double resolution_z,
                                         double origin_x, double origin_y, double origin_z) {
    grid_x_ = grid_x;
    grid_y_ = grid_y;
    grid_z_ = grid_z;
    resolution_xy_ = resolution_xy;
    resolution_z_ = resolution_z;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    origin_z_ = origin_z;
    
    voxel_grid_.resize(grid_x_ * grid_y_ * grid_z_, 0);
    clear();
}

int CPUSequentialVoxelization::voxelizeEntity(const std::shared_ptr<SpatialEntity>& entity,
                                             double buffer_size, unsigned char cost_value) {
    if (!entity) return 0;
    
    auto bbox = entity->getBoundingBox();
    if (bbox.size() < 6) return 0;
    
    // Expand bounding box by buffer size
    double min_x = bbox[0] - buffer_size, max_x = bbox[3] + buffer_size;
    double min_y = bbox[1] - buffer_size, max_y = bbox[4] + buffer_size;
    double min_z = bbox[2] - buffer_size, max_z = bbox[5] + buffer_size;
    
    // Convert to voxel coordinates
    int voxel_min_x, voxel_min_y, voxel_min_z, voxel_max_x, voxel_max_y, voxel_max_z;
    
    if (!worldToVoxel(min_x, min_y, min_z, voxel_min_x, voxel_min_y, voxel_min_z) ||
        !worldToVoxel(max_x, max_y, max_z, voxel_max_x, voxel_max_y, voxel_max_z)) {
        return 0;
    }
    
    // Clamp to grid bounds
    voxel_min_x = std::max(0, std::min(voxel_min_x, grid_x_ - 1));
    voxel_min_y = std::max(0, std::min(voxel_min_y, grid_y_ - 1));
    voxel_min_z = std::max(0, std::min(voxel_min_z, grid_z_ - 1));
    voxel_max_x = std::max(0, std::min(voxel_max_x, grid_x_ - 1));
    voxel_max_y = std::max(0, std::min(voxel_max_y, grid_y_ - 1));
    voxel_max_z = std::max(0, std::min(voxel_max_z, grid_z_ - 1));
    
    // Mark voxels based on entity type
    std::string entity_type = entity->getType();
    int marked_count = 0;
    
    if (entity_type == "box") {
        marked_count = markBoxEntity(entity, buffer_size, cost_value);
    } else if (entity_type == "cylinder") {
        marked_count = markCylinderEntity(entity, buffer_size, cost_value);
    } else if (entity_type == "sphere") {
        marked_count = markSphereEntity(entity, buffer_size, cost_value);
    } else if (entity_type == "mesh") {
        marked_count = markMeshEntity(entity, buffer_size, cost_value);
    } else {
        // Generic marking for other entity types
        for (int x = voxel_min_x; x <= voxel_max_x; ++x) {
            for (int y = voxel_min_y; y <= voxel_max_y; ++y) {
                for (int z = voxel_min_z; z <= voxel_max_z; ++z) {
                    double world_x, world_y, world_z;
                    voxelToWorld(x, y, z, world_x, world_y, world_z);
                    
                    if (entity->isPointInside(world_x, world_y, world_z)) {
                        updateVoxelCost(x, y, z, cost_value);
                        marked_count++;
                    }
                }
            }
        }
    }
    
    return marked_count;
}

int CPUSequentialVoxelization::voxelizeEntities(const std::vector<std::shared_ptr<SpatialEntity>>& entities,
                                               double buffer_size, unsigned char cost_value) {
    int total_marked = 0;
    
    for (const auto& entity : entities) {
        total_marked += voxelizeEntity(entity, buffer_size, cost_value);
    }
    
    return total_marked;
}

bool CPUSequentialVoxelization::worldToVoxel(double world_x, double world_y, double world_z,
                                            int& voxel_x, int& voxel_y, int& voxel_z) const {
    voxel_x = static_cast<int>((world_x - origin_x_) / resolution_xy_);
    voxel_y = static_cast<int>((world_y - origin_y_) / resolution_xy_);
    voxel_z = static_cast<int>((world_z - origin_z_) / resolution_z_);
    
    return isValidVoxel(voxel_x, voxel_y, voxel_z);
}

void CPUSequentialVoxelization::voxelToWorld(int voxel_x, int voxel_y, int voxel_z,
                                            double& world_x, double& world_y, double& world_z) const {
    world_x = origin_x_ + voxel_x * resolution_xy_;
    world_y = origin_y_ + voxel_y * resolution_xy_;
    world_z = origin_z_ + voxel_z * resolution_z_;
}

bool CPUSequentialVoxelization::isValidVoxel(int voxel_x, int voxel_y, int voxel_z) const {
    return (voxel_x >= 0 && voxel_x < grid_x_ &&
            voxel_y >= 0 && voxel_y < grid_y_ &&
            voxel_z >= 0 && voxel_z < grid_z_);
}

void CPUSequentialVoxelization::updateVoxelCost(int voxel_x, int voxel_y, int voxel_z, unsigned char cost_value) {
    if (isValidVoxel(voxel_x, voxel_y, voxel_z)) {
        size_t index = voxel_z * grid_x_ * grid_y_ + voxel_y * grid_x_ + voxel_x;
        voxel_grid_[index] = cost_value;
    }
}

unsigned char CPUSequentialVoxelization::getVoxelCost(int voxel_x, int voxel_y, int voxel_z) const {
    if (isValidVoxel(voxel_x, voxel_y, voxel_z)) {
        size_t index = voxel_z * grid_x_ * grid_y_ + voxel_y * grid_x_ + voxel_x;
        return voxel_grid_[index];
    }
    return 0;
}

void CPUSequentialVoxelization::clear() {
    std::fill(voxel_grid_.begin(), voxel_grid_.end(), 0);
}

bool CPUSequentialVoxelization::saveToFile(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) return false;
    
    // Write header
    file.write(reinterpret_cast<const char*>(&grid_x_), sizeof(int));
    file.write(reinterpret_cast<const char*>(&grid_y_), sizeof(int));
    file.write(reinterpret_cast<const char*>(&grid_z_), sizeof(int));
    file.write(reinterpret_cast<const char*>(&resolution_xy_), sizeof(double));
    file.write(reinterpret_cast<const char*>(&resolution_z_), sizeof(double));
    file.write(reinterpret_cast<const char*>(&origin_x_), sizeof(double));
    file.write(reinterpret_cast<const char*>(&origin_y_), sizeof(double));
    file.write(reinterpret_cast<const char*>(&origin_z_), sizeof(double));
    
    // Write voxel data
    file.write(reinterpret_cast<const char*>(voxel_grid_.data()), voxel_grid_.size());
    
    return true;
}

bool CPUSequentialVoxelization::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) return false;
    
    // Read header
    file.read(reinterpret_cast<char*>(&grid_x_), sizeof(int));
    file.read(reinterpret_cast<char*>(&grid_y_), sizeof(int));
    file.read(reinterpret_cast<char*>(&grid_z_), sizeof(int));
    file.read(reinterpret_cast<char*>(&resolution_xy_), sizeof(double));
    file.read(reinterpret_cast<char*>(&resolution_z_), sizeof(double));
    file.read(reinterpret_cast<char*>(&origin_x_), sizeof(double));
    file.read(reinterpret_cast<char*>(&origin_y_), sizeof(double));
    file.read(reinterpret_cast<char*>(&origin_z_), sizeof(double));
    
    // Resize and read voxel data
    voxel_grid_.resize(grid_x_ * grid_y_ * grid_z_);
    file.read(reinterpret_cast<char*>(voxel_grid_.data()), voxel_grid_.size());
    
    return true;
}

// Type-specific marking methods
int CPUSequentialVoxelization::markBoxEntity(const std::shared_ptr<SpatialEntity>& entity,
                                            double buffer_size, unsigned char cost_value) {
    auto props = entity->getProperties();
    double center_x = props["center_x"];
    double center_y = props["center_y"];
    double center_z = props["center_z"];
    double size_x = props["size_x"];
    double size_y = props["size_y"];
    double size_z = props["size_z"];
    
    // Calculate bounds
    double min_x = center_x - size_x/2 - buffer_size;
    double max_x = center_x + size_x/2 + buffer_size;
    double min_y = center_y - size_y/2 - buffer_size;
    double max_y = center_y + size_y/2 + buffer_size;
    double min_z = center_z - size_z/2 - buffer_size;
    double max_z = center_z + size_z/2 + buffer_size;
    
    // Convert to voxel coordinates
    int voxel_min_x, voxel_min_y, voxel_min_z, voxel_max_x, voxel_max_y, voxel_max_z;
    worldToVoxel(min_x, min_y, min_z, voxel_min_x, voxel_min_y, voxel_min_z);
    worldToVoxel(max_x, max_y, max_z, voxel_max_x, voxel_max_y, voxel_max_z);
    
    // Clamp bounds
    voxel_min_x = std::max(0, std::min(voxel_min_x, grid_x_ - 1));
    voxel_min_y = std::max(0, std::min(voxel_min_y, grid_y_ - 1));
    voxel_min_z = std::max(0, std::min(voxel_min_z, grid_z_ - 1));
    voxel_max_x = std::max(0, std::min(voxel_max_x, grid_x_ - 1));
    voxel_max_y = std::max(0, std::min(voxel_max_y, grid_y_ - 1));
    voxel_max_z = std::max(0, std::min(voxel_max_z, grid_z_ - 1));
    
    int marked_count = 0;
    
    for (int x = voxel_min_x; x <= voxel_max_x; ++x) {
        for (int y = voxel_min_y; y <= voxel_max_y; ++y) {
            for (int z = voxel_min_z; z <= voxel_max_z; ++z) {
                double world_x, world_y, world_z;
                voxelToWorld(x, y, z, world_x, world_y, world_z);
                
                if (world_x >= min_x && world_x <= max_x &&
                    world_y >= min_y && world_y <= max_y &&
                    world_z >= min_z && world_z <= max_z) {
                    updateVoxelCost(x, y, z, cost_value);
                    marked_count++;
                }
            }
        }
    }
    
    return marked_count;
}

int CPUSequentialVoxelization::markCylinderEntity(const std::shared_ptr<SpatialEntity>& entity,
                                                 double buffer_size, unsigned char cost_value) {
    auto props = entity->getProperties();
    double center_x = props["center_x"];
    double center_y = props["center_y"];
    double center_z = props["center_z"];
    double radius = props["radius"];
    double height = props["height"];
    
    double radius_sq = (radius + buffer_size) * (radius + buffer_size);
    double min_z = center_z - height/2 - buffer_size;
    double max_z = center_z + height/2 + buffer_size;
    
    // Calculate voxel bounds
    double min_x = center_x - radius - buffer_size;
    double max_x = center_x + radius + buffer_size;
    double min_y = center_y - radius - buffer_size;
    double max_y = center_y + radius + buffer_size;
    
    int voxel_min_x, voxel_min_y, voxel_min_z, voxel_max_x, voxel_max_y, voxel_max_z;
    worldToVoxel(min_x, min_y, min_z, voxel_min_x, voxel_min_y, voxel_min_z);
    worldToVoxel(max_x, max_y, max_z, voxel_max_x, voxel_max_y, voxel_max_z);
    
    // Clamp bounds
    voxel_min_x = std::max(0, std::min(voxel_min_x, grid_x_ - 1));
    voxel_min_y = std::max(0, std::min(voxel_min_y, grid_y_ - 1));
    voxel_min_z = std::max(0, std::min(voxel_min_z, grid_z_ - 1));
    voxel_max_x = std::max(0, std::min(voxel_max_x, grid_x_ - 1));
    voxel_max_y = std::max(0, std::min(voxel_max_y, grid_y_ - 1));
    voxel_max_z = std::max(0, std::min(voxel_max_z, grid_z_ - 1));
    
    int marked_count = 0;
    
    for (int x = voxel_min_x; x <= voxel_max_x; ++x) {
        for (int y = voxel_min_y; y <= voxel_max_y; ++y) {
            double world_x, world_y, world_z;
            voxelToWorld(x, y, voxel_min_z, world_x, world_y, world_z);
            
            double dx = world_x - center_x;
            double dy = world_y - center_y;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq <= radius_sq) {
                for (int z = voxel_min_z; z <= voxel_max_z; ++z) {
                    voxelToWorld(x, y, z, world_x, world_y, world_z);
                    if (world_z >= min_z && world_z <= max_z) {
                        updateVoxelCost(x, y, z, cost_value);
                        marked_count++;
                    }
                }
            }
        }
    }
    
    return marked_count;
}

int CPUSequentialVoxelization::markSphereEntity(const std::shared_ptr<SpatialEntity>& entity,
                                               double buffer_size, unsigned char cost_value) {
    auto props = entity->getProperties();
    double center_x = props["center_x"];
    double center_y = props["center_y"];
    double center_z = props["center_z"];
    double radius = props["radius"];
    
    double radius_sq = (radius + buffer_size) * (radius + buffer_size);
    
    // Calculate voxel bounds
    double min_x = center_x - radius - buffer_size;
    double max_x = center_x + radius + buffer_size;
    double min_y = center_y - radius - buffer_size;
    double max_y = center_y + radius + buffer_size;
    double min_z = center_z - radius - buffer_size;
    double max_z = center_z + radius + buffer_size;
    
    int voxel_min_x, voxel_min_y, voxel_min_z, voxel_max_x, voxel_max_y, voxel_max_z;
    worldToVoxel(min_x, min_y, min_z, voxel_min_x, voxel_min_y, voxel_min_z);
    worldToVoxel(max_x, max_y, max_z, voxel_max_x, voxel_max_y, voxel_max_z);
    
    // Clamp bounds
    voxel_min_x = std::max(0, std::min(voxel_min_x, grid_x_ - 1));
    voxel_min_y = std::max(0, std::min(voxel_min_y, grid_y_ - 1));
    voxel_min_z = std::max(0, std::min(voxel_min_z, grid_z_ - 1));
    voxel_max_x = std::max(0, std::min(voxel_max_x, grid_x_ - 1));
    voxel_max_y = std::max(0, std::min(voxel_max_y, grid_y_ - 1));
    voxel_max_z = std::max(0, std::min(voxel_max_z, grid_z_ - 1));
    
    int marked_count = 0;
    
    for (int x = voxel_min_x; x <= voxel_max_x; ++x) {
        for (int y = voxel_min_y; y <= voxel_max_y; ++y) {
            for (int z = voxel_min_z; z <= voxel_max_z; ++z) {
                double world_x, world_y, world_z;
                voxelToWorld(x, y, z, world_x, world_y, world_z);
                
                double dx = world_x - center_x;
                double dy = world_y - center_y;
                double dz = world_z - center_z;
                double dist_sq = dx*dx + dy*dy + dz*dz;
                
                if (dist_sq <= radius_sq) {
                    updateVoxelCost(x, y, z, cost_value);
                    marked_count++;
                }
            }
        }
    }
    
    return marked_count;
}

int CPUSequentialVoxelization::markMeshEntity(const std::shared_ptr<SpatialEntity>& entity,
                                             double buffer_size, unsigned char cost_value) {
    // For mesh entities, use the generic point-in-entity test
    auto bbox = entity->getBoundingBox();
    if (bbox.size() < 6) return 0;
    
    double min_x = bbox[0] - buffer_size, max_x = bbox[3] + buffer_size;
    double min_y = bbox[1] - buffer_size, max_y = bbox[4] + buffer_size;
    double min_z = bbox[2] - buffer_size, max_z = bbox[5] + buffer_size;
    
    int voxel_min_x, voxel_min_y, voxel_min_z, voxel_max_x, voxel_max_y, voxel_max_z;
    worldToVoxel(min_x, min_y, min_z, voxel_min_x, voxel_min_y, voxel_min_z);
    worldToVoxel(max_x, max_y, max_z, voxel_max_x, voxel_max_y, voxel_max_z);
    
    // Clamp bounds
    voxel_min_x = std::max(0, std::min(voxel_min_x, grid_x_ - 1));
    voxel_min_y = std::max(0, std::min(voxel_min_y, grid_y_ - 1));
    voxel_min_z = std::max(0, std::min(voxel_min_z, grid_z_ - 1));
    voxel_max_x = std::max(0, std::min(voxel_max_x, grid_x_ - 1));
    voxel_max_y = std::max(0, std::min(voxel_max_y, grid_y_ - 1));
    voxel_max_z = std::max(0, std::min(voxel_max_z, grid_z_ - 1));
    
    int marked_count = 0;
    
    for (int x = voxel_min_x; x <= voxel_max_x; ++x) {
        for (int y = voxel_min_y; y <= voxel_max_y; ++y) {
            for (int z = voxel_min_z; z <= voxel_max_z; ++z) {
                double world_x, world_y, world_z;
                voxelToWorld(x, y, z, world_x, world_y, world_z);
                
                if (entity->isPointInside(world_x, world_y, world_z)) {
                    updateVoxelCost(x, y, z, cost_value);
                    marked_count++;
                }
            }
        }
    }
    
    return marked_count;
}

} // namespace voxelization 