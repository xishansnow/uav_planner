#include "spatial_entities.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>

namespace voxelization {

// BoxEntity implementation
BoxEntity::BoxEntity(double center_x, double center_y, double center_z,
                     double size_x, double size_y, double size_z)
    : center_x_(center_x), center_y_(center_y), center_z_(center_z),
      size_x_(size_x), size_y_(size_y), size_z_(size_z) {}

std::vector<double> BoxEntity::getBoundingBox() const {
    return {
        center_x_ - size_x_ / 2, center_y_ - size_y_ / 2, center_z_ - size_z_ / 2,
        center_x_ + size_x_ / 2, center_y_ + size_y_ / 2, center_z_ + size_z_ / 2
    };
}

bool BoxEntity::isPointInside(double x, double y, double z) const {
    return (x >= center_x_ - size_x_ / 2 && x <= center_x_ + size_x_ / 2 &&
            y >= center_y_ - size_y_ / 2 && y <= center_y_ + size_y_ / 2 &&
            z >= center_z_ - size_z_ / 2 && z <= center_z_ + size_z_ / 2);
}

std::map<std::string, double> BoxEntity::getProperties() const {
    return {
        {"center_x", center_x_},
        {"center_y", center_y_},
        {"center_z", center_z_},
        {"size_x", size_x_},
        {"size_y", size_y_},
        {"size_z", size_z_}
    };
}

// CylinderEntity implementation
CylinderEntity::CylinderEntity(double center_x, double center_y, double center_z,
                               double radius, double height)
    : center_x_(center_x), center_y_(center_y), center_z_(center_z),
      radius_(radius), height_(height) {}

std::vector<double> CylinderEntity::getBoundingBox() const {
    return {
        center_x_ - radius_, center_y_ - radius_, center_z_ - height_ / 2,
        center_x_ + radius_, center_y_ + radius_, center_z_ + height_ / 2
    };
}

bool CylinderEntity::isPointInside(double x, double y, double z) const {
    double dx = x - center_x_;
    double dy = y - center_y_;
    double dist_sq = dx * dx + dy * dy;
    return (dist_sq <= radius_ * radius_ &&
            z >= center_z_ - height_ / 2 && z <= center_z_ + height_ / 2);
}

std::map<std::string, double> CylinderEntity::getProperties() const {
    return {
        {"center_x", center_x_},
        {"center_y", center_y_},
        {"center_z", center_z_},
        {"radius", radius_},
        {"height", height_}
    };
}

// SphereEntity implementation
SphereEntity::SphereEntity(double center_x, double center_y, double center_z, double radius)
    : center_x_(center_x), center_y_(center_y), center_z_(center_z), radius_(radius) {}

std::vector<double> SphereEntity::getBoundingBox() const {
    return {
        center_x_ - radius_, center_y_ - radius_, center_z_ - radius_,
        center_x_ + radius_, center_y_ + radius_, center_z_ + radius_
    };
}

bool SphereEntity::isPointInside(double x, double y, double z) const {
    double dx = x - center_x_;
    double dy = y - center_y_;
    double dz = z - center_z_;
    double dist_sq = dx * dx + dy * dy + dz * dz;
    return dist_sq <= radius_ * radius_;
}

std::map<std::string, double> SphereEntity::getProperties() const {
    return {
        {"center_x", center_x_},
        {"center_y", center_y_},
        {"center_z", center_z_},
        {"radius", radius_}
    };
}

// EllipsoidEntity implementation
EllipsoidEntity::EllipsoidEntity(double center_x, double center_y, double center_z,
                                 double radius_x, double radius_y, double radius_z)
    : center_x_(center_x), center_y_(center_y), center_z_(center_z),
      radius_x_(radius_x), radius_y_(radius_y), radius_z_(radius_z) {}

std::vector<double> EllipsoidEntity::getBoundingBox() const {
    return {
        center_x_ - radius_x_, center_y_ - radius_y_, center_z_ - radius_z_,
        center_x_ + radius_x_, center_y_ + radius_y_, center_z_ + radius_z_
    };
}

bool EllipsoidEntity::isPointInside(double x, double y, double z) const {
    double dx = (x - center_x_) / radius_x_;
    double dy = (y - center_y_) / radius_y_;
    double dz = (z - center_z_) / radius_z_;
    return (dx * dx + dy * dy + dz * dz) <= 1.0;
}

std::map<std::string, double> EllipsoidEntity::getProperties() const {
    return {
        {"center_x", center_x_},
        {"center_y", center_y_},
        {"center_z", center_z_},
        {"radius_x", radius_x_},
        {"radius_y", radius_y_},
        {"radius_z", radius_z_}
    };
}

// ConeEntity implementation
ConeEntity::ConeEntity(double center_x, double center_y, double center_z,
                       double radius, double height)
    : center_x_(center_x), center_y_(center_y), center_z_(center_z),
      radius_(radius), height_(height) {}

std::vector<double> ConeEntity::getBoundingBox() const {
    return {
        center_x_ - radius_, center_y_ - radius_, center_z_ - height_ / 2,
        center_x_ + radius_, center_y_ + radius_, center_z_ + height_ / 2
    };
}

bool ConeEntity::isPointInside(double x, double y, double z) const {
    double dx = x - center_x_;
    double dy = y - center_y_;
    double dz = z - center_z_;
    
    // Check if point is within height bounds
    if (std::abs(dz) > height_ / 2) return false;
    
    // Calculate radius at current height
    double height_ratio = (dz + height_ / 2) / height_;
    double current_radius = radius_ * height_ratio;
    
    double dist_sq = dx * dx + dy * dy;
    return dist_sq <= current_radius * current_radius;
}

std::map<std::string, double> ConeEntity::getProperties() const {
    return {
        {"center_x", center_x_},
        {"center_y", center_y_},
        {"center_z", center_z_},
        {"radius", radius_},
        {"height", height_}
    };
}

// MeshEntity implementation
MeshEntity::MeshEntity(const std::vector<Eigen::Vector3d>& vertices,
                       const std::vector<std::vector<int>>& faces)
    : vertices_(vertices), faces_(faces) {}

std::vector<double> MeshEntity::getBoundingBox() const {
    if (vertices_.empty()) return {0, 0, 0, 0, 0, 0};
    
    double min_x = vertices_[0].x(), max_x = vertices_[0].x();
    double min_y = vertices_[0].y(), max_y = vertices_[0].y();
    double min_z = vertices_[0].z(), max_z = vertices_[0].z();
    
    for (const auto& vertex : vertices_) {
        min_x = std::min(min_x, vertex.x());
        max_x = std::max(max_x, vertex.x());
        min_y = std::min(min_y, vertex.y());
        max_y = std::max(max_y, vertex.y());
        min_z = std::min(min_z, vertex.z());
        max_z = std::max(max_z, vertex.z());
    }
    
    return {min_x, min_y, min_z, max_x, max_y, max_z};
}

bool MeshEntity::isPointInside(double x, double y, double z) const {
    if (faces_.empty()) return false;
    
    Eigen::Vector3d point(x, y, z);
    Eigen::Vector3d direction(1, 0, 0); // Ray direction
    
    int intersections = 0;
    
    for (const auto& face : faces_) {
        if (face.size() < 3) continue;
        
        Eigen::Vector3d v0 = vertices_[face[0]];
        Eigen::Vector3d v1 = vertices_[face[1]];
        Eigen::Vector3d v2 = vertices_[face[2]];
        
        if (rayIntersectsTriangle(point, direction, v0, v1, v2)) {
            intersections++;
        }
    }
    
    return (intersections % 2) == 1; // Odd number of intersections means inside
}

bool MeshEntity::rayIntersectsTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                                      const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, 
                                      const Eigen::Vector3d& v2) const {
    const double EPSILON = 1e-6;
    
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d h = direction.cross(edge2);
    double a = edge1.dot(h);
    
    if (std::abs(a) < EPSILON) return false; // Ray is parallel to triangle
    
    double f = 1.0 / a;
    Eigen::Vector3d s = origin - v0;
    double u = f * s.dot(h);
    
    if (u < 0.0 || u > 1.0) return false;
    
    Eigen::Vector3d q = s.cross(edge1);
    double v = f * direction.dot(q);
    
    if (v < 0.0 || u + v > 1.0) return false;
    
    double t = f * edge2.dot(q);
    return t > EPSILON;
}

std::map<std::string, double> MeshEntity::getProperties() const {
    return {
        {"vertex_count", static_cast<double>(vertices_.size())},
        {"face_count", static_cast<double>(faces_.size())}
    };
}

// CompositeEntity implementation
CompositeEntity::CompositeEntity(const std::vector<std::shared_ptr<SpatialEntity>>& entities)
    : entities_(entities) {}

std::vector<double> CompositeEntity::getBoundingBox() const {
    if (entities_.empty()) return {0, 0, 0, 0, 0, 0};
    
    auto first_bbox = entities_[0]->getBoundingBox();
    double min_x = first_bbox[0], max_x = first_bbox[3];
    double min_y = first_bbox[1], max_y = first_bbox[4];
    double min_z = first_bbox[2], max_z = first_bbox[5];
    
    for (const auto& entity : entities_) {
        auto bbox = entity->getBoundingBox();
        min_x = std::min(min_x, bbox[0]);
        max_x = std::max(max_x, bbox[3]);
        min_y = std::min(min_y, bbox[1]);
        max_y = std::max(max_y, bbox[4]);
        min_z = std::min(min_z, bbox[2]);
        max_z = std::max(max_z, bbox[5]);
    }
    
    return {min_x, min_y, min_z, max_x, max_y, max_z};
}

bool CompositeEntity::isPointInside(double x, double y, double z) const {
    for (const auto& entity : entities_) {
        if (entity->isPointInside(x, y, z)) {
            return true;
        }
    }
    return false;
}

std::map<std::string, double> CompositeEntity::getProperties() const {
    return {
        {"entity_count", static_cast<double>(entities_.size())}
    };
}

void CompositeEntity::addEntity(const std::shared_ptr<SpatialEntity>& entity) {
    entities_.push_back(entity);
}

void CompositeEntity::removeEntity(size_t index) {
    if (index < entities_.size()) {
        entities_.erase(entities_.begin() + index);
    }
}

// SpatialEntityFactory implementation
std::shared_ptr<SpatialEntity> SpatialEntityFactory::createEntity(EntityType type,
                                                                 const std::map<std::string, double>& params) {
    switch (type) {
        case EntityType::BOX: {
            auto it_x = params.find("center_x");
            auto it_y = params.find("center_y");
            auto it_z = params.find("center_z");
            auto it_sx = params.find("size_x");
            auto it_sy = params.find("size_y");
            auto it_sz = params.find("size_z");
            
            if (it_x != params.end() && it_y != params.end() && it_z != params.end() &&
                it_sx != params.end() && it_sy != params.end() && it_sz != params.end()) {
                return std::make_shared<BoxEntity>(it_x->second, it_y->second, it_z->second,
                                                  it_sx->second, it_sy->second, it_sz->second);
            }
            break;
        }
        
        case EntityType::CYLINDER: {
            auto it_x = params.find("center_x");
            auto it_y = params.find("center_y");
            auto it_z = params.find("center_z");
            auto it_r = params.find("radius");
            auto it_h = params.find("height");
            
            if (it_x != params.end() && it_y != params.end() && it_z != params.end() &&
                it_r != params.end() && it_h != params.end()) {
                return std::make_shared<CylinderEntity>(it_x->second, it_y->second, it_z->second,
                                                       it_r->second, it_h->second);
            }
            break;
        }
        
        case EntityType::SPHERE: {
            auto it_x = params.find("center_x");
            auto it_y = params.find("center_y");
            auto it_z = params.find("center_z");
            auto it_r = params.find("radius");
            
            if (it_x != params.end() && it_y != params.end() && it_z != params.end() &&
                it_r != params.end()) {
                return std::make_shared<SphereEntity>(it_x->second, it_y->second, it_z->second, it_r->second);
            }
            break;
        }
        
        case EntityType::ELLIPSOID: {
            auto it_x = params.find("center_x");
            auto it_y = params.find("center_y");
            auto it_z = params.find("center_z");
            auto it_rx = params.find("radius_x");
            auto it_ry = params.find("radius_y");
            auto it_rz = params.find("radius_z");
            
            if (it_x != params.end() && it_y != params.end() && it_z != params.end() &&
                it_rx != params.end() && it_ry != params.end() && it_rz != params.end()) {
                return std::make_shared<EllipsoidEntity>(it_x->second, it_y->second, it_z->second,
                                                        it_rx->second, it_ry->second, it_rz->second);
            }
            break;
        }
        
        case EntityType::CONE: {
            auto it_x = params.find("center_x");
            auto it_y = params.find("center_y");
            auto it_z = params.find("center_z");
            auto it_r = params.find("radius");
            auto it_h = params.find("height");
            
            if (it_x != params.end() && it_y != params.end() && it_z != params.end() &&
                it_r != params.end() && it_h != params.end()) {
                return std::make_shared<ConeEntity>(it_x->second, it_y->second, it_z->second,
                                                   it_r->second, it_h->second);
            }
            break;
        }
        
        default:
            break;
    }
    
    return nullptr;
}

std::shared_ptr<SpatialEntity> SpatialEntityFactory::createFromJSON(const std::string& json_config) {
    // Simple JSON parser for entity creation
    // This is a basic implementation - in production, use a proper JSON library like nlohmann/json
    
    std::map<std::string, double> params;
    std::string type;
    
    // Parse JSON-like format: {"type": "box", "center_x": 1.0, "center_y": 2.0, ...}
    std::istringstream iss(json_config);
    std::string line;
    
    while (std::getline(iss, line)) {
        // Remove whitespace and quotes
        line.erase(std::remove_if(line.begin(), line.end(), ::isspace), line.end());
        line.erase(std::remove(line.begin(), line.end(), '"'), line.end());
        
        if (line.find("type:") != std::string::npos) {
            size_t pos = line.find(":");
            if (pos != std::string::npos) {
                type = line.substr(pos + 1);
            }
        } else if (line.find(":") != std::string::npos) {
            size_t pos = line.find(":");
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                try {
                    params[key] = std::stod(value);
                } catch (...) {
                    // Ignore parsing errors
                }
            }
        }
    }
    
    // Create entity based on type
    if (type == "box") {
        return createEntity(EntityType::BOX, params);
    } else if (type == "cylinder") {
        return createEntity(EntityType::CYLINDER, params);
    } else if (type == "sphere") {
        return createEntity(EntityType::SPHERE, params);
    } else if (type == "ellipsoid") {
        return createEntity(EntityType::ELLIPSOID, params);
    } else if (type == "cone") {
        return createEntity(EntityType::CONE, params);
    }
    
    return nullptr;
}

std::shared_ptr<SpatialEntity> SpatialEntityFactory::createFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return nullptr;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    return createFromJSON(content);
}

std::vector<std::string> SpatialEntityFactory::getAvailableTypes() {
    return {"box", "cylinder", "sphere", "ellipsoid", "cone", "mesh", "composite"};
}

} // namespace voxelization 