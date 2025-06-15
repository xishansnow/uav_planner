#ifndef SPATIAL_ENTITIES_HPP
#define SPATIAL_ENTITIES_HPP

#include "voxelization_base.hpp"
#include <map>
#include <vector>
#include <memory>

namespace voxelization {

/**
 * @brief Box-shaped spatial entity
 */
class BoxEntity : public SpatialEntity {
public:
    BoxEntity(double center_x, double center_y, double center_z,
              double size_x, double size_y, double size_z);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "box"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    double center_x_, center_y_, center_z_;
    double size_x_, size_y_, size_z_;
};

/**
 * @brief Cylinder-shaped spatial entity
 */
class CylinderEntity : public SpatialEntity {
public:
    CylinderEntity(double center_x, double center_y, double center_z,
                   double radius, double height);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "cylinder"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    double center_x_, center_y_, center_z_;
    double radius_, height_;
};

/**
 * @brief Sphere-shaped spatial entity
 */
class SphereEntity : public SpatialEntity {
public:
    SphereEntity(double center_x, double center_y, double center_z, double radius);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "sphere"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    double center_x_, center_y_, center_z_;
    double radius_;
};

/**
 * @brief Ellipsoid-shaped spatial entity
 */
class EllipsoidEntity : public SpatialEntity {
public:
    EllipsoidEntity(double center_x, double center_y, double center_z,
                    double radius_x, double radius_y, double radius_z);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "ellipsoid"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    double center_x_, center_y_, center_z_;
    double radius_x_, radius_y_, radius_z_;
};

/**
 * @brief Cone-shaped spatial entity
 */
class ConeEntity : public SpatialEntity {
public:
    ConeEntity(double center_x, double center_y, double center_z,
               double radius, double height);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "cone"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    double center_x_, center_y_, center_z_;
    double radius_, height_;
};

/**
 * @brief Mesh-based spatial entity (for complex shapes)
 */
class MeshEntity : public SpatialEntity {
public:
    MeshEntity(const std::vector<Eigen::Vector3d>& vertices,
               const std::vector<std::vector<int>>& faces);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "mesh"; }
    std::map<std::string, double> getProperties() const override;
    
private:
    std::vector<Eigen::Vector3d> vertices_;
    std::vector<std::vector<int>> faces_;
    
    bool rayIntersectsTriangle(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                              const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, 
                              const Eigen::Vector3d& v2) const;
};

/**
 * @brief Composite spatial entity (union of multiple entities)
 */
class CompositeEntity : public SpatialEntity {
public:
    CompositeEntity(const std::vector<std::shared_ptr<SpatialEntity>>& entities);
    
    std::vector<double> getBoundingBox() const override;
    bool isPointInside(double x, double y, double z) const override;
    std::string getType() const override { return "composite"; }
    std::map<std::string, double> getProperties() const override;
    
    void addEntity(const std::shared_ptr<SpatialEntity>& entity);
    void removeEntity(size_t index);
    size_t getEntityCount() const { return entities_.size(); }
    
private:
    std::vector<std::shared_ptr<SpatialEntity>> entities_;
};

/**
 * @brief Factory for creating spatial entities
 */
class SpatialEntityFactory {
public:
    enum class EntityType {
        BOX,
        CYLINDER,
        SPHERE,
        ELLIPSOID,
        CONE,
        MESH,
        COMPOSITE
    };
    
    /**
     * @brief Create a spatial entity from parameters
     * @param type Entity type
     * @param params Entity parameters
     * @return Shared pointer to spatial entity
     */
    static std::shared_ptr<SpatialEntity> createEntity(EntityType type, 
                                                      const std::map<std::string, double>& params);
    
    /**
     * @brief Create a spatial entity from JSON configuration
     * @param json_config JSON configuration string
     * @return Shared pointer to spatial entity
     */
    static std::shared_ptr<SpatialEntity> createFromJSON(const std::string& json_config);
    
    /**
     * @brief Create a spatial entity from file
     * @param filename File path
     * @return Shared pointer to spatial entity
     */
    static std::shared_ptr<SpatialEntity> createFromFile(const std::string& filename);
    
    /**
     * @brief Get available entity types
     * @return Vector of available entity types
     */
    static std::vector<std::string> getAvailableTypes();
};

} // namespace voxelization

#endif // SPATIAL_ENTITIES_HPP 