/*
 * Common Types for Path Planning
 * Shared data structures used across different planning algorithms
 */

#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <unordered_map>

// 3D coordinate key for map indexing without hash
struct Coord3D {
    int x, y, z;
    
    Coord3D() : x(0), y(0), z(0) {}
    Coord3D(int x, int y, int z) : x(x), y(y), z(z) {}
    
    // For unordered_map compatibility
    bool operator==(const Coord3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    // For set comparison
    bool operator<(const Coord3D& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

// Hash function for Coord3D
namespace std {
    template<>
    struct hash<Coord3D> {
        size_t operator()(const Coord3D& coord) const {
            // Simple hash function for 3D coordinates
            return coord.x * 73856093 ^ coord.y * 19349663 ^ coord.z * 83492791;
        }
    };
}

#endif // COMMON_TYPES_HPP 